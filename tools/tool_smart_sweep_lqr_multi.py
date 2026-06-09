#!/usr/bin/env python3
"""
tools/tool_smart_sweep_lqr_multi.py - multi-track variant of
tool_smart_sweep_lqr.py.

Identical adaptive random->converge search and robust (worst-of-N perturbed
neighbour) scoring, but every candidate is evaluated on EACH track in TRACKS and
scored by its WORST track. The result is therefore a single shared gain set that
keeps BOTH tracks clean (0 off-track, completed lap, robust to +/-3% jitter) and
minimises the slower of the two lap times - the min-max objective.

The tracks are selected via the TRACK environment variable, which
track_parser.c reads in track_init(); the eval binary needs no other change.

Usage (repo root, MinGW gcc on PATH, TMPDIR set to a writable temp):
    python tools/tool_smart_sweep_lqr_multi.py --trials 150 --robust 3
"""
import argparse, json, os, random, subprocess, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "HIL_Firmware", "build")
OUT = os.path.join(BUILD, "eval_smartlqr_multi.exe")

SRCS = ["tools/tool_eval_lap.c", "HIL_Firmware/src/motion_control.c",
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track_parser.c",
        "HIL_Firmware/src/path_planning.c", "HIL_Firmware/src/lqr_steer.c",
        "ECU_Firmware/src/torque_vectoring.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]
EXTRA_DEFS = []

# Tracks to co-optimise. Names must match the tracks/*.yaml layout names.
TRACKS = ["fsg2024", "fse2024"]

# (name, low, high). Same high-leverage set as the single-track sweeper.
# Ceilings on the grip-budget levers raised after a tyre/downforce-model probe
# showed peak lateral accel is ~14.5 at the hairpin but 18-21+ in the fast
# corners. The headroom is in USING the downforce (PLANNER_DOWNFORCE_FRAC up to
# 1.0) and shaping the line for it (PP_GRIP_ACCEL), not in the low-speed base
# (MAX_LATERAL_ACCEL_MS2, which the hairpin already pins).
PARAMS = [
    ("MAX_LATERAL_ACCEL_MS2", 12.0, 15.5),
    ("PP_GRIP_ACCEL",         10.0, 18.0),
    ("RACING_MARGIN",          0.18, 0.34),
    ("PP_MIN_RADIUS_M",        4.5,  6.5),
    ("GG_ACCEL_MS2",           6.0,  13.0),
    ("PLANNER_DOWNFORCE_FRAC", 0.2,  1.0),
    ("LQR_Q_E1",              10.0, 40.0),
    ("LQR_Q_E1D",              0.3,  3.0),
    ("LQR_Q_E2",               3.0, 16.0),
    ("LQR_Q_E2D",              0.1,  1.0),
    ("LQR_R",                  2.0,  8.0),
    ("LQR_KI",                 2.0, 10.0),
    ("LQR_I_MAX",              0.3,  1.0),
    ("LAT_GRIP_REF_MS2",      11.0, 20.0),
    ("KP_YAW_DEFAULT",        40.0, 90.0),
    ("TV_KFF",                 6.0, 18.0),
    ("TV_REAR_SHARE",          0.45, 0.70),
]
NAMES = [p[0] for p in PARAMS]
LO = {p[0]: p[1] for p in PARAMS}
HI = {p[0]: p[2] for p in PARAMS}

# Starting incumbent: the committed in-source defaults (the grip-aware
# distributor's tuned set: fsg2024 ~25.1s, fse2024 ~18.7s, both clean and
# robust to +/-3%; mean CTE ~0.14/0.20).
DEFAULTS = {
    "MAX_LATERAL_ACCEL_MS2": 12.6154, "PP_GRIP_ACCEL": 10.0000,
    "RACING_MARGIN": 0.2587, "PP_MIN_RADIUS_M": 6.0329, "GG_ACCEL_MS2": 8.7898,
    "PLANNER_DOWNFORCE_FRAC": 0.5670,
    "LQR_Q_E1": 10.0000, "LQR_Q_E1D": 1.0432, "LQR_Q_E2": 9.7510,
    "LQR_Q_E2D": 0.4511, "LQR_R": 2.8632, "LQR_KI": 7.9193, "LQR_I_MAX": 0.3000,
    "LAT_GRIP_REF_MS2": 16.5044, "KP_YAW_DEFAULT": 86.2440, "TV_KFF": 10.3635,
    "TV_REAR_SHARE": 0.4894,
}

ROBUST_PCT = 0.03
ROBUST_N = 3


def clamp(n, v): return max(LO[n], min(HI[n], v))


def build(cfg):
    defs = EXTRA_DEFS + ["-D%s=%.5ff" % (n, cfg[n]) for n in NAMES]
    r = subprocess.run(["gcc", "-std=c11", "-O2", *INC, *defs, "-o", OUT,
                        *SRCS, "-lm"], cwd=ROOT, capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def run_track(track):
    env = dict(os.environ)
    env["TRACK"] = track
    r = subprocess.run([OUT], cwd=ROOT, capture_output=True, text=True, env=env)
    for line in r.stdout.splitlines():
        if line.startswith("RESULT"):
            d = {}
            for kv in line[len("RESULT "):].split():
                k, v = kv.split("=")
                d[k] = float(v) if "." in v or "-" in v else int(v)
            return d
    return None


def track_score(res):
    """Per-track score: lap time if clean, else a large penalty."""
    if res is None:
        return 1e9
    off = res.get("offtrack", 9999)
    laps = res.get("laps", 0)
    lap_s = res.get("lap_s", -1)
    if laps >= 1 and off == 0 and lap_s > 0:
        return lap_s
    pen = 1000.0 + off * 0.5 + res.get("worst_cte", 9.9) * 50.0
    if laps < 1:
        pen += 500.0
    return pen


def evaluate(cfg):
    """Build once, run every track. Returns ({track: res}, combined_score).
    Combined score is the WORST (max) per-track score - the min-max objective."""
    ok, _ = build(cfg)
    if not ok:
        return None, 1e9
    results, worst = {}, 0.0
    for t in TRACKS:
        res = run_track(t)
        results[t] = res
        worst = max(worst, track_score(res))
    return results, worst


def robust_evaluate(cfg, rng):
    results, base = evaluate(cfg)
    if results is None:
        return None, 1e9
    worst = base
    for _ in range(ROBUST_N):
        c = {n: clamp(n, cfg[n] * (1 + rng.uniform(-ROBUST_PCT, ROBUST_PCT)))
             for n in NAMES}
        worst = max(worst, evaluate(c)[1])
    return results, worst


def resstr(results):
    if results is None:
        return "(build/run failed)"
    parts = []
    for t in TRACKS:
        r = results.get(t)
        if r is None:
            parts.append("%s:(fail)" % t)
        else:
            parts.append("%s:lap=%s off=%s laps=%s wCTE=%.2f"
                         % (t, r.get("lap_s"), r.get("offtrack"),
                            r.get("laps"), r.get("worst_cte", -1)))
    return "  ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=150)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--frac-random", type=float, default=0.4)
    ap.add_argument("--robust", type=int, default=3)
    args = ap.parse_args()
    global ROBUST_N
    ROBUST_N = args.robust
    random.seed(args.seed)
    rng = random.Random(args.seed + 999)
    os.makedirs(BUILD, exist_ok=True)

    t0 = time.time()
    inc_cfg = dict(DEFAULTS)
    if ROBUST_N > 0:
        inc_res, inc_score = robust_evaluate(inc_cfg, rng)
    else:
        inc_res, inc_score = evaluate(inc_cfg)
    print("baseline  score=%.3f  %s" % (inc_score, resstr(inc_res)), flush=True)

    n_rand = int(args.trials * args.frac_random)
    for i in range(args.trials):
        if i < n_rand:
            cfg = {n: random.uniform(LO[n], HI[n]) for n in NAMES}
            kind = "rand"
        else:
            prog = (i - n_rand) / max(1, args.trials - n_rand)
            radius = 0.30 * (1 - prog) + 0.05 * prog
            cfg = {n: clamp(n, inc_cfg[n] + random.gauss(0, radius * (HI[n]-LO[n])))
                   for n in NAMES}
            kind = "ref%.2f" % radius
        if ROBUST_N > 0:
            res, s = robust_evaluate(cfg, rng)
        else:
            res, s = evaluate(cfg)
        flag = ""
        if s < inc_score - 1e-6:
            inc_score, inc_cfg, inc_res = s, dict(cfg), res
            flag = "  <== NEW BEST"
        print("[%3d/%d %-7s] score=%8.3f %s%s"
              % (i+1, args.trials, kind, s, resstr(res), flag), flush=True)

    dt = time.time() - t0
    print("\n================ BEST (shared, both tracks) ================")
    print("worst-track worst-neighbour score = %.3f" % inc_score)
    print(resstr(inc_res))
    print(" ".join("%s=%.3f" % (n, inc_cfg[n]) for n in NAMES))
    print("\n-D flags:")
    print(" ".join("-D%s=%.4ff" % (n, inc_cfg[n]) for n in NAMES))
    with open(os.path.join(BUILD, "best_lqr_multi.json"), "w") as f:
        json.dump(inc_cfg, f, indent=1)
    print("\n(%d trials in %.1fs, %.2fs/trial)  -> %s"
          % (args.trials, dt, dt/max(1, args.trials),
             os.path.join("HIL_Firmware/build", "best_lqr_multi.json")))


if __name__ == "__main__":
    main()
