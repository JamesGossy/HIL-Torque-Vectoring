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
        "HIL_Firmware/src/path_planning.c",
        "ECU_Firmware/src/torque_vectoring.c", "shared/tunables.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]

# Tracks to co-optimise. Names must match the tracks/*.yaml layout names.
TRACKS = ["fsg2024", "fse2024"]

# (name, low, high). Every performance/behaviour gain is swept; only the two
# genuine non-knobs (TV yaw deadband, the empirical TV understeer term) and the
# measurable/derived physical constants (vehicle_config.h) are excluded. Integer
# search depths are swept too (rounded on apply). This is a large space (~25 dims),
# so a thorough run takes a while; trim PARAMS to the high-leverage head (grip /
# steering / margin / radius / longitudinal / TV master) for a fast pass.
PARAMS = [
    # grip / steering / racing line
    ("GRIP_USE",         0.75,  1.00),
    ("K_STANLEY",        2.0,  14.0),
    ("K_DAMP",           0.0,   1.0),
    ("RACING_MARGIN",    0.30,  0.50),
    ("PP_RADIUS_FACTOR", 1.2,   2.2),
    ("MAX_STEER_RAD",    1.3,   2.0),
    ("MAX_STEER_RATE_RADS", 5.0, 12.0),
    ("STEER_SAT_FRAC",   0.5,   0.9),
    # speed planner / longitudinal
    ("SPEED_PLAN_HORIZON_M", 50.0, 110.0),
    ("SPEED_PLAN_STEPS",  24.0,  56.0),   # rounded to int on apply
    ("MAX_BRAKE_DECEL_MS2", 4.0,  8.5),
    ("SPEED_KP_NM",     400.0, 1200.0),
    ("BRAKE_KP_NM",       8.0,  30.0),
    ("SPEED_KI_NM",     150.0,  700.0),
    ("SPEED_I_MAX_NM",  120.0,  400.0),
    ("NEAREST_SEARCH_FWD", 16.0, 44.0),   # rounded to int on apply
    # cone boundary safety net
    ("BOUNDARY_CORR_GAIN",   0.15, 0.45),
    ("BOUNDARY_SLOW_FACTOR", 0.4,  0.8),
    # torque vectoring
    ("KP_YAW",          40.0,  90.0),
    ("TV_KI_FRAC",       1.0,   4.0),
    ("TV_KD_FRAC",       0.0,   0.2),
    ("TV_KFF_FRAC",      0.0,   0.3),
    ("TV_I_MAX_FRAC",    0.2,   0.6),
    ("TV_SPEED_REF_MS",  8.0,  16.0),
    ("TV_WHEEL_YAW_TRUST", 0.0, 0.5),
]
# Names the binary reads as ints (the eval applies them via TUNE_*; getenvi rounds).
INT_PARAMS = {"SPEED_PLAN_STEPS", "NEAREST_SEARCH_FWD", "NEAREST_SEARCH_BACK"}
NAMES = [p[0] for p in PARAMS]
LO = {p[0]: p[1] for p in PARAMS}
HI = {p[0]: p[2] for p in PARAMS}

# Starting incumbent: the committed in-source defaults (shared/tunables.c).
DEFAULTS = {
    "GRIP_USE": 0.90, "K_STANLEY": 8.0, "K_DAMP": 0.30, "RACING_MARGIN": 0.40,
    "PP_RADIUS_FACTOR": 1.6, "MAX_STEER_RAD": 1.7, "MAX_STEER_RATE_RADS": 8.0,
    "STEER_SAT_FRAC": 0.7, "SPEED_PLAN_HORIZON_M": 80.0, "SPEED_PLAN_STEPS": 40,
    "MAX_BRAKE_DECEL_MS2": 5.6, "SPEED_KP_NM": 800.0, "BRAKE_KP_NM": 16.2,
    "SPEED_KI_NM": 400.0, "SPEED_I_MAX_NM": 250.0, "NEAREST_SEARCH_FWD": 30,
    "BOUNDARY_CORR_GAIN": 0.30, "BOUNDARY_SLOW_FACTOR": 0.6,
    "KP_YAW": 86.2440, "TV_KI_FRAC": 2.5, "TV_KD_FRAC": 0.05, "TV_KFF_FRAC": 0.12,
    "TV_I_MAX_FRAC": 0.408, "TV_SPEED_REF_MS": 12.0, "TV_WHEEL_YAW_TRUST": 0.25,
}

ROBUST_PCT = 0.03
ROBUST_N = 3


def clamp(n, v): return max(LO[n], min(HI[n], v))


def build_once():
    """Compile the eval binary a SINGLE time. The gains are applied at runtime via
    TUNE_* env vars (shared/tunables.c), so we never recompile per candidate -
    that turned ~86% of the old sweep's wall-time (a full gcc rebuild per
    candidate) into nothing. The binary reads the env each run."""
    r = subprocess.run(["gcc", "-std=c11", "-O2", *INC, "-o", OUT, *SRCS, "-lm"],
                       cwd=ROOT, capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def cfg_env(cfg):
    """Environment carrying this candidate's gains as TUNE_* overrides."""
    env = dict(os.environ)
    for n in NAMES:
        env["TUNE_%s" % n] = "%.6f" % cfg[n]
    return env


def run_track(track, env):
    env = dict(env)
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
    """Run every track with this candidate's gains applied via env (no rebuild).
    Returns ({track: res}, combined_score). Combined score is the WORST (max)
    per-track score - the min-max objective."""
    env = cfg_env(cfg)
    results, worst = {}, 0.0
    for t in TRACKS:
        res = run_track(t, env)
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

    # Compile the eval binary ONCE; every candidate is then just a run with TUNE_*
    # env overrides (shared/tunables.c), not a recompile.
    ok, err = build_once()
    if not ok:
        print("BUILD FAILED:\n" + err)
        return

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
