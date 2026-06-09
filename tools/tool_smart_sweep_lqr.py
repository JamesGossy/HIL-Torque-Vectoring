#!/usr/bin/env python3
"""
tools/tool_smart_sweep_lqr.py - adaptive, robustness-aware parameter optimiser for
the (LQR) steering + speed-planner + TV tuning.

An adaptive random-explore -> converge search that links lqr_steer.c and sweeps
the high-leverage parameter set: the path-planner / speed knobs the tight tracker
needs (RACING_MARGIN, PP_MIN_RADIUS_M, PP_GRIP_ACCEL, MAX_LATERAL_ACCEL_MS2,
GG_ACCEL_MS2), the LQR cost weights (Q/R, KI, I_MAX), and the throttle / TV
gains. The tuned values it produces are committed as the in-source #ifndef
defaults, so a plain `make eval` reproduces the result with no -D overrides.

Robust scoring is ON by default (--robust 3): each candidate is judged by its
WORST of N perturbed neighbours, so the optimiser avoids knife-edge configs that
are clean only at the exact point (see the Pure-Pursuit finding in the repo
history - the fastest "clean" PP lap failed 20/20 perturbation jitters).

Usage (repo root, MinGW gcc on PATH, TMPDIR set to a writable temp):
    python tools/tool_smart_sweep_lqr.py --trials 150 --robust 3
"""
import argparse, json, os, random, subprocess, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "HIL_Firmware", "build")
OUT = os.path.join(BUILD, "eval_smartlqr.exe")

SRCS = ["tools/tool_eval_lap.c", "HIL_Firmware/src/motion_control.c",
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track.c",
        "HIL_Firmware/src/path_planning.c", "HIL_Firmware/src/lqr_steer.c",
        "ECU_Firmware/src/torque_vectoring.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]
EXTRA_DEFS = []   # LQR steering is unconditional; no enabling define needed

# (name, low, high). Bounds bracket the recommended companion settings.
PARAMS = [
    # path planner / speed (the line the tight tracker needs)
    ("MAX_LATERAL_ACCEL_MS2", 10.0, 14.5),
    ("PP_GRIP_ACCEL",         10.0, 14.5),
    ("RACING_MARGIN",          0.18, 0.34),
    ("PP_MIN_RADIUS_M",        4.5,  6.5),
    ("GG_ACCEL_MS2",           6.0,  10.0),
    # LQR cost weights (the controller's own tuning)
    ("LQR_Q_E1",              10.0, 40.0),
    ("LQR_Q_E1D",              0.3,  3.0),
    ("LQR_Q_E2",               3.0, 16.0),
    ("LQR_Q_E2D",              0.1,  1.0),
    ("LQR_R",                  2.0,  8.0),
    ("LQR_KI",                 2.0, 10.0),
    ("LQR_I_MAX",              0.3,  1.0),
    # throttle / TV
    ("LAT_GRIP_REF_MS2",      11.0, 16.0),
    ("KP_YAW_DEFAULT",        40.0, 90.0),
    ("TV_KFF",                 6.0, 18.0),
    ("TV_REAR_SHARE",          0.45, 0.70),
]
NAMES = [p[0] for p in PARAMS]
LO = {p[0]: p[1] for p in PARAMS}
HI = {p[0]: p[2] for p in PARAMS}

# Recommended companion config (clean 27.03 s) as the starting incumbent.
DEFAULTS = {
    "MAX_LATERAL_ACCEL_MS2": 13.0, "PP_GRIP_ACCEL": 13.0, "RACING_MARGIN": 0.26,
    "PP_MIN_RADIUS_M": 5.5, "GG_ACCEL_MS2": 8.0,
    "LQR_Q_E1": 20.0, "LQR_Q_E1D": 1.0, "LQR_Q_E2": 8.0, "LQR_Q_E2D": 0.3,
    "LQR_R": 4.0, "LQR_KI": 5.0, "LQR_I_MAX": 0.6,
    "LAT_GRIP_REF_MS2": 15.0, "KP_YAW_DEFAULT": 60.0, "TV_KFF": 12.0,
    "TV_REAR_SHARE": 0.6,
}

ROBUST_PCT = 0.03
ROBUST_N = 3


def clamp(n, v): return max(LO[n], min(HI[n], v))


def build(cfg):
    defs = EXTRA_DEFS + ["-D%s=%.5ff" % (n, cfg[n]) for n in NAMES]
    r = subprocess.run(["gcc", "-std=c11", "-O2", *INC, *defs, "-o", OUT,
                        *SRCS, "-lm"], cwd=ROOT, capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def run():
    r = subprocess.run([OUT], cwd=ROOT, capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if line.startswith("RESULT"):
            d = {}
            for kv in line[len("RESULT "):].split():
                k, v = kv.split("=")
                d[k] = float(v) if "." in v or "-" in v else int(v)
            return d
    return None


def score(res):
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
    ok, err = build(cfg)
    return (run(), "") if ok else (None, err)


def robust_evaluate(cfg, rng):
    res, _ = evaluate(cfg)
    if res is None:
        return None, 1e9
    worst = score(res)
    for _ in range(ROBUST_N):
        c = {n: clamp(n, cfg[n] * (1 + rng.uniform(-ROBUST_PCT, ROBUST_PCT)))
             for n in NAMES}
        worst = max(worst, score(evaluate(c)[0]))
    return res, worst


def resstr(res):
    if res is None:
        return "(build/run failed)"
    return ("lap_s=%s off=%s laps=%s meanCTE=%.3f worstCTE=%.3f"
            % (res.get("lap_s"), res.get("offtrack"), res.get("laps"),
               res.get("mean_cte", -1), res.get("worst_cte", -1)))


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
        inc_res = evaluate(inc_cfg)[0]
        inc_score = score(inc_res)
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
            res, _ = evaluate(cfg)
            s = score(res)
        flag = ""
        if s < inc_score - 1e-6:
            inc_score, inc_cfg, inc_res = s, dict(cfg), res
            flag = "  <== NEW BEST"
        print("[%3d/%d %-7s] score=%8.3f %s%s"
              % (i+1, args.trials, kind, s, resstr(res), flag), flush=True)

    dt = time.time() - t0
    print("\n==================== BEST (LQR) ====================")
    print("worst-neighbour score = %.3f" % inc_score)
    print(resstr(inc_res))
    print(" ".join("%s=%.3f" % (n, inc_cfg[n]) for n in NAMES))
    print("\n-D flags:")
    print(" ".join("-D%s=%.4ff" % (n, inc_cfg[n]) for n in NAMES))
    with open(os.path.join(BUILD, "best_lqr.json"), "w") as f:
        json.dump(inc_cfg, f, indent=1)
    print("\n(%d trials in %.1fs, %.2fs/trial)  -> %s"
          % (args.trials, dt, dt/max(1, args.trials),
             os.path.join("HIL_Firmware/build", "best_lqr.json")))


if __name__ == "__main__":
    main()
