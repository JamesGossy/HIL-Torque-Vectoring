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
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track_parser.c",
        "HIL_Firmware/src/path_planning.c",
        "ECU_Firmware/src/torque_vectoring.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]
EXTRA_DEFS = []   # steering is unconditional; no enabling define needed

# (name, low, high). This single-track tool sweeps the HIGH-LEVERAGE head only,
# so it stays fast; the full ~25-gain space is in tool_smart_sweep_lqr_multi.py.
# Add any other tunable here to include it (every gain has a TUNE_* override).
PARAMS = [
    ("GRIP_USE",         0.75,  1.00),  # fraction of physical peak lateral grip
    ("K_STANLEY",        2.0,  14.0),   # Stanley steering cross-track gain
    ("K_DAMP",           0.0,   1.0),   # Stanley yaw-rate damping gain
    ("RACING_MARGIN",    0.30,  0.50),  # racing-line corridor safety margin
    ("PP_RADIUS_FACTOR", 1.2,   2.2),   # racing-line radius-floor opening
    ("MAX_STEER_RAD",    1.3,   2.0),   # steering reference limit
    ("MAX_BRAKE_DECEL_MS2", 4.0, 8.5),  # braking-effort cap
    ("SPEED_KP_NM",    400.0, 1200.0),  # throttle P-gain
    ("KP_YAW",          40.0,  90.0),   # master torque-vectoring yaw gain
]
INT_PARAMS = {"SPEED_PLAN_STEPS", "NEAREST_SEARCH_FWD", "NEAREST_SEARCH_BACK"}
NAMES = [p[0] for p in PARAMS]
LO = {p[0]: p[1] for p in PARAMS}
HI = {p[0]: p[2] for p in PARAMS}

# Starting incumbent: the committed in-source defaults (shared/tunables.c). Full
# set so any DEFAULT carries through even for gains not in PARAMS above. Keep in
# sync with tunables.c.
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
