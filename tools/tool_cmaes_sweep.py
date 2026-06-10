#!/usr/bin/env python3
"""
tools/tool_cmaes_sweep.py - the gain optimiser (single- and multi-track).

CMA-ES (Covariance Matrix Adaptation Evolution Strategy) learns the shape of
the search landscape by adapting a covariance matrix across generations. It
handles correlated gains (e.g. GRIP_USE and PP_RADIUS_FACTOR move together),
noisy objectives, and high-dimensional spaces far better than random search.
This is the project's tuner; it replaced the earlier adaptive random→converge
sweeps (the removed tool_smart_sweep_lqr*.py).

Each candidate is scored by its WORST ±3% perturbed neighbour across every track
in --tracks, so the search finds a config in a clean basin rather than a
knife-edge that only laps cleanly at the exact point. Each generation evaluates
a population in parallel (one thread per candidate × track × robust-neighbour),
then CMA-ES updates its distribution toward the best performers. Bounds are
enforced by reflection. Always confirm 0 off-track with `make eval` on every
track after applying a result.

Usage (repo root, MSYS2 ucrt64 Python, TMPDIR set):
    # shared two-track set (default):
    python tools/tool_cmaes_sweep.py --generations 200 --popsize 16 --robust 3
    # single layout:
    python tools/tool_cmaes_sweep.py --tracks fsg2024

Install dependency once:
    pip install cma
"""
import argparse, json, os, random, subprocess, time
from concurrent.futures import ThreadPoolExecutor, as_completed
import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    import cma
    CMAES = cma.purecma.CMAES

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "HIL_Firmware", "build")
OUT = os.path.join(BUILD, "eval_cmaes.exe")

SRCS = ["tools/tool_eval_lap.c", "HIL_Firmware/src/motion_control.c",
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track_parser.c",
        "HIL_Firmware/src/path_planning.c",
        "ECU_Firmware/src/torque_vectoring.c", "shared/tunables.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]

TRACKS = ["fsg2024", "fse2024"]

PARAMS = [
    ("GRIP_USE",             0.75,  1.00),
    ("K_STANLEY",            2.0,  14.0),
    ("K_DAMP",               0.0,   1.0),
    ("RACING_MARGIN",        0.30,  0.50),
    ("PP_RADIUS_FACTOR",     0.7,   2.2),
    ("MAX_STEER_RAD",        1.3,   2.5),
    ("MAX_STEER_RATE_RADS",  5.0,  12.0),
    ("STEER_SAT_FRAC",       0.5,   0.9),
    ("SPEED_PLAN_HORIZON_M", 20.0, 110.0),
    ("SPEED_PLAN_STEPS",     20.0,  56.0),
    ("MAX_BRAKE_DECEL_MS2",  1.5,   8.5),
    ("SPEED_KP_NM",         400.0, 1200.0),
    ("BRAKE_KP_NM",           8.0,  30.0),
    ("SPEED_KI_FRAC",         0.2,   1.0),
    ("SPEED_I_MAX_NM",       120.0, 400.0),
    ("NEAREST_SEARCH_FWD",   16.0,  44.0),
    ("KP_YAW",               40.0,  90.0),
    ("TV_KI_FRAC",            1.0,   4.0),
    ("TV_KD_FRAC",            0.0,   0.2),
    ("TV_KFF_FRAC",           0.0,   0.3),
    ("TV_I_MAX_FRAC",         0.2,   0.6),
]
NAMES = [p[0] for p in PARAMS]
LO    = {p[0]: p[1] for p in PARAMS}
HI    = {p[0]: p[2] for p in PARAMS}

# current in-source defaults, CMA-ES starts here
DEFAULTS = {
    "GRIP_USE": 0.993, "K_STANLEY": 4.766, "K_DAMP": 0.334, "RACING_MARGIN": 0.375,
    "PP_RADIUS_FACTOR": 0.936, "MAX_STEER_RAD": 2.408, "MAX_STEER_RATE_RADS": 8.632,
    "STEER_SAT_FRAC": 0.668, "SPEED_PLAN_HORIZON_M": 98.970, "SPEED_PLAN_STEPS": 37,
    "MAX_BRAKE_DECEL_MS2": 3.887, "SPEED_KP_NM": 1187.606, "BRAKE_KP_NM": 13.504,
    "SPEED_KI_FRAC": 0.829, "SPEED_I_MAX_NM": 313.801, "NEAREST_SEARCH_FWD": 41,
    "KP_YAW": 66.208, "TV_KI_FRAC": 1.542, "TV_KD_FRAC": 0.127, "TV_KFF_FRAC": 0.133,
    "TV_I_MAX_FRAC": 0.511,
}

ROBUST_PCT = 0.03
ROBUST_N   = 3


def clamp(n, v):
    return max(LO[n], min(HI[n], v))


def reflect(n, v):
    """Reflect v back into [LO, HI] so CMA-ES does not drift out of bounds."""
    lo, hi = LO[n], HI[n]
    r = hi - lo
    v = v - lo
    v = v % (2 * r)
    if v > r:
        v = 2 * r - v
    return lo + v


def vec_to_cfg(x):
    """Unnormalise a CMA-ES vector (each dim in [0,1]) to physical gains."""
    return {n: reflect(n, LO[n] + x[i] * (HI[n] - LO[n])) for i, n in enumerate(NAMES)}


def cfg_to_vec(cfg):
    """Normalise a cfg dict to [0,1] per dimension for CMA-ES."""
    return [(cfg[n] - LO[n]) / (HI[n] - LO[n]) for n in NAMES]


def build_once():
    r = subprocess.run(["gcc", "-std=c11", "-O2", *INC, "-o", OUT, *SRCS, "-lm"],
                       cwd=ROOT, capture_output=True, text=True)
    return r.returncode == 0, r.stderr


def run_track(track, env):
    e = dict(env); e["TRACK"] = track
    r = subprocess.run([OUT], cwd=ROOT, capture_output=True, text=True, env=e)
    for line in r.stdout.splitlines():
        if line.startswith("RESULT"):
            d = {}
            for kv in line[len("RESULT "):].split():
                k, v = kv.split("=")
                d[k] = float(v) if "." in v or "-" in v else int(v)
            return d
    return None


def track_score(res):
    if res is None:
        return 1e9
    off   = res.get("offtrack", 9999)
    laps  = res.get("laps", 0)
    lap_s = res.get("lap_s", -1)
    if laps >= 1 and off == 0 and lap_s > 0:
        return lap_s
    pen = 1000.0 + off * 0.5 + res.get("worst_cte", 9.9) * 50.0
    if laps < 1:
        pen += 500.0
    return pen


def evaluate_cfg(cfg):
    """Evaluate one cfg on all tracks in parallel. Returns (results_dict, worst_score)."""
    env = dict(os.environ)
    for n in NAMES:
        env["TUNE_%s" % n] = "%.6f" % cfg[n]
    results, worst = {}, 0.0
    with ThreadPoolExecutor(max_workers=len(TRACKS)) as ex:
        futs = {ex.submit(run_track, t, env): t for t in TRACKS}
        for fut in as_completed(futs):
            t = futs[fut]
            res = fut.result()
            results[t] = res
            worst = max(worst, track_score(res))
    return results, worst


def robust_score(cfg, rng):
    """Worst-of-(1+ROBUST_N) score: base + N perturbed neighbours, all parallel."""
    # Build all configs at once
    configs = [cfg] + [
        {n: clamp(n, cfg[n] * (1 + rng.uniform(-ROBUST_PCT, ROBUST_PCT))) for n in NAMES}
        for _ in range(ROBUST_N)
    ]
    envs = []
    for c in configs:
        env = dict(os.environ)
        for n in NAMES:
            env["TUNE_%s" % n] = "%.6f" % c[n]
        envs.append(env)

    jobs = [(ci, t) for ci in range(len(configs)) for t in TRACKS]
    with ThreadPoolExecutor(max_workers=len(jobs)) as ex:
        futs = {ex.submit(run_track, t, envs[ci]): (ci, t) for ci, t in jobs}
        track_results = {}
        for fut in as_completed(futs):
            ci, t = futs[fut]
            track_results[(ci, t)] = fut.result()

    base_results = {t: track_results.get((0, t)) for t in TRACKS}
    worst = 0.0
    for ci in range(len(configs)):
        w = max(track_score(track_results.get((ci, t))) for t in TRACKS)
        worst = max(worst, w)
    return base_results, worst


def resstr(results):
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
    ap.add_argument("--generations", type=int, default=150,
                    help="CMA-ES generations (default 150)")
    ap.add_argument("--popsize", type=int, default=0,
                    help="population per generation (default: CMA-ES auto ~4+3*ln(N))")
    ap.add_argument("--sigma", type=float, default=0.15,
                    help="initial step size in normalised [0,1] space (default 0.15)")
    ap.add_argument("--robust", type=int, default=3,
                    help="perturbed neighbours per candidate for robustness (default 3)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--tracks", default=",".join(TRACKS),
                    help="comma-separated track list to tune the shared set on "
                         "(default: %(default)s; pass one name to tune a single layout)")
    args = ap.parse_args()
    global ROBUST_N, TRACKS
    ROBUST_N = args.robust
    TRACKS = [t.strip() for t in args.tracks.split(",") if t.strip()]

    os.makedirs(BUILD, exist_ok=True)
    ok, err = build_once()
    if not ok:
        print("BUILD FAILED:\n" + err); return

    rng = random.Random(args.seed)
    dim = len(NAMES)

    import math
    x0 = cfg_to_vec(DEFAULTS)
    popsize = args.popsize if args.popsize > 0 else int(4 + 3 * math.log(dim))
    maxfevals = args.generations * popsize
    es = cma.purecma.CMAES(x0, args.sigma,
                           popsize=str(popsize),
                           maxfevals=str(maxfevals))

    t0 = time.time()
    best_score = 1e9
    best_cfg   = dict(DEFAULTS)
    best_res   = None
    gen        = 0

    print("CMA-ES: dim=%d  popsize=%d  sigma0=%.3f  robust=%d  maxfevals=%d"
          % (dim, popsize, args.sigma, ROBUST_N, maxfevals), flush=True)

    # Evaluate baseline
    base_res, base_score = robust_score(DEFAULTS, rng)
    print("baseline  score=%.3f  %s" % (base_score, resstr(base_res)), flush=True)
    best_score, best_cfg, best_res = base_score, dict(DEFAULTS), base_res

    while not es.stop():
        gen += 1
        solutions = es.ask()  # list of normalised vectors

        # Convert to physical configs
        cfgs = [vec_to_cfg(x) for x in solutions]

        # Evaluate all candidates in this generation in parallel
        pop = len(cfgs)
        rob_rngs = [random.Random(args.seed + gen * 1000 + i) for i in range(pop)]

        fitnesses = [None] * pop
        gen_results = [None] * pop

        with ThreadPoolExecutor(max_workers=pop) as ex:
            futs = {ex.submit(robust_score, cfgs[i], rob_rngs[i]): i for i in range(pop)}
            for fut in as_completed(futs):
                i = futs[fut]
                res, s = fut.result()
                fitnesses[i] = s
                gen_results[i] = res

        es.tell(solutions, fitnesses)

        gen_best_i = min(range(pop), key=lambda i: fitnesses[i])
        gen_best_s = fitnesses[gen_best_i]
        flag = ""
        if gen_best_s < best_score - 1e-6:
            best_score = gen_best_s
            best_cfg   = cfgs[gen_best_i]
            best_res   = gen_results[gen_best_i]
            flag = "  <== NEW BEST"

        print("[gen %3d] best=%.3f  pop_range=%.3f–%.3f  sigma=%.4f  %s%s"
              % (gen, gen_best_s, min(fitnesses), max(fitnesses),
                 es.sigma, resstr(gen_results[gen_best_i]), flag), flush=True)

        if es.stop():
            print("CMA-ES stop condition:", es.stop())
            break

    dt = time.time() - t0
    evals = gen * popsize

    print("\n================ BEST (CMA-ES, both tracks) ================")
    print("worst-track worst-neighbour score = %.3f" % best_score)
    print(resstr(best_res))
    print(" ".join("%s=%.3f" % (n, best_cfg[n]) for n in NAMES))
    print("\n-D flags:")
    print(" ".join("-D%s=%.4ff" % (n, best_cfg[n]) for n in NAMES))

    with open(os.path.join(BUILD, "best_cmaes.json"), "w") as f:
        json.dump(best_cfg, f, indent=1)

    print("\n(%d gens, %d evals in %.1fs, %.3fs/eval)  -> HIL_Firmware/build/best_cmaes.json"
          % (gen, evals, dt, dt / max(1, evals)))


if __name__ == "__main__":
    main()
