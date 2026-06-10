#!/usr/bin/env python3
"""Robustness probe for the steering / speed / TV tuning. Perturbs the config
+/-pct one axis at a time and as combined jitter, reporting any that go
off-track, so you can see how close a config sits to the off-track edge.

After the controller simplification there are only four runtime tunables, applied
via TUNE_* env vars on a binary built ONCE (no recompile per candidate)."""
import os, random, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "HIL_Firmware", "build", "eval_roblqr.exe")
SRCS = ["tools/tool_eval_lap.c", "HIL_Firmware/src/motion_control.c",
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track_parser.c",
        "HIL_Firmware/src/path_planning.c",
        "ECU_Firmware/src/torque_vectoring.c", "shared/tunables.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]

# Current in-source defaults (shared/tunables.c). Perturbs the high-leverage
# gains; add any TUNE_* gain to probe its sensitivity.
BEST = {"GRIP_USE":0.90, "K_STANLEY":8.0, "K_DAMP":0.30, "RACING_MARGIN":0.40,
        "PP_RADIUS_FACTOR":1.6, "MAX_STEER_RAD":1.7, "MAX_BRAKE_DECEL_MS2":5.6,
        "SPEED_KP_NM":800.0, "KP_YAW":86.2440}
NAMES = list(BEST)

_built = [False]

def ev(cfg):
    if not _built[0]:
        if subprocess.run(["gcc","-std=c11","-O2",*INC,"-o",OUT,*SRCS,"-lm"],
                          cwd=ROOT, capture_output=True).returncode != 0:
            return None
        _built[0] = True
    env = dict(os.environ)
    for n in NAMES:
        env["TUNE_%s" % n] = "%.6f" % cfg[n]
    r = subprocess.run([OUT], cwd=ROOT, capture_output=True, text=True, env=env)
    for ln in r.stdout.splitlines():
        if ln.startswith("RESULT"):
            d={}
            for kv in ln[7:].split():
                k,v=kv.split("="); d[k]=float(v) if "." in v or "-" in v else int(v)
            return d
    return None

pct = float(sys.argv[1]) if len(sys.argv)>1 else 0.03
base = ev(BEST)
print("BASELINE: lap=%s off=%s meanCTE=%.3f worstCTE=%.3f"
      % (base["lap_s"],base["offtrack"],base["mean_cte"],base["worst_cte"]))
worst_off = 0
for n in NAMES:
    for sign in (+1,-1):
        c=dict(BEST); c[n]=BEST[n]*(1+sign*pct)
        r=ev(c)
        if r is None: print("  BUILD FAIL %s%+g%%"%(n,sign*pct*100)); continue
        worst_off=max(worst_off,r["offtrack"])
        if r["offtrack"]>0 or r["laps"]<1:
            print("  %-22s %+5.1f%%  lap=%-6s off=%-3s  OFF-TRACK!"%(n,sign*pct*100,r["lap_s"],r["offtrack"]))
print("-- 20 combined +/-%g%% random jitters --"%(pct*100))
random.seed(0); fails=0
for i in range(20):
    c={n:BEST[n]*(1+random.uniform(-pct,pct)) for n in NAMES}
    r=ev(c)
    if r is None or r["offtrack"]>0 or r["laps"]<1:
        fails+=1
        print("  jitter %2d  lap=%s off=%s"%(i,r and r["lap_s"],r and r["offtrack"]))
print("SUMMARY: worst single-axis off-track=%d ; combined-jitter failures=%d/20"%(worst_off,fails))
