#!/usr/bin/env python3
"""Robustness probe for the LQR steering / speed / TV tuning. Perturbs the config
+/-pct one axis at a time and as combined jitter, reporting any that go
off-track, so you can see how close a config sits to the off-track edge."""
import os, random, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "HIL_Firmware", "build", "eval_roblqr.exe")
SRCS = ["tools/tool_eval_lap.c", "HIL_Firmware/src/motion_control.c",
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track_parser.c",
        "HIL_Firmware/src/path_planning.c", "HIL_Firmware/src/lqr_steer.c",
        "ECU_Firmware/src/torque_vectoring.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]

# Current in-source defaults (the clean ~25.8 s downforce-aware config).
BEST = {
 "MAX_LATERAL_ACCEL_MS2":14.0433,"PP_GRIP_ACCEL":11.1584,"RACING_MARGIN":0.2686,
 "PP_MIN_RADIUS_M":6.5000,"GG_ACCEL_MS2":7.8831,"PLANNER_DOWNFORCE_FRAC":0.1085,
 "LAT_GRIP_REF_MS2":15.6526,
 "KP_YAW_DEFAULT":86.7214,"TV_KFF":12.4083,"TV_REAR_SHARE":0.6017,
 "LQR_Q_E1":10.1899,"LQR_Q_E1D":0.3000,"LQR_Q_E2":9.7728,"LQR_Q_E2D":0.1229,
 "LQR_R":3.8399,"LQR_KI":5.8300,"LQR_I_MAX":0.3350}
NAMES = list(BEST)

def ev(cfg):
    defs = ["-D%s=%.5ff" % (n, cfg[n]) for n in NAMES]
    if subprocess.run(["gcc","-std=c11","-O2",*INC,*defs,"-o",OUT,*SRCS,"-lm"],
                      cwd=ROOT, capture_output=True).returncode != 0:
        return None
    r = subprocess.run([OUT], cwd=ROOT, capture_output=True, text=True)
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
