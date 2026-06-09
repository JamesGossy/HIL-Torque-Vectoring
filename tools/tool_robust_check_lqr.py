#!/usr/bin/env python3
"""Robustness probe for the LQR steering / speed / TV tuning. Perturbs the config
+/-pct one axis at a time and as combined jitter, reporting any that go
off-track, so you can see how close a config sits to the off-track edge."""
import os, random, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "HIL_Firmware", "build", "eval_roblqr.exe")
SRCS = ["tools/tool_eval_lap.c", "HIL_Firmware/src/motion_control.c",
        "HIL_Firmware/src/vehicle_model.c", "HIL_Firmware/src/track.c",
        "HIL_Firmware/src/path_planning.c", "HIL_Firmware/src/lqr_steer.c",
        "ECU_Firmware/src/torque_vectoring.c"]
INC = ["-I", "HIL_Firmware/include", "-I", "shared", "-I", "ECU_Firmware/include"]

# LQR + recommended companion settings (the clean 27.03 s config).
BEST = {
 "MAX_LATERAL_ACCEL_MS2":13.7214,"PP_GRIP_ACCEL":13.6871,"RACING_MARGIN":0.2756,
 "PP_MIN_RADIUS_M":6.2565,"GG_ACCEL_MS2":7.7134,"LAT_GRIP_REF_MS2":14.1014,
 "KP_YAW_DEFAULT":48.5755,"TV_KFF":13.8448,"TV_REAR_SHARE":0.4602,
 "LQR_Q_E1":11.2975,"LQR_Q_E1D":1.4164,"LQR_Q_E2":14.2220,"LQR_Q_E2D":0.8434,
 "LQR_R":7.7125,"LQR_KI":5.4654,"LQR_I_MAX":0.5629}
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
