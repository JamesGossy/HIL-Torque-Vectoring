"""
make_plots.py

Runs the HIL simulation and turns the real STATE telemetry into static plots
for the README. No hand-faked data: every line is captured from hil_sim.

Produces (in docs/):
  speed_trace.png    actual speed vs the planner's target speed, one lap
  torque_corner.png  the four wheel torques through one corner (shows vectoring)

Usage:
    python tools/make_plots.py

Env:
    TRACK   track layout (default fsg2024), forwarded to hil_sim
"""

import os
import sys
import subprocess

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DOCS = os.path.join(REPO_ROOT, "docs")
HIL_SIM_EXE = os.path.join(REPO_ROOT, "HIL_Firmware", "build", "hil_sim")
if sys.platform == "win32" and not HIL_SIM_EXE.endswith(".exe"):
    HIL_SIM_EXE += ".exe"

# Dark theme to match the visualiser / GIF
BG = "#1a1a1a"
FG = "#dcdcdc"
GRID = "#3a3a3a"
GREEN = "#64dc96"    # actual
YELLOW = "#ffc832"   # desired / target
CYAN = "#00d2ff"
FL = "#64b4ff"
FR = "#64dc96"
RL = "#ffb464"
RR = "#dc64c8"


def run_one_lap():
    """Run hil_sim, skip the track header, return per-tick STATE arrays for one lap."""
    proc = subprocess.Popen(
        [HIL_SIM_EXE],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    # Skip the three header blocks
    for terminator in ("END_TRACK", "END_LEFT_CONES", "END_RIGHT_CONES"):
        while True:
            raw = proc.stdout.readline()
            if not raw:
                print("ERROR: hil_sim exited during startup")
                sys.exit(1)
            if raw.decode(errors="replace").rstrip() == terminator:
                break

    rows = []
    start_lap = None
    while True:
        raw = proc.stdout.readline()
        if not raw:
            break
        parts = raw.decode(errors="replace").split()
        if len(parts) != 21 or parts[0] != "STATE":
            continue
        vals = [float(p) for p in parts[1:]]
        lap = int(vals[11])
        if start_lap is None:
            start_lap = lap
        rows.append(vals)
        if lap >= start_lap + 1:
            break
        if len(rows) > 5000:
            break

    try:
        proc.stdin.write(b"q")
        proc.stdin.flush()
    except OSError:
        pass
    proc.terminate()

    a = np.array(rows)
    # Column order after STATE (0-indexed into vals):
    # 0 x 1 y 2 heading 3 speed_kmh 4 yaw_deg_s 5 fl 6 fr 7 rl 8 rr 9 tv
    # 10 kp 11 lap 12 elapsed_s 13 steering 14 slip 15 desired_yaw 16 ax
    # 17 ay 18 vy 19 target_kmh
    return {
        "t": a[:, 12] - a[0, 12],
        "speed": a[:, 3],
        "target": a[:, 19],
        "yaw": np.radians(a[:, 4]),
        "desired_yaw": a[:, 15],
        "fl": a[:, 5], "fr": a[:, 6], "rl": a[:, 7], "rr": a[:, 8],
    }


def smooth(y, n=5):
    """Light centred moving average so the per-tick controller noise doesn't
    bury the underlying left/right split. Window is short (n ticks ~ 0.5 s)."""
    if n < 2:
        return y
    k = np.ones(n) / n
    return np.convolve(y, k, mode="same")


def style(ax):
    ax.set_facecolor(BG)
    ax.grid(True, color=GRID, linewidth=0.6)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=FG)
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.title.set_color(FG)


def save(fig, name):
    path = os.path.join(DOCS, name)
    fig.savefig(path, dpi=110, facecolor=BG, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {path}  ({os.path.getsize(path)/1024:.0f} KB)")


def main():
    os.makedirs(DOCS, exist_ok=True)
    if not os.path.isfile(HIL_SIM_EXE):
        print(f"ERROR: {HIL_SIM_EXE} not found. Build with `make` first.")
        sys.exit(1)

    d = run_one_lap()

    # --- Speed vs target ---
    fig, ax = plt.subplots(figsize=(8, 3.2))
    style(ax)
    ax.plot(d["t"], d["target"], color=YELLOW, lw=2, label="planned target speed")
    ax.plot(d["t"], d["speed"], color=GREEN, lw=2, label="actual speed")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (km/h)")
    ax.set_title("Speed: the planner brakes for every corner, the car follows")
    ax.legend(facecolor=BG, edgecolor=GRID, labelcolor=FG, loc="lower right")
    ax.set_xlim(d["t"][0], d["t"][-1])
    save(fig, "speed_trace.png")

    # --- Four wheel torques through ONE corner ---
    # The full-lap torque trace is too dense to read, so zoom into the single
    # hardest corner (highest sustained yaw rate) where the left/right split is
    # clearest. A short moving average removes per-tick noise.
    dt = np.median(np.diff(d["t"]))
    w = max(1, int(round(3.0 / dt)))            # 3-second window
    yaw_abs = np.abs(d["yaw"])
    score = np.array([yaw_abs[i:i + w].mean() for i in range(len(d["t"]) - w)])
    i0 = int(score.argmax())
    sl = slice(i0, i0 + w)
    t = d["t"][sl]

    fig, ax = plt.subplots(figsize=(8, 3.4))
    style(ax)
    ax.plot(t, smooth(d["rl"])[sl], color=RL, lw=2, label="rear left")
    ax.plot(t, smooth(d["rr"])[sl], color=RR, lw=2, label="rear right")
    ax.plot(t, smooth(d["fl"])[sl], color=FL, lw=1.4, label="front left")
    ax.plot(t, smooth(d["fr"])[sl], color=FR, lw=1.4, label="front right")
    ax.axhline(0, color=FG, lw=0.6)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("wheel torque (Nm)")
    ax.set_title("The four wheels get different torque: the lines fan apart in a corner")
    ax.legend(facecolor=BG, edgecolor=GRID, labelcolor=FG, loc="best")
    ax.set_xlim(t[0], t[-1])
    save(fig, "torque_corner.png")


if __name__ == "__main__":
    main()
