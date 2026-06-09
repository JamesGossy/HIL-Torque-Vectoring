"""
make_tv_shot.py

Captures a still frame for the README straight from the real visualiser: the car
at the hairpin in follow-cam, next to the same wheel-torque bars the live tool
draws. No extra labels or arrows are added; it is exactly what the visualiser
renders, just cropped to the car and the torque panel.

It reuses the visualiser's own drawing code (imported as a module) and replays the
sim to the hairpin frame, so the torques shown are the real values at that point.

Produces docs/tv_hairpin.png

Usage:
    python tools/make_tv_shot.py
"""

import os
import sys
import subprocess

import pygame

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, REPO_ROOT)

import visualiser as V   # noqa: E402  (reuse the real drawing code)

HIL_SIM_EXE = os.path.join(REPO_ROOT, "HIL_Firmware", "build", "hil_sim")
if sys.platform == "win32" and not HIL_SIM_EXE.endswith(".exe"):
    HIL_SIM_EXE += ".exe"

# The hairpin: tightest corner of the FSG 2024 lap (max steering). Replay to here.
HAIRPIN_FRAME = 342

# Tighter follow-cam zoom than the live default, so the car fills the crop.
FOLLOW_SCALE = 26.0


def read_block(proc, terminator):
    out = []
    while True:
        raw = proc.stdout.readline()
        if not raw:
            print("ERROR: hil_sim exited during startup")
            sys.exit(1)
        line = raw.decode(errors="replace").rstrip()
        if line == terminator:
            return out
        parts = line.split()
        if len(parts) == 3 and parts[0] in ("WP", "CONE"):
            try:
                out.append((float(parts[1]), float(parts[2])))
            except ValueError:
                pass


def main():
    if not os.path.isfile(HIL_SIM_EXE):
        print(f"ERROR: {HIL_SIM_EXE} not found. Build with `make` first.")
        sys.exit(1)

    proc = subprocess.Popen(
        [HIL_SIM_EXE],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    waypoints = read_block(proc, "END_TRACK")
    left = read_block(proc, "END_LEFT_CONES")
    right = read_block(proc, "END_RIGHT_CONES")

    V.TRACK_WAYPOINTS = waypoints
    V.set_track_bounds(waypoints)
    V.update_cone_screen_pts(left, right)
    V.FOLLOW_SCALE = FOLLOW_SCALE

    pygame.init()
    canvas = pygame.Surface((V.WINDOW_W, V.WINDOW_H))
    font_large = pygame.font.SysFont("consolas", 20, bold=True)
    font_small = pygame.font.SysFont("consolas", 14)

    state = V.SimState()

    # Replay STATE lines up to the hairpin frame
    frame = 0
    while frame <= HAIRPIN_FRAME:
        raw = proc.stdout.readline()
        if not raw:
            break
        line = raw.decode(errors="replace").rstrip()
        if state.parse(line):
            frame += 1

    try:
        proc.stdin.write(b"q")
        proc.stdin.flush()
    except OSError:
        pass
    proc.terminate()

    # --- Draw the track view in follow-cam (exactly like the visualiser) ---
    canvas.fill(V.BLACK)
    pygame.draw.rect(canvas, V.DARK_GREY, (V.VIEW_X, V.VIEW_Y, V.VIEW_W, V.VIEW_H))
    V.set_follow_transform(state.x, state.y)
    canvas.set_clip(pygame.Rect(V.VIEW_X, V.VIEW_Y, V.VIEW_W, V.VIEW_H))
    V.draw_track(canvas)
    V.draw_racing_line(canvas)
    V.draw_car(canvas, state)
    canvas.set_clip(None)
    V.restore_map_transform()

    # --- Compose the final image: a square car crop on the left, the visualiser's
    # torque bars on the right, vertically centred, with no added labels/arrows. ---
    CAR = 360                       # square car crop, px
    BARS_BLOCK_W = 2 * 50 + 10      # two 50px bar columns + 10px gap
    BARS_BLOCK_H = 2 * (64 + 26) + 10
    GAP = 36
    MARGIN = 18
    HDR_H = 24
    HDR_W = 150                     # room for the "WHEEL TORQUES (Nm)" header

    out_w = CAR + GAP + max(BARS_BLOCK_W, HDR_W) + MARGIN
    out_h = CAR

    # The follow-cam centres the car in the track viewport; crop a square there.
    car_cx = V.VIEW_X + V.VIEW_W // 2
    car_cy = V.VIEW_Y + V.VIEW_H // 2
    car_crop = pygame.Rect(car_cx - CAR // 2, car_cy - CAR // 2, CAR, CAR)

    final = pygame.Surface((out_w, out_h))
    final.fill(V.BLACK)
    final.blit(canvas, (0, 0), area=car_crop)

    bars_x = CAR + GAP
    bars_y = (out_h - BARS_BLOCK_H) // 2 + HDR_H // 2
    hdr = font_small.render("WHEEL TORQUES (Nm)", True, V.LIGHT_GREY)
    final.blit(hdr, (bars_x, bars_y - HDR_H - 4))
    V.draw_torque_bars(final, font_small, state, bars_x, bars_y)

    os.makedirs(os.path.join(REPO_ROOT, "docs"), exist_ok=True)
    out = os.path.join(REPO_ROOT, "docs", "tv_hairpin.png")
    pygame.image.save(final, out)
    pygame.quit()
    print(f"Wrote {out}  ({os.path.getsize(out)/1024:.0f} KB)  "
          f"FL {state.fl:+.1f} FR {state.fr:+.1f} RL {state.rl:+.1f} RR {state.rr:+.1f}")


if __name__ == "__main__":
    main()
