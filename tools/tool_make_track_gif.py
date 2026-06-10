# Renders a looping GIF of the car driving the track for the README header.
# Runs hil_sim headless, captures frames, then writes an optimised looping GIF.
# Usage: python tools/tool_make_track_gif.py [output.gif]. TRACK env picks the layout.

import os
import sys
import math
import subprocess

import pygame
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
HIL_SIM_EXE = os.path.join(REPO_ROOT, "HIL_Firmware", "build", "hil_sim")
if sys.platform == "win32" and not HIL_SIM_EXE.endswith(".exe"):
    HIL_SIM_EXE += ".exe"

W, H = 900, 720
MARGIN = 24

BG = (10, 10, 10)  # colours match the visualiser
VIEW_BG = (26, 26, 26)
ASPHALT = (45, 45, 45)
BOUNDARY = (70, 70, 70)
CONE_BLUE = (30, 100, 255)
CONE_BLUE_EDGE = (120, 180, 255)
CONE_YELLOW = (255, 210, 0)
CONE_YELLOW_EDGE = (255, 240, 120)
RACING_LINE = (0, 210, 255)
CAR_FILL = (255, 80, 80)
CAR_OUTLINE = (255, 180, 180)
TRAIL_COL = (60, 200, 60)

CAPTURE_EVERY = 1  # state lines stream at 20 Hz, so capturing all and playing at 20 fps is real time
GIF_FPS = 20
TRAIL_LEN = 70

_scale = 1.0
_off_x = 0.0
_off_y = 0.0


# Compute the scale and offset that fit the whole track into the view.
def fit_bounds(pts):
    global _scale, _off_x, _off_y
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad = 4.0
    min_x, max_x = min(xs) - pad, max(xs) + pad
    min_y, max_y = min(ys) - pad, max(ys) + pad
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    vw, vh = W - 2 * MARGIN, H - 2 * MARGIN
    _scale = min(vw / span_x, vh / span_y)
    used_w = span_x * _scale
    used_h = span_y * _scale
    _off_x = MARGIN + (vw - used_w) / 2.0 - min_x * _scale
    _off_y = MARGIN + (vh - used_h) / 2.0 - min_y * _scale


# Convert world coordinates to screen pixels.
def w2s(wx, wy):
    sx = _off_x + wx * _scale
    sy = H - (_off_y + wy * _scale)   # flip y
    return (int(sx), int(sy))


# Read CONE/WP lines until a terminator line and return list of (x, y).
def read_block(proc, terminator):
    out = []
    while True:
        raw = proc.stdout.readline()
        if not raw:
            print("ERROR: hil_sim exited during startup")
            sys.exit(1)
        line = raw.decode("utf-8", errors="replace").rstrip()
        if line == terminator:
            return out
        parts = line.split()
        if len(parts) == 3 and parts[0] in ("WP", "CONE"):
            try:
                out.append((float(parts[1]), float(parts[2])))
            except ValueError:
                pass


# Draw the car as a rotated rectangle at the given pose.
def draw_car(surf, x, y, heading):
    sx, sy = w2s(x, y)
    half_l = max(3, int(1.4 * _scale))
    half_w = max(2, int(0.5 * _scale))
    c, s = math.cos(heading), math.sin(heading)
    corners = [
        (sx + half_l * c + half_w * s, sy - half_l * s + half_w * c),
        (sx + half_l * c - half_w * s, sy - half_l * s - half_w * c),
        (sx - half_l * c - half_w * s, sy + half_l * s - half_w * c),
        (sx - half_l * c + half_w * s, sy + half_l * s + half_w * c),
    ]
    ipts = [(int(px), int(py)) for px, py in corners]
    pygame.draw.polygon(surf, CAR_FILL, ipts)
    pygame.draw.polygon(surf, CAR_OUTLINE, ipts, 2)


# Run the sim, capture frames, and write the looping GIF.
def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO_ROOT, "docs", "track.gif")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

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

    fit_bounds(waypoints + left + right)

    wp_s = [w2s(*p) for p in waypoints]  # pre-compute static screen geometry
    left_s = [w2s(*p) for p in left]
    right_s = [w2s(*p) for p in right]
    cone_r = max(4, int(0.22 * _scale))

    pygame.init()
    surf = pygame.Surface((W, H))

    def draw_static(surf):
        surf.fill(BG)
        pygame.draw.rect(surf, VIEW_BG, (0, 0, W, H))
        if len(left_s) >= 2 and len(right_s) >= 2:
            poly = left_s + [left_s[0], right_s[0]] + list(reversed(right_s))
            pygame.draw.polygon(surf, ASPHALT, poly)
            pygame.draw.lines(surf, BOUNDARY, True, left_s, 1)
            pygame.draw.lines(surf, BOUNDARY, True, right_s, 1)
        if len(wp_s) >= 2:
            pygame.draw.lines(surf, RACING_LINE, True, wp_s, 2)
        for pt in left_s:
            pygame.draw.circle(surf, CONE_BLUE, pt, cone_r)
            pygame.draw.circle(surf, CONE_BLUE_EDGE, pt, cone_r, 1)
        for pt in right_s:
            pygame.draw.circle(surf, CONE_YELLOW, pt, cone_r)
            pygame.draw.circle(surf, CONE_YELLOW_EDGE, pt, cone_r, 1)

    background = pygame.Surface((W, H))  # cache the static background once
    draw_static(background)

    frames = []
    trail = []
    tick = 0
    first_lap_done = False
    start_lap = None

    while True:
        raw = proc.stdout.readline()
        if not raw:
            break
        line = raw.decode("utf-8", errors="replace").rstrip()
        parts = line.split()
        if len(parts) != 21 or parts[0] != "STATE":
            continue
        try:
            x = float(parts[1]); y = float(parts[2]); heading = float(parts[3])
            lap = int(parts[12])
        except ValueError:
            continue

        if start_lap is None:
            start_lap = lap

        trail.append((x, y))
        if len(trail) > TRAIL_LEN:
            trail.pop(0)

        tick += 1
        if tick % CAPTURE_EVERY == 0:
            surf.blit(background, (0, 0))
            if len(trail) >= 2:
                tpts = [w2s(tx, ty) for tx, ty in trail]
                pygame.draw.lines(surf, TRAIL_COL, False, tpts, 3)
            draw_car(surf, x, y, heading)
            raw_str = pygame.image.tostring(surf, "RGB")
            frames.append(Image.frombytes("RGB", (W, H), raw_str))

        if lap >= start_lap + 1:  # stop after one full lap for a clean loop, or safety cap
            first_lap_done = True
            break
        if tick > 20000:
            break

    try:
        proc.stdin.write(b"q")
        proc.stdin.flush()
    except OSError:
        pass
    proc.terminate()
    pygame.quit()

    if not frames:
        print("ERROR: no frames captured")
        sys.exit(1)

    target_w = 480  # downscale to keep the GIF small
    if frames[0].width > target_w:
        ratio = target_w / frames[0].width
        new_size = (target_w, int(frames[0].height * ratio))
        frames = [f.resize(new_size, Image.LANCZOS) for f in frames]

    # one shared palette keeps delta compression small, per-frame palettes bloat the file
    base_pal = frames[0].convert("P", palette=Image.ADAPTIVE, colors=64)
    pal_frames = [f.quantize(palette=base_pal, dither=Image.NONE) for f in frames]
    duration_ms = int(1000 / GIF_FPS)
    pal_frames[0].save(
        out_path,
        save_all=True,
        append_images=pal_frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
        disposal=2,  # clear each frame first or the moving trail tail smears into a streak
    )

    size_kb = os.path.getsize(out_path) / 1024
    status = "1 full lap" if first_lap_done else f"{tick} ticks"
    print(f"Wrote {out_path}  ({len(frames)} frames, {status}, {size_kb:.0f} KB)")


if __name__ == "__main__":
    main()
