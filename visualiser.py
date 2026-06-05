"""
visualiser.py

Launches the HIL simulation (hil_sim) and draws its output in a pygame window.

How it works:
  - hil_sim is started as a subprocess. Its stdout is piped to this script.
  - hil_sim first prints the track waypoints (a header block), then streams
    one STATE line per display tick (20 Hz).
  - This script reads those lines and redraws the window each frame.
  - Keypresses in the pygame window are forwarded to hil_sim's stdin so the
    t / [ / ] / q controls still work.

Install pygame with:
    pip install pygame

Run with:
    python visualiser.py

Or on Windows, if you built hil_sim in the MSYS2 shell:
    python visualiser.py
(make sure hil_sim.exe is in HIL_Firmware/ relative to this script)
"""

import subprocess
import sys
import os
import math
import threading
import queue
import pygame


# ---- Path to the simulation binary ----
# Adjust this if you put the binary somewhere else.
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
HIL_SIM_EXE = os.path.join(SCRIPT_DIR, "HIL_Firmware", "build", "hil_sim")

# On Windows the binary is called hil_sim.exe
if sys.platform == "win32" and not HIL_SIM_EXE.endswith(".exe"):
    HIL_SIM_EXE += ".exe"


# ---- Window and colours ----
WINDOW_W = 1100
WINDOW_H = 700

BLACK      = (10,  10,  10)
DARK_GREY  = (30,  30,  30)
MID_GREY   = (60,  60,  60)
LIGHT_GREY = (180, 180, 180)
WHITE      = (240, 240, 240)

TRACK_FILL   = (50,  50,  50)    # tarmac surface
TRACK_EDGE   = (220, 220, 100)   # yellow kerb line
CAR_COLOUR   = (255,  80,  80)   # red car dot
CAR_OUTLINE  = (255, 180, 180)
HEADING_COL  = (255, 200,  50)   # heading arrow

TV_ON_COL    = (80,  220,  80)   # green when TV is on
TV_OFF_COL   = (220,  80,  80)   # red when TV is off

FL_COL = (100, 180, 255)         # front-left torque bar
FR_COL = (100, 220, 150)         # front-right
RL_COL = (255, 180, 100)         # rear-left
RR_COL   = (220, 100, 200)       # rear-right
REGEN_COL = (80, 160, 255)       # blue for regenerative braking bars


# ---- Track geometry ----
# The visualiser no longer knows the track shape.  It reads the centreline
# waypoints from hil_sim's startup header (the firmware is the single source of
# truth) and draws whatever it gets — so any track, however complex, just works.
TRACK_WIDTH = 8.0     # metres, visual band width (must match the feel of the sim)

# Centreline waypoints in world metres, filled in once from the header block.
TRACK_WAYPOINTS = []


# ---- Coordinate mapping ----
# Maps world metres to screen pixels.  The world bounds are computed from the
# parsed waypoints (set_track_bounds) so the track is always framed nicely,
# whatever its size or shape.

# The track view occupies the left portion of the window.
VIEW_X = 20
VIEW_Y = 20
VIEW_W = 720
VIEW_H = WINDOW_H - 40

# World->screen mapping, recomputed by set_track_bounds().  A single uniform
# scale (plus centring offsets) keeps the track's aspect ratio undistorted.
_SCALE   = 1.0
_OFF_X   = 0.0
_OFF_Y   = 0.0


def set_track_bounds(waypoints):
    """
    Compute a uniform world->screen scale that fits all waypoints inside the
    view with a margin, preserving aspect ratio and centring the track.
    """
    global _SCALE, _OFF_X, _OFF_Y
    if not waypoints:
        return

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]
    margin = TRACK_WIDTH          # world metres of breathing room around the track
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)

    # One scale for both axes = no distortion; pick whichever axis is the limit.
    _SCALE = min(VIEW_W / span_x, VIEW_H / span_y)

    # Centre the track within the view.
    used_w = span_x * _SCALE
    used_h = span_y * _SCALE
    _OFF_X = VIEW_X + (VIEW_W - used_w) / 2.0 - min_x * _SCALE
    _OFF_Y = VIEW_Y + (VIEW_H - used_h) / 2.0 - min_y * _SCALE


def world_to_screen(wx, wy):
    """Convert a world (x, y) in metres to screen (col, row) in pixels."""
    sx = _OFF_X + wx * _SCALE
    # Flip Y: world up = screen up.  Reflect around the view's vertical centre.
    sy = (2 * VIEW_Y + VIEW_H) - (_OFF_Y + wy * _SCALE)
    return (int(sx), int(sy))


def world_len_to_pixels(metres):
    """Convert a world length in metres to a pixel length."""
    return max(1, int(metres * _SCALE))


# ---- State parsed from hil_sim stdout ----
class SimState:
    def __init__(self):
        self.x          = 0.0
        self.y          = 0.0
        self.heading    = 0.0
        self.speed_kmh  = 0.0
        self.yaw_degs   = 0.0
        self.fl         = 0.0
        self.fr         = 0.0
        self.rl         = 0.0
        self.rr         = 0.0
        self.tv_enabled = True
        self.kp         = 50.0
        self.lap        = 0
        self.elapsed_s  = 0.0
        self.steering   = 0.0

    def parse(self, line):
        """Parse a STATE line. Returns True on success."""
        parts = line.split()
        if len(parts) != 15 or parts[0] != "STATE":
            return False
        try:
            self.x          = float(parts[1])
            self.y          = float(parts[2])
            self.heading    = float(parts[3])
            self.speed_kmh  = float(parts[4])
            self.yaw_degs   = float(parts[5])
            self.fl         = float(parts[6])
            self.fr         = float(parts[7])
            self.rl         = float(parts[8])
            self.rr         = float(parts[9])
            self.tv_enabled = int(parts[10]) != 0
            self.kp         = float(parts[11])
            self.lap        = int(parts[12])
            self.elapsed_s  = float(parts[13])
            self.steering   = float(parts[14])
        except ValueError:
            return False
        return True


# ---- Background reader thread ----
# Reads lines from hil_sim stdout and puts them in a queue.
# Keeps the pygame event loop from blocking on I/O.

def reader_thread(proc, line_queue):
    for raw_line in proc.stdout:
        line_queue.put(raw_line.decode("utf-8", errors="replace").rstrip())
    line_queue.put(None)   # signals that the process has ended


# ---- Drawing helpers ----

def _stroke_path(surface, points, colour, width):
    """
    Draw a closed polyline as a thick stroke, with rounded joins/caps so the
    band has no gaps or notches at corners.  This is the core trick: rendering
    the track as a *stroke* of the centreline (rather than offset edge curves)
    means self-intersections and tight turns just overlap cleanly — no special
    cases, works for any track shape.
    """
    if len(points) < 2:
        return
    radius = max(1, width // 2)
    # The thick connecting lines form the band...
    pygame.draw.lines(surface, colour, True, points, width)
    # ...and a filled circle at every vertex rounds off the joins so corners
    # don't show notches where consecutive segments meet at an angle.
    for p in points:
        pygame.draw.circle(surface, colour, p, radius)


def draw_track(surface, font_small):
    """
    Draw the track centreline as two stacked strokes:

      1. a wide YELLOW stroke  (the kerb edge)
      2. a slightly narrower GREY stroke on top  (the tarmac)

    The yellow that peeks out around the grey becomes a perfect outline that
    follows the path exactly — including crossings and hairpins — with zero
    offset-curve maths and zero magic numbers.  Add more complex tracks freely.
    """
    pts = [world_to_screen(wx, wy) for (wx, wy) in TRACK_WAYPOINTS]
    if len(pts) < 2:
        return

    tarmac_px = world_len_to_pixels(TRACK_WIDTH)
    kerb_px   = tarmac_px + 4            # 2 px of yellow showing on each side

    _stroke_path(surface, pts, TRACK_EDGE, kerb_px)    # kerb underneath
    _stroke_path(surface, pts, TRACK_FILL, tarmac_px)  # tarmac on top


def draw_car(surface, state):
    """Draw the car as a dot with a heading arrow."""
    sx, sy = world_to_screen(state.x, state.y)
    r = 8

    pygame.draw.circle(surface, CAR_COLOUR, (sx, sy), r)
    pygame.draw.circle(surface, CAR_OUTLINE, (sx, sy), r, 2)

    arrow_len = 26
    ex = sx + int(arrow_len * math.cos(state.heading))
    ey = sy - int(arrow_len * math.sin(state.heading))
    pygame.draw.line(surface, HEADING_COL, (sx, sy), (ex, ey), 3)


def draw_torque_bars(surface, font, state, panel_x, panel_y):
    """
    Bidirectional torque bars: 0 at the centre line, drive torque fills upward,
    regen (negative) fills downward in blue.  Scale: ±100 Nm per wheel.
    """
    MAX_T     = 100.0
    BAR_W     = 55
    HALF_H    = 35    # pixels per direction (35 px = 100 Nm)
    BAR_MAX_H = HALF_H * 2
    GAP       = 12
    LABEL_H   = 26

    positions = [
        ("FL", FL_COL, state.fl,  panel_x,               panel_y),
        ("FR", FR_COL, state.fr,  panel_x + BAR_W + GAP, panel_y),
        ("RL", RL_COL, state.rl,  panel_x,               panel_y + BAR_MAX_H + LABEL_H + GAP),
        ("RR", RR_COL, state.rr,  panel_x + BAR_W + GAP, panel_y + BAR_MAX_H + LABEL_H + GAP),
    ]

    for lbl_text, colour, torque, bx, by in positions:
        zero_y = by + HALF_H   # screen y for the 0-Nm centre line

        # Background
        pygame.draw.rect(surface, MID_GREY, (bx, by, BAR_W, BAR_MAX_H))

        # Drive (positive): fills upward from centre
        if torque > 0:
            fill_h = int(HALF_H * min(torque, MAX_T) / MAX_T)
            pygame.draw.rect(surface, colour, (bx, zero_y - fill_h, BAR_W, fill_h))
        # Regen (negative): fills downward from centre
        elif torque < 0:
            fill_h = int(HALF_H * min(-torque, MAX_T) / MAX_T)
            pygame.draw.rect(surface, REGEN_COL, (bx, zero_y, BAR_W, fill_h))

        # Border + centre line
        pygame.draw.rect(surface, LIGHT_GREY, (bx, by, BAR_W, BAR_MAX_H), 1)
        pygame.draw.line(surface, LIGHT_GREY, (bx, zero_y), (bx + BAR_W, zero_y), 1)

        lbl = font.render(lbl_text, True, LIGHT_GREY)
        surface.blit(lbl, (bx + BAR_W // 2 - lbl.get_width() // 2, by + BAR_MAX_H + 2))

        val = font.render(f"{torque:+.0f}", True, WHITE)
        surface.blit(val, (bx + BAR_W // 2 - val.get_width() // 2, by + BAR_MAX_H + 13))


def draw_panel(surface, font_large, font, font_small, state):
    """Draw the data panel on the right side of the window."""
    px = VIEW_X + VIEW_W + 30
    py = 20
    sep_w = 310

    def sep(y):
        pygame.draw.line(surface, MID_GREY, (px, y), (px + sep_w, y), 1)
        return y + 8

    def label(text, y, colour=LIGHT_GREY):
        s = font_small.render(text, True, colour)
        surface.blit(s, (px, y))
        return y + s.get_height() + 2

    def value(text, y, colour=WHITE):
        s = font_large.render(text, True, colour)
        surface.blit(s, (px, y))
        return y + s.get_height() + 6

    # Title
    title = font_large.render("HIL  Torque  Vectoring", True, WHITE)
    surface.blit(title, (px, py))
    py += title.get_height() + 6
    py = sep(py)

    # TV status + Kp on same line to save vertical space
    tv_text = "TV  ON" if state.tv_enabled else "TV  OFF"
    tv_col  = TV_ON_COL if state.tv_enabled else TV_OFF_COL
    tv_s    = font_large.render(tv_text, True, tv_col)
    surface.blit(tv_s, (px, py))
    kp_s = font.render(f"Kp = {state.kp:.1f}", True, LIGHT_GREY)
    surface.blit(kp_s, (px + tv_s.get_width() + 14, py + 4))
    py += tv_s.get_height() + 6
    py = sep(py)

    # Lap / time side by side
    mins = int(state.elapsed_s) // 60
    secs = int(state.elapsed_s) % 60
    col2 = px + 100
    lap_lbl = font_small.render("LAP",  True, LIGHT_GREY)
    tim_lbl = font_small.render("TIME", True, LIGHT_GREY)
    surface.blit(lap_lbl, (px,   py))
    surface.blit(tim_lbl, (col2, py))
    py += lap_lbl.get_height() + 2
    lap_val = font_large.render(f"{state.lap}", True, WHITE)
    tim_val = font_large.render(f"{mins}:{secs:02d}", True, WHITE)
    surface.blit(lap_val, (px,   py))
    surface.blit(tim_val, (col2, py))
    py += lap_val.get_height() + 6
    py = sep(py)

    # Car-relative measurements (what the driver / ECU actually sense)
    speed_ms = state.speed_kmh / 3.6
    lat_g    = abs(math.radians(state.yaw_degs) * speed_ms) / 9.81
    steer_deg = math.degrees(state.steering)

    col2 = px + 155
    # Speed + lateral g side by side
    surface.blit(font_small.render("SPEED",    True, LIGHT_GREY), (px,   py))
    surface.blit(font_small.render("LAT G",    True, LIGHT_GREY), (col2, py))
    py += font_small.size("X")[1] + 2
    surface.blit(font_large.render(f"{state.speed_kmh:.1f} km/h", True, WHITE), (px,   py))
    surface.blit(font_large.render(f"{lat_g:.2f} g",              True, WHITE), (col2, py))
    py += font_large.size("X")[1] + 6

    # Yaw rate + steering angle side by side
    surface.blit(font_small.render("YAW RATE", True, LIGHT_GREY), (px,   py))
    surface.blit(font_small.render("STEER",    True, LIGHT_GREY), (col2, py))
    py += font_small.size("X")[1] + 2
    surface.blit(font_large.render(f"{state.yaw_degs:+.1f} d/s", True, WHITE), (px,   py))
    surface.blit(font_large.render(f"{steer_deg:+.1f} deg",       True, WHITE), (col2, py))
    py += font_large.size("X")[1] + 6
    py = sep(py)

    # Torque bars
    py = label("WHEEL TORQUES  (Nm, ±100)", py)
    py += 4
    draw_torque_bars(surface, font_small, state, px, py)
    py += (70 + 26 + 12) * 2 - 12 + 6
    py = sep(py)

    # Controls hint
    s = font_small.render("T  toggle TV    [/]  Kp±5    Q  quit", True, MID_GREY)
    surface.blit(s, (px, py))


# ---- Main ----

def main():
    # Check the binary exists before starting anything
    if not os.path.isfile(HIL_SIM_EXE):
        print(f"ERROR: Could not find {HIL_SIM_EXE}")
        print("Build it first with:  make  (from the repo root, in MSYS2 MinGW x64 shell)")
        sys.exit(1)

    # Start the simulation
    proc = subprocess.Popen(
        [HIL_SIM_EXE],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    # Read the track header block and parse the waypoints.  The firmware is the
    # single source of truth for the track shape; we just draw what it sends.
    #   TRACK <count>
    #   WP <x> <y>      (one per waypoint)
    #   END_TRACK
    global TRACK_WAYPOINTS
    waypoints = []
    while True:
        raw = proc.stdout.readline()
        if not raw:
            print("ERROR: hil_sim exited before sending END_TRACK")
            sys.exit(1)
        line = raw.decode("utf-8", errors="replace").rstrip()
        if line == "END_TRACK":
            break
        parts = line.split()
        if len(parts) == 3 and parts[0] == "WP":
            try:
                waypoints.append((float(parts[1]), float(parts[2])))
            except ValueError:
                pass

    TRACK_WAYPOINTS = waypoints
    set_track_bounds(waypoints)

    # Start the background reader thread for STATE lines
    line_q = queue.Queue()
    t = threading.Thread(target=reader_thread, args=(proc, line_q), daemon=True)
    t.start()

    # Pygame setup
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("HIL Torque Vectoring")
    clock  = pygame.time.Clock()

    font_large  = pygame.font.SysFont("consolas", 22, bold=True)
    font_medium = pygame.font.SysFont("consolas", 17)
    font_small  = pygame.font.SysFont("consolas", 13)

    state    = SimState()
    running  = True

    while running:
        clock.tick(60)   # cap at 60 fps; state arrives at 20 Hz from hil_sim

        # --- Drain all available state lines (take the latest) ---
        latest = None
        try:
            while True:
                item = line_q.get_nowait()
                if item is None:
                    running = False
                    break
                latest = item
        except queue.Empty:
            pass

        if latest is not None:
            state.parse(latest)

        # --- Pygame events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                key_map = {
                    pygame.K_t:         b"t",
                    pygame.K_LEFTBRACKET:  b"[",
                    pygame.K_RIGHTBRACKET: b"]",
                    pygame.K_q:         b"q",
                    pygame.K_ESCAPE:    b"q",
                }
                cmd = key_map.get(event.key)
                if cmd and proc.stdin:
                    try:
                        proc.stdin.write(cmd)
                        proc.stdin.flush()
                    except OSError:
                        pass
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

        # --- Draw ---
        screen.fill(BLACK)

        # Track view background
        pygame.draw.rect(screen, DARK_GREY, (VIEW_X, VIEW_Y, VIEW_W, VIEW_H))

        draw_track(screen, font_small)
        draw_car(screen, state)
        draw_panel(screen, font_large, font_medium, font_small, state)

        # View border
        pygame.draw.rect(screen, MID_GREY, (VIEW_X, VIEW_Y, VIEW_W, VIEW_H), 1)

        pygame.display.flip()

    # Clean up
    try:
        proc.stdin.write(b"q")
        proc.stdin.flush()
    except OSError:
        pass
    proc.terminate()
    pygame.quit()


if __name__ == "__main__":
    main()
