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

STATE protocol (20 fields):
  STATE x y heading speed_kmh yaw_deg_s fl fr rl rr tv kp lap elapsed_s
        steering_rad slip_angle_rad desired_yaw_rad_s ax_ms2 ay_ms2 vy_ms

Install pygame with:
    pip install pygame

Run with:
    python visualiser.py
"""

import subprocess
import sys
import os
import math
import threading
import queue
import collections
import pygame


# ---- Path to the simulation binary ----
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
HIL_SIM_EXE = os.path.join(SCRIPT_DIR, "HIL_Firmware", "build", "hil_sim")

if sys.platform == "win32" and not HIL_SIM_EXE.endswith(".exe"):
    HIL_SIM_EXE += ".exe"


# ---- Window and colours ----
WINDOW_W = 1400
WINDOW_H = 820

BLACK      = (10,  10,  10)
DARK_GREY  = (30,  30,  30)
MID_GREY   = (60,  60,  60)
LIGHT_GREY = (180, 180, 180)
WHITE      = (240, 240, 240)

TRACK_FILL   = (50,  50,  50)
TRACK_EDGE   = (220, 220, 100)
CAR_COLOUR   = (255,  80,  80)
CAR_OUTLINE  = (255, 180, 180)
HEADING_COL  = (255, 200,  50)

TV_ON_COL    = (80,  220,  80)
TV_OFF_COL   = (220,  80,  80)

TRAJ_ON_COL  = ( 60, 200,  60)   # dark green  — trajectory with TV on
TRAJ_OFF_COL = (200, 160,  40)   # amber       — trajectory with TV off

FL_COL    = (100, 180, 255)
FR_COL    = (100, 220, 150)
RL_COL    = (255, 180, 100)
RR_COL    = (220, 100, 200)
REGEN_COL = (80,  160, 255)

YAW_ACTUAL_COL  = (100, 220, 150)   # green  – actual yaw rate
YAW_DESIRED_COL = (255, 200,  50)   # yellow – desired yaw rate

SLIP_GREEN  = ( 80, 220,  80)
SLIP_YELLOW = (220, 200,  50)
SLIP_RED    = (220,  80,  80)

US_COL   = (255, 160,  40)   # orange — understeer
OS_COL   = (220,  60,  60)   # red    — oversteer

LAP_TV_ON_COL  = ( 80, 220,  80)
LAP_TV_OFF_COL = (220, 200,  50)

GG_DOT_COL    = (255, 240,  50)   # current G-G point
GG_TRAIL_COL  = ( 80, 120, 180)   # history trail


# ---- Track geometry ----
TRACK_WIDTH = 8.0     # metres (used for bounding-box margin only)

TRACK_WAYPOINTS  = []
LEFT_CONE_PTS    = []   # screen-space blue cone positions
RIGHT_CONE_PTS   = []   # screen-space yellow cone positions

CONE_BLUE        = ( 30, 100, 255)
CONE_BLUE_EDGE   = (120, 180, 255)
CONE_YELLOW      = (255, 210,   0)
CONE_YELLOW_EDGE = (255, 240, 120)
CONE_ORANGE      = (255, 120,   0)

RACING_LINE_COL  = (  0, 210, 255)   # cyan — optimal racing line


# ---- View modes ----
# MAP mode   : entire track fits the viewport (default)
# FOLLOW mode: fixed zoom centred on the car, toggled with M key
VIEW_MODE_MAP    = 0
VIEW_MODE_FOLLOW = 1
_view_mode       = VIEW_MODE_MAP

FOLLOW_SCALE    = 12.0   # px per metre in follow-cam
FOLLOW_VIEW_W   = None   # set at runtime to VIEW_W
FOLLOW_VIEW_H   = None   # set at runtime to VIEW_H

# ---- Coordinate mapping ----
VIEW_X = 20
VIEW_Y = 20
VIEW_W = 680
VIEW_H = WINDOW_H - 40

_SCALE     = 1.0
_OFF_X     = 0.0
_OFF_Y     = 0.0

# Saved map transform — restored after each follow-cam frame
_MAP_SCALE = 1.0
_MAP_OFF_X = 0.0
_MAP_OFF_Y = 0.0

# Cached track screen coords — computed once in set_track_bounds(), used every frame.
_TRACK_SCREEN_PTS = []

# World-space cone positions (kept for follow-cam reprojection each frame)
_LEFT_WORLD  = []
_RIGHT_WORLD = []


def set_track_bounds(waypoints):
    global _SCALE, _OFF_X, _OFF_Y, _TRACK_SCREEN_PTS
    if not waypoints:
        return

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]
    margin = TRACK_WIDTH
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)

    _SCALE = min(VIEW_W / span_x, VIEW_H / span_y)

    used_w = span_x * _SCALE
    used_h = span_y * _SCALE
    _OFF_X = VIEW_X + (VIEW_W - used_w) / 2.0 - min_x * _SCALE
    _OFF_Y = VIEW_Y + (VIEW_H - used_h) / 2.0 - min_y * _SCALE

    # Save the map transform so follow-cam can restore it
    global _MAP_SCALE, _MAP_OFF_X, _MAP_OFF_Y
    _MAP_SCALE = _SCALE
    _MAP_OFF_X = _OFF_X
    _MAP_OFF_Y = _OFF_Y

    _TRACK_SCREEN_PTS = [world_to_screen(wx, wy) for (wx, wy) in waypoints]


def update_cone_screen_pts(left_world, right_world):
    """Store world-frame cone positions and cache map-scale screen coords."""
    global LEFT_CONE_PTS, RIGHT_CONE_PTS, _LEFT_WORLD, _RIGHT_WORLD
    _LEFT_WORLD    = list(left_world)
    _RIGHT_WORLD   = list(right_world)
    LEFT_CONE_PTS  = [world_to_screen(wx, wy) for (wx, wy) in left_world]
    RIGHT_CONE_PTS = [world_to_screen(wx, wy) for (wx, wy) in right_world]


def set_follow_transform(car_x, car_y):
    """Override the coordinate transform so the car stays centred in the viewport."""
    global _SCALE, _OFF_X, _OFF_Y
    cx = VIEW_X + VIEW_W / 2.0
    cy = VIEW_Y + VIEW_H / 2.0
    _SCALE = FOLLOW_SCALE
    _OFF_X = cx - car_x * _SCALE
    _OFF_Y = cy - car_y * _SCALE


def restore_map_transform():
    """Restore the full-track coordinate transform."""
    global _SCALE, _OFF_X, _OFF_Y
    _SCALE = _MAP_SCALE
    _OFF_X = _MAP_OFF_X
    _OFF_Y = _MAP_OFF_Y


def world_to_screen(wx, wy):
    sx = _OFF_X + wx * _SCALE
    sy = (2 * VIEW_Y + VIEW_H) - (_OFF_Y + wy * _SCALE)
    return (int(sx), int(sy))


def world_len_to_pixels(metres):
    return max(1, int(metres * _SCALE))


# ---- State parsed from hil_sim stdout ----
class SimState:
    def __init__(self):
        self.x               = 0.0
        self.y               = 0.0
        self.heading         = 0.0
        self.speed_kmh       = 0.0
        self.yaw_degs        = 0.0
        self.fl              = 0.0
        self.fr              = 0.0
        self.rl              = 0.0
        self.rr              = 0.0
        self.tv_enabled      = True
        self.kp              = 80.0
        self.lap             = 0
        self.elapsed_s       = 0.0
        self.steering        = 0.0
        self.slip_angle      = 0.0   # radians
        self.desired_yaw     = 0.0   # rad/s
        self.ax              = 0.0   # m/s^2
        self.ay              = 0.0   # m/s^2
        self.vy              = 0.0   # m/s

    def parse(self, line):
        parts = line.split()
        if len(parts) != 20 or parts[0] != "STATE":
            return False
        try:
            self.x           = float(parts[1])
            self.y           = float(parts[2])
            self.heading     = float(parts[3])
            self.speed_kmh   = float(parts[4])
            self.yaw_degs    = float(parts[5])
            self.fl          = float(parts[6])
            self.fr          = float(parts[7])
            self.rl          = float(parts[8])
            self.rr          = float(parts[9])
            self.tv_enabled  = int(parts[10]) != 0
            self.kp          = float(parts[11])
            self.lap         = int(parts[12])
            self.elapsed_s   = float(parts[13])
            self.steering    = float(parts[14])
            self.slip_angle  = float(parts[15])
            self.desired_yaw = float(parts[16])
            self.ax          = float(parts[17])
            self.ay          = float(parts[18])
            self.vy          = float(parts[19])
        except ValueError:
            return False
        return True


# ---- Lap time tracker ----
class LapTimer:
    """Records completed lap times for TV-on and TV-off separately."""
    def __init__(self):
        self._prev_lap      = 0
        self._lap_start_s   = 0.0
        self._prev_tv       = True
        self.last_on_s      = None
        self.last_off_s     = None
        self.best_on_s      = None
        self.best_off_s     = None

    def update(self, state: SimState):
        if state.lap != self._prev_lap:
            lap_time = state.elapsed_s - self._lap_start_s
            if self._prev_tv:
                self.last_on_s = lap_time
                if self.best_on_s is None or lap_time < self.best_on_s:
                    self.best_on_s = lap_time
            else:
                self.last_off_s = lap_time
                if self.best_off_s is None or lap_time < self.best_off_s:
                    self.best_off_s = lap_time
            self._lap_start_s = state.elapsed_s
            self._prev_lap    = state.lap
        self._prev_tv = state.tv_enabled

    def current_lap_time(self, elapsed_s):
        return elapsed_s - self._lap_start_s


# ---- History lengths ----
YAW_HISTORY_LEN = 100   # ~5 s at 20 Hz
TRAJ_HISTORY_LEN = 500  # trajectory trace
GG_HISTORY_LEN   = 200  # G-G diagram trail


# ---- Background reader thread ----
def reader_thread(proc, line_queue):
    for raw_line in proc.stdout:
        line_queue.put(raw_line.decode("utf-8", errors="replace").rstrip())
    line_queue.put(None)


# ---- Drawing helpers ----

def _stroke_path(surface, points, colour, width):
    if len(points) < 2:
        return
    radius = max(1, width // 2)
    pygame.draw.lines(surface, colour, True, points, width)
    for p in points:
        pygame.draw.circle(surface, colour, p, radius)


def draw_racing_line(surface):
    """Draw the pre-computed optimal racing line through the gate waypoints."""
    if len(TRACK_WAYPOINTS) < 2:
        return
    pts = [world_to_screen(wx, wy) for (wx, wy) in TRACK_WAYPOINTS]
    pygame.draw.lines(surface, RACING_LINE_COL, True, pts, 2)


def draw_track(surface):
    """Draw track surface as a filled polygon between left and right cones, plus cone dots.
    Uses _LEFT_WORLD/_RIGHT_WORLD so it works correctly in both map and follow-cam modes."""
    left  = [world_to_screen(wx, wy) for (wx, wy) in _LEFT_WORLD]
    right = [world_to_screen(wx, wy) for (wx, wy) in _RIGHT_WORLD]

    # Filled asphalt polygon: left cones forward + right cones reversed
    if len(left) >= 2 and len(right) >= 2:
        poly = left + list(reversed(right))
        pygame.draw.polygon(surface, (45, 45, 45), poly)
        pygame.draw.polygon(surface, (70, 70, 70), poly, 1)

    # Cone radius in pixels — fixed 4px min so they're always visible on map,
    # larger in follow-cam where the scale is higher
    r = max(4, world_len_to_pixels(0.18))

    # Blue (left) cones
    for pt in left:
        pygame.draw.circle(surface, CONE_BLUE,      pt, r)
        pygame.draw.circle(surface, CONE_BLUE_EDGE, pt, r, 1)

    # Yellow (right) cones — indices 3-4 are big-orange (start/finish gate)
    for idx, pt in enumerate(right):
        if idx in (3, 4):
            pygame.draw.circle(surface, CONE_ORANGE, pt, r + 2)
            pygame.draw.circle(surface, (255, 200, 50), pt, r + 2, 1)
        else:
            pygame.draw.circle(surface, CONE_YELLOW,      pt, r)
            pygame.draw.circle(surface, CONE_YELLOW_EDGE, pt, r, 1)


def draw_trajectory_trace(surface, traj_deque):
    """Draw TV-on (green) / TV-off (amber) trajectory trace behind the car."""
    if len(traj_deque) < 2:
        return

    run_pts = []
    run_tv  = None

    for (wx, wy, tv) in traj_deque:
        sx, sy = world_to_screen(wx, wy)
        if run_tv is None:
            run_tv = tv
        if tv != run_tv:
            if len(run_pts) >= 2:
                col = TRAJ_ON_COL if run_tv else TRAJ_OFF_COL
                pygame.draw.lines(surface, col, False, run_pts, 2)
            run_pts = run_pts[-1:]   # carry last point to avoid a gap
            run_tv = tv
        run_pts.append((sx, sy))

    if len(run_pts) >= 2:
        col = TRAJ_ON_COL if run_tv else TRAJ_OFF_COL
        pygame.draw.lines(surface, col, False, run_pts, 2)


def draw_car(surface, state):
    """Draw a rotated rectangle for the car body (2.8 m × 1.2 m) with a heading arrow."""
    sx, sy   = world_to_screen(state.x, state.y)
    h        = state.heading
    half_l   = world_len_to_pixels(1.4)    # half car length
    half_w   = world_len_to_pixels(0.5)    # half car width

    cos_h = math.cos(h)
    sin_h = math.sin(h)

    # In screen coords: forward = (cos h, -sin h), right = (sin h, cos h)
    corners = [
        (sx + half_l * cos_h + half_w * sin_h,
         sy - half_l * sin_h + half_w * cos_h),   # front-right
        (sx + half_l * cos_h - half_w * sin_h,
         sy - half_l * sin_h - half_w * cos_h),   # front-left
        (sx - half_l * cos_h - half_w * sin_h,
         sy + half_l * sin_h - half_w * cos_h),   # rear-left
        (sx - half_l * cos_h + half_w * sin_h,
         sy + half_l * sin_h + half_w * cos_h),   # rear-right
    ]
    ipts = [(int(x), int(y)) for (x, y) in corners]

    pygame.draw.polygon(surface, CAR_COLOUR,  ipts)
    pygame.draw.polygon(surface, CAR_OUTLINE, ipts, 2)


def draw_torque_bars(surface, font, state, panel_x, panel_y):
    MAX_T     = 100.0
    BAR_W     = 50
    HALF_H    = 32
    BAR_MAX_H = HALF_H * 2
    GAP       = 10
    LABEL_H   = 26

    positions = [
        ("FL", FL_COL, state.fl,  panel_x,               panel_y),
        ("FR", FR_COL, state.fr,  panel_x + BAR_W + GAP, panel_y),
        ("RL", RL_COL, state.rl,  panel_x,               panel_y + BAR_MAX_H + LABEL_H + GAP),
        ("RR", RR_COL, state.rr,  panel_x + BAR_W + GAP, panel_y + BAR_MAX_H + LABEL_H + GAP),
    ]

    for lbl_text, colour, torque, bx, by in positions:
        zero_y = by + HALF_H

        pygame.draw.rect(surface, MID_GREY, (bx, by, BAR_W, BAR_MAX_H))

        if torque > 0:
            fill_h = int(HALF_H * min(torque, MAX_T) / MAX_T)
            pygame.draw.rect(surface, colour, (bx, zero_y - fill_h, BAR_W, fill_h))
        elif torque < 0:
            fill_h = int(HALF_H * min(-torque, MAX_T) / MAX_T)
            pygame.draw.rect(surface, REGEN_COL, (bx, zero_y, BAR_W, fill_h))

        pygame.draw.rect(surface, LIGHT_GREY, (bx, by, BAR_W, BAR_MAX_H), 1)
        pygame.draw.line(surface, LIGHT_GREY, (bx, zero_y), (bx + BAR_W, zero_y), 1)

        lbl = font.render(lbl_text, True, LIGHT_GREY)
        surface.blit(lbl, (bx + BAR_W // 2 - lbl.get_width() // 2, by + BAR_MAX_H + 2))

        val = font.render(f"{torque:+.0f}", True, WHITE)
        surface.blit(val, (bx + BAR_W // 2 - val.get_width() // 2, by + BAR_MAX_H + 13))


def draw_steering_wheel(surface, font_small, state, cx, cy, radius=28):
    MAX_STEER_RAD   = 0.35
    MAX_VISUAL_DEG  = 90.0

    steer_visual_deg = math.degrees(state.steering) * (MAX_VISUAL_DEG / math.degrees(MAX_STEER_RAD))

    pygame.draw.circle(surface, MID_GREY,   (cx, cy), radius, 3)
    pygame.draw.circle(surface, LIGHT_GREY, (cx, cy), radius, 1)

    for spoke_base_deg in (90, 210, 330):
        angle_rad = math.radians(spoke_base_deg + steer_visual_deg)
        sx = cx + int(radius * math.cos(angle_rad))
        sy = cy - int(radius * math.sin(angle_rad))
        ox = cx - int((radius // 2) * math.cos(angle_rad))
        oy = cy + int((radius // 2) * math.sin(angle_rad))
        pygame.draw.line(surface, LIGHT_GREY, (ox, oy), (sx, sy), 2)

    pygame.draw.circle(surface, LIGHT_GREY, (cx, cy), 4)

    steer_deg = math.degrees(state.steering)
    lbl = font_small.render(f"{steer_deg:+.1f}°", True, WHITE)
    surface.blit(lbl, (cx - lbl.get_width() // 2, cy + radius + 4))


def draw_slip_gauge(surface, font_small, state, gx, gy, width=110, height=14):
    MAX_SLIP_DEG = 10.0
    slip_deg = abs(math.degrees(state.slip_angle))
    fill_frac = min(slip_deg / MAX_SLIP_DEG, 1.0)

    if slip_deg < 3.0:
        bar_col = SLIP_GREEN
    elif slip_deg < 6.0:
        bar_col = SLIP_YELLOW
    else:
        bar_col = SLIP_RED

    lbl = font_small.render(f"SLIP  {slip_deg:.1f}°", True, WHITE)
    surface.blit(lbl, (gx, gy))
    bar_y = gy + lbl.get_height() + 2

    pygame.draw.rect(surface, MID_GREY,   (gx, bar_y, width, height))
    pygame.draw.rect(surface, bar_col,    (gx, bar_y, int(width * fill_frac), height))
    pygame.draw.rect(surface, LIGHT_GREY, (gx, bar_y, width, height), 1)


def draw_us_os_bar(surface, font_small, state, gx, gy, width=270, height=14):
    """
    Understeer / oversteer indicator.
    Value = desired_yaw - actual_yaw.
    Positive = understeer (nose goes wide), negative = oversteer (rear goes wide).
    """
    MAX_ERR_RAD_S = 1.5
    yaw_err = state.desired_yaw - math.radians(state.yaw_degs)

    lbl = font_small.render("UNDERSTEER  ←  0  →  OVERSTEER", True, LIGHT_GREY)
    surface.blit(lbl, (gx, gy))
    bar_y = gy + lbl.get_height() + 2

    half_w   = width // 2
    centre_x = gx + half_w

    pygame.draw.rect(surface, MID_GREY, (gx, bar_y, width, height))
    pygame.draw.line(surface, LIGHT_GREY, (centre_x, bar_y), (centre_x, bar_y + height), 1)

    frac = max(-1.0, min(1.0, yaw_err / MAX_ERR_RAD_S))

    if frac > 0.0:
        fill_w = int(half_w * frac)
        pygame.draw.rect(surface, US_COL, (centre_x, bar_y, fill_w, height))
    elif frac < 0.0:
        fill_w = int(half_w * (-frac))
        pygame.draw.rect(surface, OS_COL, (centre_x - fill_w, bar_y, fill_w, height))

    pygame.draw.rect(surface, LIGHT_GREY, (gx, bar_y, width, height), 1)


def draw_yaw_chart(surface, font_small, actual_hist, desired_hist, chart_x, chart_y,
                   chart_w=230, chart_h=70):
    MAX_YAW = 4.0

    pygame.draw.rect(surface, DARK_GREY,  (chart_x, chart_y, chart_w, chart_h))
    pygame.draw.rect(surface, MID_GREY,   (chart_x, chart_y, chart_w, chart_h), 1)

    zero_y = chart_y + chart_h // 2
    pygame.draw.line(surface, MID_GREY, (chart_x, zero_y), (chart_x + chart_w, zero_y), 1)

    def val_to_py(v):
        frac = max(-1.0, min(1.0, v / MAX_YAW))
        return int(zero_y - frac * (chart_h // 2 - 2))

    def draw_series(hist, colour):
        n = len(hist)
        if n < 2:
            return
        pts = []
        for i, v in enumerate(hist):
            px = chart_x + int(i * (chart_w - 1) / max(YAW_HISTORY_LEN - 1, 1))
            py = val_to_py(v)
            pts.append((px, py))
        pygame.draw.lines(surface, colour, False, pts, 2)

    draw_series(desired_hist, YAW_DESIRED_COL)
    draw_series(actual_hist,  YAW_ACTUAL_COL)

    lx = chart_x + 4
    ly = chart_y + 2
    pygame.draw.line(surface, YAW_DESIRED_COL, (lx, ly + 5), (lx + 12, ly + 5), 2)
    surface.blit(font_small.render("desired", True, YAW_DESIRED_COL), (lx + 15, ly))
    ly2 = ly + 14
    pygame.draw.line(surface, YAW_ACTUAL_COL, (lx, ly2 + 5), (lx + 12, ly2 + 5), 2)
    surface.blit(font_small.render("actual",  True, YAW_ACTUAL_COL),  (lx + 15, ly2))


def draw_gg_diagram(surface, font_small, gg_hist, gx, gy, size=130):
    """
    G-G (friction circle) diagram.  Shows longitudinal (ax) vs lateral (ay) acceleration.
    Scale: ±1.5 g. Friction limit circle drawn at 1.2 g (MU_TYRE).
    """
    G_SCALE = 1.5
    MU_G    = 1.2
    CX      = gx + size // 2
    CY      = gy + size // 2
    HALF    = size // 2 - 4

    pygame.draw.rect(surface, DARK_GREY, (gx, gy, size, size))
    pygame.draw.rect(surface, MID_GREY,  (gx, gy, size, size), 1)

    # Axis lines
    pygame.draw.line(surface, MID_GREY, (CX, gy + 2),    (CX, gy + size - 2), 1)
    pygame.draw.line(surface, MID_GREY, (gx + 2, CY),    (gx + size - 2, CY), 1)

    # Friction limit circle
    fric_r = int(HALF * MU_G / G_SCALE)
    pygame.draw.circle(surface, LIGHT_GREY, (CX, CY), fric_r, 1)

    def gg_to_screen(ax_g, ay_g):
        px = CX + int(ay_g / G_SCALE * HALF)       # lateral = horizontal
        py = CY - int(ax_g / G_SCALE * HALF)       # longitudinal = vertical (ax+ = up = accel)
        return (px, py)

    # History trail
    trail = list(gg_hist)
    n = len(trail)
    for i, (ax_g, ay_g) in enumerate(trail):
        alpha = max(60, int(255 * i / max(n - 1, 1)))
        col   = (max(0, GG_TRAIL_COL[0] - (255 - alpha)),
                 max(0, GG_TRAIL_COL[1] - (255 - alpha)),
                 max(0, GG_TRAIL_COL[2]))
        sx, sy = gg_to_screen(ax_g, ay_g)
        if gx <= sx < gx + size and gy <= sy < gy + size:
            pygame.draw.circle(surface, col, (sx, sy), 2)

    # Current point
    if trail:
        ax_g, ay_g = trail[-1]
        sx, sy = gg_to_screen(ax_g, ay_g)
        if gx <= sx < gx + size and gy <= sy < gy + size:
            pygame.draw.circle(surface, GG_DOT_COL, (sx, sy), 4)

    # Labels
    surface.blit(font_small.render("G", True, LIGHT_GREY),
                 (gx + 2, gy + size // 2 - 6))
    surface.blit(font_small.render("+a", True, LIGHT_GREY),
                 (CX - 10, gy + 2))
    surface.blit(font_small.render("-a", True, LIGHT_GREY),
                 (CX - 10, gy + size - 14))


def _fmt_lap(t):
    if t is None:
        return "--:--.--"
    mins = int(t) // 60
    secs = t - mins * 60
    return f"{mins}:{secs:05.2f}"


def draw_lap_comparison(surface, font_large, font_small, lap_timer, px, py):
    COL_W = 120

    surface.blit(font_small.render("LAP TIMES", True, LIGHT_GREY), (px, py))
    py += font_small.get_height() + 4

    on_x  = px
    off_x = px + COL_W

    surface.blit(font_small.render("TV ON",  True, TV_ON_COL),  (on_x,  py))
    surface.blit(font_small.render("TV OFF", True, TV_OFF_COL), (off_x, py))
    py += font_small.get_height() + 2

    surface.blit(font_small.render("Last", True, LIGHT_GREY), (px, py))
    py += font_small.get_height() + 1

    on_last  = font_large.render(_fmt_lap(lap_timer.last_on_s),  True, TV_ON_COL)
    off_last = font_large.render(_fmt_lap(lap_timer.last_off_s), True, TV_OFF_COL)
    surface.blit(on_last,  (on_x,  py))
    surface.blit(off_last, (off_x, py))
    py += on_last.get_height() + 4

    surface.blit(font_small.render("Best", True, LIGHT_GREY), (px, py))
    py += font_small.get_height() + 1

    on_best  = font_large.render(_fmt_lap(lap_timer.best_on_s),  True, TV_ON_COL)
    off_best = font_large.render(_fmt_lap(lap_timer.best_off_s), True, TV_OFF_COL)
    surface.blit(on_best,  (on_x,  py))
    surface.blit(off_best, (off_x, py))
    py += on_best.get_height() + 4

    return py


def draw_panel(surface, font_large, font, font_small,
               state, lap_timer, actual_yaw_hist, desired_yaw_hist, gg_hist):
    """Draw the data panel on the right side of the window."""
    px = VIEW_X + VIEW_W + 30
    py = 20
    sep_w = 490

    def sep(y):
        pygame.draw.line(surface, MID_GREY, (px, y), (px + sep_w, y), 1)
        return y + 8

    def label(text, y, colour=LIGHT_GREY):
        s = font_small.render(text, True, colour)
        surface.blit(s, (px, y))
        return y + s.get_height() + 2

    # Title
    title = font_large.render("HIL  Torque  Vectoring", True, WHITE)
    surface.blit(title, (px, py))
    py += title.get_height() + 6
    py = sep(py)

    # TV status + Kp
    tv_text = "TV  ON" if state.tv_enabled else "TV  OFF"
    tv_col  = TV_ON_COL if state.tv_enabled else TV_OFF_COL
    tv_s    = font_large.render(tv_text, True, tv_col)
    surface.blit(tv_s, (px, py))
    kp_s = font.render(f"Kp = {state.kp:.1f}", True, LIGHT_GREY)
    surface.blit(kp_s, (px + tv_s.get_width() + 14, py + 4))
    py += tv_s.get_height() + 6
    py = sep(py)

    # LAP / TIME / CUR LAP  (three-column row)
    cur_lap_s = lap_timer.current_lap_time(state.elapsed_s)
    mins      = int(state.elapsed_s) // 60
    secs      = int(state.elapsed_s) % 60
    col2 = px + 110
    col3 = px + 230

    lap_lbl = font_small.render("LAP",     True, LIGHT_GREY)
    tim_lbl = font_small.render("TIME",    True, LIGHT_GREY)
    cur_lbl = font_small.render("CUR LAP", True, LIGHT_GREY)
    surface.blit(lap_lbl, (px,   py))
    surface.blit(tim_lbl, (col2, py))
    surface.blit(cur_lbl, (col3, py))
    py += lap_lbl.get_height() + 2

    lap_val = font_large.render(f"{state.lap}", True, WHITE)
    tim_val = font_large.render(f"{mins}:{secs:02d}", True, WHITE)
    cur_val = font_large.render(_fmt_lap(cur_lap_s), True, LIGHT_GREY)
    surface.blit(lap_val, (px,   py))
    surface.blit(tim_val, (col2, py))
    surface.blit(cur_val, (col3, py))
    py += lap_val.get_height() + 6
    py = sep(py)

    # Lap time comparison
    py = draw_lap_comparison(surface, font, font_small, lap_timer, px, py)
    py = sep(py)

    # Speed + lateral g
    speed_ms = state.speed_kmh / 3.6
    lat_g    = abs(math.radians(state.yaw_degs) * speed_ms) / 9.81
    col2 = px + 165

    surface.blit(font_small.render("SPEED",  True, LIGHT_GREY), (px,   py))
    surface.blit(font_small.render("LAT G",  True, LIGHT_GREY), (col2, py))
    py += font_small.get_height() + 2
    surface.blit(font_large.render(f"{state.speed_kmh:.1f} km/h", True, WHITE), (px,   py))
    surface.blit(font_large.render(f"{lat_g:.2f} g",              True, WHITE), (col2, py))
    py += font_large.get_height() + 4

    # Yaw rate + steering wheel
    WHEEL_R   = 26
    WHEEL_COL = px + 175

    surface.blit(font_small.render("YAW RATE", True, LIGHT_GREY), (px, py))
    surface.blit(font_small.render("STEER",    True, LIGHT_GREY), (WHEEL_COL - 20, py))
    py += font_small.get_height() + 2

    row_h   = WHEEL_R * 2 + font_small.get_height() + 8
    yaw_val = font_large.render(f"{state.yaw_degs:+.1f} d/s", True, WHITE)
    surface.blit(yaw_val, (px, py + (row_h - yaw_val.get_height()) // 2))

    wheel_cy = py + WHEEL_R + 2
    draw_steering_wheel(surface, font_small, state, WHEEL_COL, wheel_cy, radius=WHEEL_R)

    py += row_h + 4
    py = sep(py)

    # Slip angle bar + understeer/oversteer bar
    draw_slip_gauge(surface, font_small, state, px, py, width=270, height=12)
    py += font_small.get_height() + 2 + 12 + 5

    draw_us_os_bar(surface, font_small, state, px, py, width=270, height=12)
    py += font_small.get_height() + 2 + 12 + 6
    py = sep(py)

    # Yaw chart (left) + G-G diagram (right) on the same row
    GG_SIZE  = 130
    GG_GAP   = 10
    CHART_W  = sep_w - GG_SIZE - GG_GAP

    yaw_top = py
    py = label("YAW RATE  (rad/s)  desired / actual", py)
    draw_yaw_chart(surface, font_small,
                   actual_yaw_hist, desired_yaw_hist,
                   px, py, chart_w=CHART_W, chart_h=65)

    # G-G diagram aligned to the right of the yaw label row, tall enough to span it
    gg_x = px + CHART_W + GG_GAP
    draw_gg_diagram(surface, font_small, gg_hist, gg_x, yaw_top, size=GG_SIZE)

    # Advance py past whichever widget is taller
    py = max(py + 65, yaw_top + GG_SIZE) + 6
    py = sep(py)

    # Torque bars
    py = label("WHEEL TORQUES  (Nm, ±100)", py)
    py += 4
    draw_torque_bars(surface, font_small, state, px, py)
    py += (64 + 26 + 10) * 2 - 10 + 6
    py = sep(py)

    # Controls hint
    s = font_small.render("T  toggle TV    [/]  Kp±5    Q  quit", True, MID_GREY)
    surface.blit(s, (px, py))


# ---- Main ----

def main():
    if not os.path.isfile(HIL_SIM_EXE):
        print(f"ERROR: Could not find {HIL_SIM_EXE}")
        print("Build it first with:  make  (from the repo root, in MSYS2 MinGW x64 shell)")
        sys.exit(1)

    proc = subprocess.Popen(
        [HIL_SIM_EXE],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    global TRACK_WAYPOINTS
    waypoints   = []
    left_world  = []
    right_world = []

    def read_line():
        raw = proc.stdout.readline()
        if not raw:
            print("ERROR: hil_sim exited during startup")
            sys.exit(1)
        return raw.decode("utf-8", errors="replace").rstrip()

    # --- Centreline waypoints ---
    while True:
        line = read_line()
        if line == "END_TRACK":
            break
        parts = line.split()
        if len(parts) == 3 and parts[0] == "WP":
            try:
                waypoints.append((float(parts[1]), float(parts[2])))
            except ValueError:
                pass

    # --- Left (blue) cones ---
    while True:
        line = read_line()
        if line == "END_LEFT_CONES":
            break
        parts = line.split()
        if len(parts) == 3 and parts[0] == "CONE":
            try:
                left_world.append((float(parts[1]), float(parts[2])))
            except ValueError:
                pass

    # --- Right (yellow/orange) cones ---
    while True:
        line = read_line()
        if line == "END_RIGHT_CONES":
            break
        parts = line.split()
        if len(parts) == 3 and parts[0] == "CONE":
            try:
                right_world.append((float(parts[1]), float(parts[2])))
            except ValueError:
                pass

    TRACK_WAYPOINTS = waypoints
    set_track_bounds(waypoints)
    update_cone_screen_pts(left_world, right_world)

    line_q = queue.Queue()
    t = threading.Thread(target=reader_thread, args=(proc, line_q), daemon=True)
    t.start()

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("HIL Torque Vectoring")
    clock  = pygame.time.Clock()

    font_large  = pygame.font.SysFont("consolas", 20, bold=True)
    font_medium = pygame.font.SysFont("consolas", 16)
    font_small  = pygame.font.SysFont("consolas", 12)

    state            = SimState()
    lap_timer        = LapTimer()
    actual_yaw_hist  = collections.deque([0.0] * YAW_HISTORY_LEN, maxlen=YAW_HISTORY_LEN)
    desired_yaw_hist = collections.deque([0.0] * YAW_HISTORY_LEN, maxlen=YAW_HISTORY_LEN)
    traj_deque       = collections.deque(maxlen=TRAJ_HISTORY_LEN)
    gg_hist          = collections.deque(maxlen=GG_HISTORY_LEN)
    running          = True
    view_mode        = VIEW_MODE_MAP

    while running:
        clock.tick(60)

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
            if state.parse(latest):
                actual_yaw_hist.append(math.radians(state.yaw_degs))
                desired_yaw_hist.append(state.desired_yaw)
                traj_deque.append((state.x, state.y, state.tv_enabled))
                gg_hist.append((state.ax / 9.81, state.ay / 9.81))
                lap_timer.update(state)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                # M toggles map / follow-cam view
                if event.key == pygame.K_m:
                    view_mode = VIEW_MODE_FOLLOW if view_mode == VIEW_MODE_MAP else VIEW_MODE_MAP

                key_map = {
                    pygame.K_t:            b"t",
                    pygame.K_LEFTBRACKET:  b"[",
                    pygame.K_RIGHTBRACKET: b"]",
                    pygame.K_q:            b"q",
                    pygame.K_ESCAPE:       b"q",
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

        screen.fill(BLACK)
        pygame.draw.rect(screen, DARK_GREY, (VIEW_X, VIEW_Y, VIEW_W, VIEW_H))

        # Apply view transform before drawing, restore after
        if view_mode == VIEW_MODE_FOLLOW:
            set_follow_transform(state.x, state.y)

        # Clip all track drawing to the map viewport so it cannot bleed into the data panel
        screen.set_clip(pygame.Rect(VIEW_X, VIEW_Y, VIEW_W, VIEW_H))
        draw_track(screen)
        draw_racing_line(screen)
        draw_trajectory_trace(screen, traj_deque)
        draw_car(screen, state)
        screen.set_clip(None)

        if view_mode == VIEW_MODE_FOLLOW:
            restore_map_transform()
            # Label so the user knows which mode is active
            mode_lbl = font_small.render("FOLLOW CAM  [M] map", True, MID_GREY)
        else:
            mode_lbl = font_small.render("MAP VIEW  [M] follow", True, MID_GREY)
        screen.blit(mode_lbl, (VIEW_X + 4, VIEW_Y + VIEW_H - mode_lbl.get_height() - 4))

        draw_panel(screen, font_large, font_medium, font_small,
                   state, lap_timer, actual_yaw_hist, desired_yaw_hist, gg_hist)

        pygame.draw.rect(screen, MID_GREY, (VIEW_X, VIEW_Y, VIEW_W, VIEW_H), 1)
        pygame.display.flip()

    try:
        proc.stdin.write(b"q")
        proc.stdin.flush()
    except OSError:
        pass
    proc.terminate()
    pygame.quit()


if __name__ == "__main__":
    main()
