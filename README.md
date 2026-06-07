# HIL Torque Vectoring

A race car simulation with torque vectoring, written in C.

The simulation pretends to be a real race car and feeds sensor data to ECU code. The ECU code decides how to split drive torque between the four wheels. This lets you test and tune the ECU logic without a real car. That is what Hardware-in-the-Loop (HIL) testing means.

When you run it, a pygame window opens showing the FSG 2024 cone track, the car
moving around it in real time, and a live data panel on the right with speed,
yaw rate, lap count, and a bar chart of the four wheel torques.


## How to build and run

You need two things:

**1. Build the C simulation** (requires gcc and make — use MSYS2 on Windows):

```
cd HIL-Torque-Vectoring
make
```

This produces `HIL_Firmware/build/hil_sim` (or `hil_sim.exe` on Windows).

**2. Run the Python visualiser** (requires Python 3 and pygame):

```
pip install pygame
python visualiser.py
```

`visualiser.py` launches `hil_sim` automatically and opens the window.
You do not need to run `hil_sim` yourself.

Controls (click the pygame window first so it captures keypresses):

| Key | Action |
|-----|--------|
| T | Toggle torque vectoring on / off |
| [ | Decrease TV gain (Kp) by 5 |
| ] | Increase TV gain (Kp) by 5 |
| Q or Escape | Quit |


## Project layout

```
HIL-Torque-Vectoring/
|
|-- visualiser.py             Python/pygame window. Launches hil_sim and draws it live.
|-- shared/
|   `-- tv_interface.h        The data types shared between the HIL and ECU sides.
|                             In a real system these would cross a CAN bus.
|
|-- HIL_Firmware/             The simulation host. Pretends to be the real car.
|   |-- main.c                The simulation loop. Prints state as CSV to stdout.
|   |-- vehicle_model.c/.h    The physics of the car (position, speed, yaw rate).
|   |-- track.c/.h            The FSG 2024 cone track defined as boundary cone positions.
|   |-- path_planning.c/.h    Builds the racing-line waypoints from the cone data.
|   |-- motion_control.c/.h   The virtual driver: Stanley steering + curvature speed planner.
|   `-- Makefile              Builds everything into one binary called hil_sim.
|
|-- ECU_Firmware/             The ECU side. Only sees sensor data, not the full car state.
|   |-- torque_vectoring.c    The TV algorithm: splits torque between the four wheels.
|   `-- torque_vectoring.h    Function prototype and tuning constants.
|
`-- ECU_Hardware/
    `-- README.md             Placeholder for future PCB and wiring documentation.
```

There are three roles in the system, kept in separate files on purpose:

| Role | File | What it does |
|------|------|--------------|
| Driver | `motion_control.c` | Stanley path tracking and curvature-based speed planning |
| Car | `vehicle_model.c` | Updates the physics of the car each tick |
| ECU | `torque_vectoring.c` | Splits the throttle demand into four wheel torques |


## How it works

Each tick (100 times per second), this happens:

```
Motion control (driver)
    projects car onto nearest racing-line segment (Stanley)
    sets steering angle
    plans target speed from upcoming curvature (two-pass backward sweep)
    returns driver torque demand (total Nm)
        |
        v
ECU Firmware (torque_vectoring_update)
    receives sensor data: yaw rate, speed, steering angle, wheel speeds
    fuses IMU yaw rate with wheel-speed estimate
    computes four wheel torques
        |
        v
Vehicle Model (vehicle_model_update)
    applies wheel torques and steering to update:
    position (x, y), heading, speed, yaw rate
        |
        v
Track (track_update)
    checks if car has reached the next waypoint
    advances to the next one, counts laps
        |
        v
hil_sim prints a STATE line to stdout (every 5 ticks = 20 Hz)
    x, y, heading, speed, yaw_rate, four wheel torques, TV state, lap, time, ...
        |
        v  (stdout pipe)
visualiser.py reads the STATE line
    draws the track, cones, car, and data panel in the pygame window
```

The ECU code only ever receives a `SensorData` struct. It never touches the full
`VehicleState`. That is the HIL boundary. In a real system, the real ECU would
receive real sensor readings over CAN, and the HIL PC would receive real torque
commands back. The code structure already reflects this.


## The track

The track is the FSG 2024 Formula Student Germany endurance layout, defined by
228 measured cone positions (117 left/blue, 111 right/yellow-orange).

At startup, `path_plan()` builds the racing-line waypoints from the raw cone data
in three stages:

1. **Gate extraction** — each left cone is paired with its nearest right cone to
   form a gate. Gates wider than 10 m are rejected.

2. **Centreline resampling** — gate midpoints are resampled to uniform 2.5 m
   spacing so the optimiser sees evenly spaced control points.

3. **Minimum-curvature racing line** — each uniformly spaced point carries a
   lateral offset along the track normal. A Gauss-Seidel sweep of the
   `[1, -4, 6, -4, 1]` bending-energy stencil minimises total curvature (wider
   arcs through corners = higher speed limit) while keeping the line inside the
   cone corridor.


## The vehicle model

The car uses a single-track bicycle model with a Pacejka lateral tyre model.

It tracks: position (x, y), heading angle, speed, yaw rate, body slip angle,
lateral velocity, and longitudinal/lateral accelerations. Load transfer is
modelled with a first-order lag (~80 ms) to avoid algebraic feedback loops.

Parameters match the M25 Formula Student car (260 kg, 1.55 m wheelbase,
1.30 m track width, four 29.4 Nm peak motors with a 15.47:1 gear ratio).


## The torque vectoring algorithm

The algorithm lives entirely in `ECU_Firmware/torque_vectoring.c`. Here is what it does.

When a car goes around a corner, the outside wheels travel a longer arc than
the inside wheels. If all four wheels get the same torque, the car tends to
push wide (understeer). Giving more torque to the outside wheels helps the car
rotate into the corner.

The algorithm measures this using yaw rate control:

**Step 1: Desired yaw rate**

At any given speed and steering angle, there is a yaw rate the car should have
if it is following the intended path. This comes from simple geometry:

```
desired_yaw_rate = speed * tan(steering_angle) / wheelbase
```

**Step 1b: Fuse IMU with wheel-speed estimate**

The outer wheels spin faster than the inner wheels. The yaw rate follows:

```
r_wheels = (v_right - v_left) / track_width
```

The final estimate blends the IMU (primary) with the wheel-speed channel
(25% weight) to corroborate the gyro while staying robust under tyre slip.

**Step 2: Yaw rate error**

```
error = desired_yaw_rate - fused_yaw_rate
```

A deadband of ±0.03 rad/s suppresses chatter from sensor noise and the
steady-state bias of the kinematic desired-yaw estimate.

**Step 3: Torque bias (speed-dependent gain)**

```
effective_Kp = Kp * (12 m/s / speed)    [capped at 3× Kp near standstill]
torque_bias = effective_Kp * error
```

`Kp` starts at 60 Nm per rad/s and can be changed at runtime with `[` and `]`.
Scaling inversely with speed keeps the yaw moment response consistent across
the speed range.

**Step 4: Apply the bias**

The total driver torque is split equally across all four motors. Within each
axle the bias shifts torque to the outer wheels:

```
outer_wheel = (total / 4) + (bias / 2)
inner_wheel = (total / 4) - (bias / 2)
```

**Step 5: Clamp**

Each wheel torque is clamped to the motor limits (−100 Nm regen to +29.4 Nm drive).
The bias itself is clamped to ±14.7 Nm (half peak), allowing the inner wheel
to go into regen while the outer drives — creating a stronger yaw moment than
a drive-only differential.

That is the whole algorithm. A real production TV system would also include
feedforward terms, a tyre model, a stability controller, and driver preference
maps. This version captures the essential idea in about 60 lines.


## The motion controller (virtual driver)

**Steering — Stanley controller**

The front axle is projected onto the nearest racing-line segment each tick.
The commanded steering angle combines:
- A curvature feedforward term (pre-aligns the wheel with the corner)
- Heading error (aligns the car with the path tangent)
- Cross-track error term `atan(K_CTE * cte / (v + K_soft))` (pulls back to centreline)
- A gentle cone-repulsion safety net

A slew-rate limit (4 rad/s road-wheel) prevents unrealistic instantaneous
steering snaps.

**Speed — two-pass backward-sweep planner**

1. Forward pass: for each waypoint within 80 m, compute `v_limit = sqrt(a_lat / kappa)`.
2. Backward pass: propagate braking from the furthest waypoint back: `v[i] = min(v_limit[i], sqrt(v[i+1]^2 + 2*a_brake*ds))`.

This models late-braking into corners and full-throttle exits.

**Throttle/brake**

A proportional speed controller with a traction-circle cut (throttle scales with
`sqrt(1 - (ay / ay_ref)^2)`) backs off power mid-corner so the car does not
power-understeer on exit.


## Things to try

**Toggle TV on and off**
Press `T` while the simulation is running. Watch the torque bar chart on the right.
With TV off, all four wheels get the same torque. With TV on, the outer wheels get
more torque through corners.

**Change the gain**
Press `[` to decrease Kp or `]` to increase it. A very high gain (try 200+)
makes the TV system very aggressive. A gain of 0 is the same as TV off.

**Read the code**
The torque vectoring algorithm is about 60 lines of C with comments explaining
every step. The vehicle model is the next place to read. Start with
`ECU_Firmware/torque_vectoring.c` then move to `HIL_Firmware/vehicle_model.c`.

**Change the track**
Open `HIL_Firmware/track.c`. The cone positions are measured data from the real
FSG 2024 event and can be edited to create a custom layout. After changing them,
`path_plan()` automatically rebuilds the racing line.

**Change the driver**
Open `HIL_Firmware/motion_control.h`. Change `TARGET_SPEED_MS` to make the car
go faster or slower. Change `K_CTE` to make the Stanley tracker more or less
aggressive. Change `MAX_LATERAL_ACCEL_MS2` to loosen or tighten the corner
speed limits.


## What is not modelled

This simulation leaves out a lot of things on purpose. The goal is to be
readable, not to be a racing simulator.

Not modelled:
- Weight transfer under braking and acceleration (load transfer lag is modelled but not full weight transfer)
- Aerodynamic drag and downforce
- Brake torque vectoring (only drive torque is vectored here)
- Suspension travel and body roll
- Motor and inverter dynamics
- Battery state of charge

Do not use this to predict real lap times or vehicle limits. Use it to understand
how torque vectoring control logic works, and to test that the ECU code behaves
correctly in response to sensor inputs.


## Platform notes

**Python visualiser:** Requires Python 3 and pygame. Install with:
```
pip install -r requirements.txt
```

**Building hil_sim on Linux / macOS:** Works out of the box with any gcc installation.

**Building hil_sim on Windows:** Use MSYS2 with the MinGW-w64 toolchain.

Install MSYS2 from https://www.msys2.org, then in the MSYS2 shell:

```
pacman -S mingw-w64-x86_64-gcc make
```

Then open the MinGW 64-bit shell, navigate to the repo, and run `make`.
You can then run `python visualiser.py` from a normal Windows terminal (cmd,
PowerShell, or the MSYS2 shell — all work as long as Python is on your PATH).
