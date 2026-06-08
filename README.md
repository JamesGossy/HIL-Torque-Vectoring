# HIL Torque Vectoring

A race-car torque-vectoring simulation, written in C.

The simulation pretends to be a real race car and feeds sensor data to ECU code.
The ECU decides how to split drive torque between the four wheels. This lets you
test and tune the ECU logic without a real car, which is what Hardware-in-the-Loop
(HIL) testing means.

When you run it, a pygame window opens showing the FSG 2024 cone track, the car
driving round it in real time, and a live data panel with speed, yaw rate, lap
count, and a bar chart of the four wheel torques.


## How to build and run

**1. Build the C simulation** (needs gcc and make; use MSYS2 on Windows):

```
make
```

This produces `HIL_Firmware/build/hil_sim` (or `hil_sim.exe` on Windows).

**2. Run the Python visualiser** (needs Python 3 and pygame):

```
pip install pygame
python visualiser.py
```

`visualiser.py` launches `hil_sim` itself and opens the window. You do not need
to run `hil_sim` by hand.

Controls (click the window first so it captures keypresses):

| Key | Action |
|-----|--------|
| T | Toggle torque vectoring on / off |
| [ | Decrease TV gain (Kp) by 5 |
| ] | Increase TV gain (Kp) by 5 |
| M | Toggle map / follow-cam view |
| F | Toggle fullscreen |
| Q or Escape | Quit |

The window is resizable and movable: drag an edge to resize, or press F for
fullscreen. The view scales to fit at any size.


## Project layout

```
HIL-Torque-Vectoring/
|
|-- Makefile                  Builds hil_sim, the tests, and the tools.
|-- visualiser.py             Pygame window. Launches hil_sim and draws it live.
|
|-- shared/                   Shared by both the HIL and ECU sides.
|   |-- tv_interface.h        The data types that cross the HIL/ECU boundary.
|   |-- vehicle_config.h      The car's physical constants (mass, geometry, tyres).
|   `-- parameters_config.h   Every tunable parameter (driver gains, TV gains).
|
|-- HIL_Firmware/             The simulation host. Pretends to be the real car.
|   |-- src/main.c            The sim loop. Streams STATE lines to stdout.
|   |-- src/vehicle_model.c   The car physics (per-wheel dynamic bicycle model).
|   |-- src/track.c           The FSG 2024 cone track (measured cone positions).
|   |-- src/path_planning.c   Builds the racing-line waypoints from the cones.
|   `-- src/motion_control.c  The virtual driver: Pure Pursuit + speed planner.
|
|-- ECU_Firmware/             The ECU side. Only sees sensor data.
|   `-- src/torque_vectoring.c  The TV algorithm: splits torque four ways.
|
|-- tests/                    Unit tests (make test).
|-- tools/                    Diagnostics: eval_lap, perf_sim, sweep.sh, CI helpers.
`-- ECU_Hardware/             Placeholder for PCB and wiring docs.
```

There are three roles, kept in separate files on purpose:

| Role | File | What it does |
|------|------|--------------|
| Driver | `motion_control.c` | Pure Pursuit path tracking and curvature speed planning |
| Car | `vehicle_model.c` | Updates the car physics each tick |
| ECU | `torque_vectoring.c` | Splits the throttle demand into four wheel torques |

The ECU only ever receives a `SensorData` struct. It never touches the full
vehicle state. That is the HIL boundary. The `make` build checks it by compiling
`torque_vectoring.c` with only `shared/` and its own header on the include path,
so the ECU cannot accidentally depend on anything in `HIL_Firmware/`.


## Tuning parameters

Every value you would tune lives in `shared/parameters_config.h`: the driver
gains (look-ahead, cross-track pull, speed budget, throttle/brake), the cone
safety net, and the torque-vectoring gains. The car's physical constants (mass,
geometry, tyre coefficients) are separate, in `shared/vehicle_config.h`, because
they describe the hardware rather than a tuning choice.

The highest-leverage gains are wrapped in `#ifndef`, so the sweep tool can
override them at compile time with `-Dname=value` without editing the file.


## How it works

Each tick (100 times a second):

```
motion_control (driver)
    projects the car onto the racing line, sets the steering angle
    plans a target speed from the upcoming curvature
    returns a driver torque demand (total Nm)
        |
        v
torque_vectoring (ECU)
    receives sensor data: yaw rate, speed, steering, wheel speeds
    splits the demand into four wheel torques
        |
        v
vehicle_model (car)
    applies the torques and steering, updates position, heading, speed, yaw
        |
        v
track
    advances the waypoint index and counts laps
        |
        v
hil_sim prints a STATE line (20 Hz) which the visualiser reads and draws
```


## The track

The FSG 2024 Formula Student Germany endurance layout, defined by 228 measured
boundary cones (117 left, 111 right). At startup `path_plan()` builds the racing
line in three stages:

1. **Gate extraction**: pair each left cone with its nearest right cone.
2. **Centreline resampling**: resample the gate midpoints to uniform 2.5 m spacing.
3. **Minimum-curvature line**: sweep a bending-energy stencil to find the
   lowest-curvature line inside the cone corridor. Wider arcs through corners
   mean a higher corner-speed limit.


## The vehicle model

A 3-DOF planar model (vx, vy, yaw) evaluated per wheel:

- Per-wheel Pacejka lateral tyre forces (load-sensitive and nonlinear).
- Quadratic aerodynamic downforce and drag.
- Longitudinal and lateral load transfer giving individual wheel loads, with a
  first-order lag so the transfer does not form an algebraic feedback loop.
- A per-wheel friction circle, so a wheel cannot put down more combined
  longitudinal and lateral force than its grip allows.
- Ackermann per-wheel steering and per-wheel drive force from motor torque,
  which is what makes torque vectoring possible.

Parameters match the M25 Formula Student car (260 kg, 1.55 m wheelbase, 1.30 m
track, four 29.4 Nm motors through a 15.47:1 gear).


## The torque vectoring algorithm

It lives entirely in `ECU_Firmware/src/torque_vectoring.c`. It is a model-based
yaw-moment controller: it shifts torque from the inner to the outer wheels of
each axle so the car rotates at the rate the steering is asking for, no more
(oversteer) and no less (understeer).

**Step 1: Desired yaw rate.** A `v^2` understeer term bends the kinematic
estimate down to the yaw rate the car can actually reach:

```
desired_yaw_rate = speed * tan(steering) / (wheelbase + K_us * speed^2)
```

**Step 1b: Fuse the IMU with a wheel-speed estimate.** The outer wheels spin
faster than the inner ones, so `r_wheels = (v_right - v_left) / track`. The final
estimate blends the IMU (primary) with this wheel-speed channel.

**Step 2: Yaw error**, with a deadband so the bias does not chatter on noise.

**Step 3: Feedforward.** Pre-loads the differential from the cornering demand the
instant the steering moves, so the yaw moment is there before any error develops.
This is the biggest contributor to good corner tracking.

**Step 4: PID feedback** (with anti-windup):

```
P = effective_Kp * error      [Kp scales inversely with speed]
I = Ki * integral(error)      [erases the standing understeer]
D = Kd * d(error)/dt          [damps the turn-in]
bias = feedforward + P + I + D
```

`Kp` defaults to 60 and is tunable at runtime with `[` and `]`. `Ki` and `Kd` are
fractions of it. The integrator freezes when the bias saturates, so it cannot
wind up.

**Step 5: Rear-biased split.** The driver torque is split evenly across the four
motors. The yaw-moment differential is then split front/rear with the larger
share to the rear (the rear tyres spend less grip on steering, so they have more
to give the differential).

**Step 6: Differential-preserving clamp.** Each wheel torque is clamped to the
motor limits (+29.4 Nm drive to -100 Nm regen). When the outer wheel saturates,
the clipped amount is pushed onto the inner wheel so the commanded differential,
and the yaw moment, is held as far as the motors allow.


## The motion controller (virtual driver)

**Steering: Pure Pursuit.** It aims at a point on the racing line a look-ahead
distance ahead of the rear axle and computes the single-arc steer angle that
drives the rear axle through it. A short look-ahead at low speed commits the car
to a tight apex; a longer one at speed keeps the line smooth. A small cross-track
trim pulls the car back when it drifts off the line, and a cone repulsion term is
a safety net near the boundary.

**Speed: two-pass planner.** A forward pass caps each waypoint's speed from its
curvature (`v = sqrt(a_lat / kappa)`), then a backward pass propagates braking
back from the furthest waypoint. This models late braking into corners and full
power out of them.

**Throttle/brake.** A proportional-plus-integral speed controller with a
traction-circle cut (throttle scales with `sqrt(1 - (ay/ay_ref)^2)`) backs off
power mid-corner, and a steering-saturation cut fades throttle near full lock so
the car does not power wide.


## Evaluating changes

Judge any driver, vehicle, or controller change with the headless lap evaluator,
not by eye and not by the unit tests alone:

```
make eval
```

It runs the full motion-control to ECU to vehicle loop over the FSG track as fast
as possible and prints lap-tracking metrics, ending in a machine-readable
`RESULT` line. A good change keeps **off-track ticks at 0** and **completes a
lap**, and should not regress mean or worst cross-track error or lap time.

`tools/sweep.sh` sweeps the highest-leverage gains and prints a table sorted by
clean-lap time. Only accept a config with 0 off-track ticks.


## Things to try

**Toggle TV on and off** with `T`, and watch the torque bars. With TV off all
four wheels get the same torque; with TV on the outer wheels get more through
corners.

**Change the gain** with `[` and `]`. A very high gain makes TV aggressive; a
gain of 0 is the same as TV off.

**Change the driver.** Edit `shared/parameters_config.h`. `TARGET_SPEED_MS` sets
the straight-line cruise speed, `K_CTE_PP` sets how hard the tracker pulls back
to the line, and `MAX_LATERAL_ACCEL_MS2` sets the corner-speed budget. The speed
budget and the cross-track pull interact, so after raising the budget you usually
need more pull to stay clean. Validate with `make eval`.

**Change the track.** Edit the cone positions in `HIL_Firmware/src/track.c`.
`path_plan()` rebuilds the racing line automatically.


## What is not modelled

This sim leaves things out on purpose, to stay readable rather than be a full
racing simulator. Not modelled: suspension travel and body roll, motor and
inverter dynamics, battery state, and brake torque vectoring (only drive torque
is vectored). Use it to understand how the control logic works and to test that
the ECU behaves correctly, not to predict real lap times.


## Platform notes

**Visualiser:** needs Python 3 and pygame (`pip install pygame`).

**Building on Linux / macOS:** works with any gcc.

**Building on Windows:** use MSYS2 with the MinGW-w64 toolchain. Install MSYS2
from https://www.msys2.org, then in the MSYS2 shell:

```
pacman -S mingw-w64-x86_64-gcc make
```

Open the MinGW 64-bit shell, go to the repo, and run `make`. You can then run
`python visualiser.py` from any terminal that has Python on its PATH.
