# HIL Torque Vectoring

A simple race car simulation with torque vectoring, written in C.

The simulation pretends to be a real race car and feeds sensor data to ECU code. The ECU code decides how to split drive torque between the four wheels. This lets you test and tune the ECU logic without a real car. That is what Hardware-in-the-Loop (HIL) testing means.

When you run it, a pygame window opens showing the figure-8 track, the car
moving around it in real time, and a live data panel on the right with speed,
yaw rate, lap count, and a bar chart of the four wheel torques.


## How to build and run

You need two things:

**1. Build the C simulation** (requires gcc and make -- use MSYS2 on Windows):

```
cd HIL-Torque-Vectoring
make
```

This produces `HIL_Firmware/hil_sim` (or `hil_sim.exe` on Windows).

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
|   |-- track.c/.h            The figure-8 track defined as waypoints.
|   |-- autopilot.c/.h        The virtual driver. Steers toward the next waypoint.
|   `-- Makefile              Builds everything into one binary called hil_sim.
|
|-- ECU_Firmware/             The ECU side. Only sees sensor data, not the full car state.
|   |-- torque_vectoring.c    The TV algorithm: splits torque between the four wheels.
|   `-- torque_vectoring.h    Function prototype and tuning constants.
|
`-- ECU_Hardware/
    `-- README.md             Placeholder for future PCB and wiring documentation.
```

There are three roles in the system. They are kept in separate files on purpose:

| Role | File | What it does |
|------|------|--------------|
| Driver | `autopilot.c` | Steers toward the next waypoint and sets throttle demand |
| Car | `vehicle_model.c` | Updates the physics of the car each tick |
| ECU | `torque_vectoring.c` | Splits the throttle demand into four wheel torques |


## How it works

Each tick (100 times per second), this happens:

```
Autopilot (driver)
    sets steering angle
    returns driver torque demand (total Nm)
        |
        v
ECU Firmware (torque_vectoring_update)
    receives sensor data: yaw rate, speed, steering angle
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
    x, y, heading, speed, yaw_rate, four wheel torques, TV state, lap, time
        |
        v  (stdout pipe)
visualiser.py reads the STATE line
    draws the track, car, and data panel in the pygame window
```

The ECU code only ever receives a `SensorData` struct. It never touches the full
`VehicleState`. That is the HIL boundary. In a real system, the real ECU would
receive real sensor readings over CAN, and the HIL PC would receive real torque
commands back. The code structure already reflects this.


## The vehicle model

The car uses a single-track bicycle model. This is the simplest model that still
behaves like a real car.

It tracks: position (x, y), heading angle, speed, yaw rate, and body slip angle.

The left and right wheels on each axle are treated as one wheel in the centre
of the car. The model knows about total drive force and yaw moments from
left-right torque differences, but it does not model detailed tyre slip curves,
suspension movement, aerodynamic forces, or braking.

This is good enough for testing the torque vectoring logic. The car corners,
accelerates, and responds to different torque distributions in a physically
reasonable way.


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

**Step 2: Yaw rate error**

```
error = desired_yaw_rate - actual_yaw_rate
```

A positive error means the car is not turning enough (understeer). A negative
error means it is turning too much (oversteer).

**Step 3: Torque bias**

```
torque_bias = Kp * error
```

`Kp` is the proportional gain. It starts at 50 Nm per rad/s. You can change it
at runtime with `[` and `]`.

**Step 4: Apply the bias**

The total driver torque is split 50/50 between front and rear axles. Within
each axle, the bias shifts torque to the outer wheels:

```
outer_wheel = (total / 4) + (bias / 2)
inner_wheel = (total / 4) - (bias / 2)
```

**Step 5: Clamp**

Each wheel torque is clamped to the motor limits (0 to 200 Nm).

That is the whole algorithm. A real production TV system would also include
feedforward terms, a tyre model, a stability controller, and driver preference
maps. This version captures the essential idea in about 50 lines.


## Things to try

**Toggle TV on and off**
Press `t` while the simulation is running. Watch the torque numbers in the
status line. With TV off, all four wheels get the same torque. With TV on,
you can see the outer wheels getting more torque through corners.

**Change the gain**
Press `[` to decrease Kp or `]` to increase it. A very high gain (try 200+)
makes the TV system very aggressive. A gain of 0 is the same as TV off.

**Read the code**
The torque vectoring algorithm is about 50 lines of C with comments explaining
every step. The vehicle model is about 60 lines. Start with
`ECU_Firmware/torque_vectoring.c` then move to `HIL_Firmware/vehicle_model.c`.

**Change the track**
Open `HIL_Firmware/track.c`. Change `LOOP_RADIUS_M` to make the loops bigger
or smaller. Change `STEPS_PER_LOOP` to make the track smoother or coarser.

**Change the driver**
Open `HIL_Firmware/autopilot.c`. Change `LOOKAHEAD_M` to make the autopilot
look further ahead (smoother, cuts more corners) or closer (tighter, more
nervous). Change `TARGET_SPEED_MS` to make the car go faster or slower.


## What is not modelled

This simulation leaves out a lot of things on purpose. The goal is to be
readable, not to be a racing simulator.

Not modelled:
- Tyre slip and the Pacejka magic formula
- Weight transfer under braking and acceleration
- Aerodynamic drag and downforce
- Brake torque vectoring (only drive torque is vectored here)
- Suspension travel and body roll
- Motor and inverter dynamics
- Battery state of charge

Do not use this to predict real lap times or vehicle limits. Use it to understand
how torque vectoring control logic works, and to test that the ECU code behaves
correctly in response to sensor inputs.


## Platform notes

**Python visualiser:** Requires Python 3 and pygame. Install pygame with:
```
pip install pygame
```
pygame works on Windows, macOS, and Linux with no extra setup.

**Building hil_sim on Linux / macOS:** Works out of the box with any gcc installation.

**Building hil_sim on Windows:** Use MSYS2 with the MinGW-w64 toolchain.

Install MSYS2 from https://www.msys2.org, then in the MSYS2 shell:

```
pacman -S mingw-w64-x86_64-gcc make
```

Then open the MinGW 64-bit shell, navigate to the repo, and run `make`.
You can then run `python visualiser.py` from a normal Windows terminal (cmd,
PowerShell, or the MSYS2 shell -- all work as long as Python is on your PATH).
