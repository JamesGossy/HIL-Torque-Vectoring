# ECU Hardware

This folder is a placeholder for hardware documentation.

In the real system, this would contain:

- Schematics for the ECU PCB
- Pin mapping between the ECU and the four motor controllers
- CAN bus configuration (baud rate, message IDs, timing)
- SPI or UART wiring for the IMU and steering angle sensor
- Power supply requirements
- Notes on which microcontroller the ECU code runs on

For now, the ECU logic lives in `ECU_Firmware/torque_vectoring.c` and runs on
the host PC as part of the HIL simulation. When you are ready to move to real
hardware, that file is the one you port to the embedded target.
