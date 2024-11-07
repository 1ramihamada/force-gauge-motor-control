# Controlling Motor with Force Gauge Input

This repository contains a Python script that interfaces a Dynamixel motor with a Mark-10 / OMEGA DFG55 force gauge to control the motor's movement based on force readings. The motor adjusts its position in response to force applied to the gauge: moving in one direction when the force exceeds a positive threshold and in the opposite direction when the force exceeds a negative threshold. This setup is ideal for applications requiring real-time feedback-driven motor control, such as automated testing rigs, dynamic load applications, or experimental robotics.

## How It Works

- The **Mark-10 / OMEGA DFG55 force gauge** continuously reads the applied force and communicates this data to the Python script via a serial USB connection.
- The **Dynamixel motor** receives movement instructions based on the force data. When the force exceeds a threshold, the motor moves in the appropriate direction. If the force is within the specified dead zone (between -2 and 2 N), the motor immediately stops.
- The program includes a dead zone to prevent minor fluctuations from causing unnecessary motor movements and a smoothing mechanism to reduce noise in the force readings.

## Usage Instructions

- Connect the Mark-10 / OMEGA force gauge and the Dynamixel motor to the computer. Note the USB ports used by each device `(e.g., /dev/ttyUSB0 and /dev/ttyUSB1 on Linux)`.
-  Set the correct port for the force gauge (FORCE_PORT) and the Dynamixel motor (DXL_PORT).
-  Verify that the DXL_ID matches the ID set on your Dynamixel motor.
-  Verify the `baud_rate` setting matches the gauge’s configuration (default is 115200 for Mark-10 Series 5).
- run with `python3 force_control.py`

## Requirements

### Hardware
- **Mark-10 / OMEGA DFG55 Force Gauge**: capable of outputting data via serial/USB.
- **Dynamixel Motor**: compatible with the Dynamixel SDK, along with a power supply.
- **Computer** with USB connection to run the script.

### Python Libraries
- `pyserial`: for serial communication with the force gauge.
- `dynamixel_sdk`: for interfacing with the Dynamixel motor.

Install the required libraries with:
```bash
- pip install pyserial
- pip install dynamixel_sdk
