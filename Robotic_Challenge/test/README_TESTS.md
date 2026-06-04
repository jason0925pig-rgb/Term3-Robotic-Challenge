# Robotics Challenge Test Sketches

This folder contains standalone Arduino sketches used for component-level tests,
trial-run checks, and focused integration experiments. These files are not meant
to be the most polished software architecture in the repository. During testing
we deliberately kept many sketches as single `.ino` files so the team could copy
one folder into Arduino IDE, change a parameter quickly, upload, and iterate on
the robot without chasing header/source dependencies.

The production-style code is handled in `Robotic_Challenge/src/main`, where the
logic is split into smaller headers and modules. The `test` folder is therefore
best read as a record of calibration, debugging, and subsystem validation.

## WiFi Credentials

WiFi-related sketches require a local `secrets.h` file in the same sketch folder.
Copy the provided `secrets.example.h` to `secrets.h`, then fill in the lab WiFi
and MiniMessenger broker details.

Real `secrets.h` files contain credentials and should not be committed to GitHub.

## Current I2C Addresses

- Motoron M3S550 on `Wire1`: `0x11`
- RFID2 on `Wire`: `0x28`
- Modulino Distance on `Wire`: `0x29`
- Modulino Pixels on `Wire`: `0x36`
- ICM20948 IMU on `Wire`: `0x68`

## Shared Wiring Assumptions

- Front sonar: trig `D8`, echo `D11` through a 5 V to 3.3 V level shifter.
- Left sonar: trig `D9`, echo `D12` through a level shifter.
- Right sonar: trig `D10`, echo `D13` through a level shifter.
- QTR-HD-09RC line sensor: CTRL odd/even `D2/D3`, sensor pins `D22-D30`.
- Revival button: `D31` to GND, configured as `INPUT_PULLUP`.
- Start/stop or kill button: `D32` to GND, configured as `INPUT_PULLUP`.
- Seed servo signal: `D33`.
- Left encoder C1/C2: `D34/D35`.
- Right encoder C1/C2: `D36/D37`.
- Motoron M1/M2: left/right track motors.

## QTR Calibration

The replacement QTR sensor currently uses these saved raw calibration arrays:

```cpp
constexpr uint16_t kSavedQtrMin[9] = {98, 82, 82, 82, 86, 94, 105, 121, 142};
constexpr uint16_t kSavedQtrMax[9] = {1479, 921, 866, 797, 762, 733, 779, 901, 1177};
```

These values come from `QTR_Raw_Read_Test/output.txt`. The line-following code
maps raw readings into `0..1000` using these arrays, then compares each channel
with `kLineThreshold`. If the sensor board, mounting height, or floor tape
changes, run `QTR_Raw_Read_Test` again and update the saved arrays in the
affected test sketches.

## Active Test Sketches

- `Base_Exit_Tunnel_Wall_Test`: focused base-to-arena integration test. It
  registers over MiniMessenger, follows the base route, requests airlock A from
  the RFID tag, switches from line following to tunnel wall following, and stops
  when the arena grid line is detected.
- `I2C_Scanner_Test`: scans both `Wire` and `Wire1` to verify device addresses.
- `Line_Planting_IMU`: combined line-following, RFID-triggered planting, and
  IMU turn experiment.
- `Motoron_Address_Test`: Motoron address and channel diagnostic.
- `Obstacle_Avoidance_Test`: modular obstacle-avoidance experiments with
  supporting headers for line-based and wall-based behaviours.
- `QTR_Raw_Read_Test`: raw QTR timing and min/max calibration output.
- `Robot_Revival`: rescue/revival behaviour test using front sonar, D31 revival
  button, Motoron, and Modulino Pixels.
- `Servo_60deg_Test`: DS-R005 300-degree positional servo stepping test.
- `Sonar_Distance_Test`: front/left/right HC-SR04 distance readout.
- `Trial_Run_1`: integrated checklist sketch for the first trial run.
- `Wall_Following`: sonar wall-following, RFID corner counting, and IMU turning.
- `WiFi_Kill_Forward_Test`: MiniMessenger enable/disable safety test with simple
  forward motion.

## Debug Data

- `Debug output/easy_strategy_debug.txt`: captured serial output for the easy
  strategy.
- `QTR_Raw_Read_Test/output.txt`: latest replacement QTR raw calibration output
  and explanation.
