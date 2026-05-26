# Robotics Challenge Test Sketches

This folder contains standalone Arduino sketches for component-level and integration testing.

## WiFi Credentials

WiFi-related sketches require a local `secrets.h` file in the same sketch folder. Copy the provided `secrets.example.h` to `secrets.h`, then replace the placeholder password with the lab WiFi password.

The real `secrets.h` files are intentionally ignored by git and should not be pushed to GitHub.

## Current I2C Addresses

- Motoron M3S550 on `Wire1`: `0x11`
- RFID2 on `Wire`: `0x28`
- Modulino Distance on `Wire`: `0x29`
- Modulino Pixels on `Wire`: `0x36`
- ICM20948 IMU on `Wire`: `0x68`

## Main Test Sketches

- `I2C_Scanner_Test`: scans both `Wire` and `Wire1`.
- `LineFollow_Test`: line sensor and Motoron line-following test.
- `Line_Planting_IMU`: combined line-following, RFID planting, and IMU turning test.
- `Motor_Encoder_U_Test`: motor encoder movement and WiFi kill switch integration.
- `Motor_IMU`: IMU-assisted turning test.
- `planting`: RFID-triggered forward offset and seed release test.
- `QTR_Raw_Read_Test`: raw reflectance sensor calibration readings.
- `Revival_LED_Test`: revival button and Modulino Pixels test.
- `Servo_60deg_Test`: DS-R005 300-degree servo step test.
- `Sonar_Distance_Test`: front/left/right HC-SR04 distance test.
- `Trial_Run_1`: integrated trial-run checklist sketch.
- `Wall_Following`: sonar wall following, RFID corner counting, and IMU turn test.
- `WiFi_Kill_Forward_Test`: continuous forward motion with MiniMessenger enable/disable safety.
- `WiFi_Only_Test`: WiFi-only connection diagnostic.
