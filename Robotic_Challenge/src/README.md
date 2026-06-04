# Source Layout

This folder contains the main robot firmware source used by PlatformIO.

## `main/`

`main/` contains the active firmware entry point and shared support code.

- `main.ino` selects the active difficulty implementation by including one difficulty `.cpp` file.
- `constants.h`, `types.h`, `globals.h`, and `declarations.h` define shared configuration, data types, global state, and cross-header declarations.
- `basic_utils.h`, `motor_utils.h`, `encoder_utils.h`, `line_following_utils.h`, `sonar_wall_utils.h`, `imu_turn_utils.h`, and `serial_utils.h` contain reusable hardware and control helpers.
- `obstacle_demo.h` is a small shared/demo helper for obstacle-related testing.

## Difficulty Folders

Difficulty-specific behavior lives in dedicated folders under `main/`.

- `easy-difficulty/` contains the easy mission controller, RFID/server handling, base exit flow, navigation helpers, route data, and planting helpers for the easy strategy. This strategy is also viable for the medium diffiulty in the competition.
- `hard-difficulty/` contains the hard mission controller, hard route logic, RFID/server handling, obstacle bypass, return-route logic, and hard-specific configuration.

The difficulty folders should reuse the shared headers for common hardware behavior whenever possible. Only behavior unique to a difficulty should live in that difficulty folder.

## Adding Or Changing Code

Prefer this structure:

- Put reusable hardware primitives or shared control behavior in `main/*.h`.
- Put mission/state-machine behavior in the relevant difficulty folder.
- Keep difficulty constants and state close to the difficulty implementation.
- Avoid duplicating motor, encoder, QTR line sensor, sonar, IMU, or serial helper code across difficulty folders.
