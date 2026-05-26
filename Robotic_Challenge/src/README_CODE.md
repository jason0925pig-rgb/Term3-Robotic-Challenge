# Robot code v1 notes

`main.ino` is the Arduino-style entry point for this first version.
The current PlatformIO project will compile every `.cpp` file in `src/`
plus this `.ino` sketch.

Edit `Config.h` first for the values that will be tuned most often:

- motor speed and turn parameters
- RFID-to-planting offset distance
- Modulino Distance hole/floor thresholds
- DS-R005 300 degree servo pulse range and 60 degree seed-release step
- obstacle stop threshold
- WiFi SSID/password
- 11.1V battery divider resistor values
- LDR divider resistor value

Important hardware note: the DS-R005 300 degree positional servo signal goes
to GIGA D33, but servo power must come from a 5V-6V regulator. Do not power
the servo from GIGA 5V and do not connect it directly to the 11.1V battery or
Motoron VIN.
