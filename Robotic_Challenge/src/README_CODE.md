# Robot code v1 notes

The active Arduino-style sketch is selected by `src_dir` in
`Robotic_Challenge/platformio.ini`.

Use `platformio.ini` to switch between the main sketch and test sketches.
Most tuning values currently live near the top of the active `.ino` file or
in the matching helper headers for that sketch:

- motor speed and turn parameters
- RFID-to-planting offset distance
- Modulino Distance hole/floor thresholds
- DS-R005 300 degree servo pulse range and 60 degree seed-release step
- obstacle stop threshold
- WiFi SSID/password via a local `secrets.h`
- 11.1V battery divider resistor values
- LDR divider resistor value

Important hardware note: the DS-R005 300 degree positional servo signal goes
to GIGA D33, but servo power must come from a 5V-6V regulator. Do not power
the servo from GIGA 5V and do not connect it directly to the 11.1V battery or
Motoron VIN.
