# UCL Year 1 Robotics Challenge 2026 — Robot System Documentation

## Project Overview

### Competition Theme
"UCL RAI Terraforming Initiative" — 14 autonomous robots are sent to another planet to prepare it for human habitation. Robots must explore the surface, plant seeds at fertile locations, respond to emergency warnings, and rescue stranded teammates.

### Arena Layout
- **Underground Base** (1.2 × 2.4m): Has WiFi, floor markings, power supply. Contains robot deployment area ("Start") and collection area ("Parking"). Two RFID tags (A and B) control airlock doors.
- **Main Arena / Surface** (2.5 × 2.5m): 9×9 grid of RFID tags buried under holes. Left half has solid lines for line-following; right half has only holes (no lines). Robots must navigate autonomously on the right half.
- **Two Ramped Tunnels**: Tunnel A (entrance into base), Tunnel B (exit from base). Each tunnel has two sets of doors (airlock). Ramp angle <20°, tunnel length ~1.2m. Up to 5 robots can fit in a tunnel simultaneously.

### Missions
- **Main**: Area exploration, seed planting at fertile RFID locations, emergency return to base (solar flare warning)
- **Bonus**: Detect brightest lit areas (for solar panels), rescue stranded robots

### Key Rules
- One robot per team, max dimensions 20×20×15 cm
- Each robot carries max 5 seeds (½ inch diameter spheres, dropped into 25mm diameter holes)
- Limited arena time (~5 min per sortie), charging timeout (~5 min) after return
- Two attempts max per round
- Soil fertility is randomly assigned; must query server via WiFi after reading RFID tag
- Emergency warning: all robots must return within ~1 minute
- Stranded robots can be revived by another robot pressing their front button

### Airlock Protocols
**Exiting (Tunnel B)**: Robot drives to RFID tag B → sends ID to server → receives clearance → follows line to door → door opens automatically → enters tunnel → base door closes → surface door opens.

**Entering (Tunnel A)**: Robot sends "Let me in" via WiFi → another robot INSIDE base drives over RFID tag A → surface door opens → robot enters → sensor detects entry → door closes after 5 seconds → base door opens.

**Critical implication**: At least one robot must remain inside the base to open the door for returning robots.

### Scoring
1. Most individual points (seeds planted)
2. Most revivals (other robots saved)
3. Most efficient robot (points / BOM cost ratio)
4. People's choice award
5. Sustainability award

---

## Hardware Components

### 1. Main Controller
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | Microcontroller | **Arduino GIGA R1 WiFi** | Robot brain. Dual-core STM32H747 (480/240 MHz). Runs all logic, sensor reading, motor control. Built-in WiFi for server communication and remote kill switch. **3.3V logic level — all 5V signals must be level-shifted.** |

### 2. Drive System
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | Motor Driver | **Pololu Motoron M3S550** | Triple motor controller shield. Verified working I2C address **0x11** on **Wire1** / shield SDA1-SCL1. Converts speed commands to motor power. Requires external motor power on the Motoron **VIN/GND** terminal pair. |
| 2 | DC Motor | **Waveshare DCGM-N20-12V-EN-200RPM** | Four-wheel differential drive (2 motors, each driving 2 wheels). N20 DC gear motor with magnetic Hall AB encoder and L-shaped 6-pin connector. Rated 12V, no-load speed 200 rpm, gear reduction ratio 1:150, encoder basic pulses 7 PPR and 1050 PPR after gearbox. Signals used by the robot are Motor M1/M2, Encoder VCC/GND, and Encoder C1/C2. |
| 1 | Servo | **DS-R005 300 degree positional servo** | Controls seed dispensing mechanism. Resets to 0 degrees, then advances 60 degrees per seed-release step up to 300 degrees before resetting. PWM controlled, external 5V-6V power. |

### 3. Identification & Positioning
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | RFID Reader | **M5Stack RFID2 (WS1850S)** | Reads RFID tags embedded in arena floor to determine robot position (coordinates like A1, D5, I9). Also reads base RFID tags A/B for airlock control. I2C address **0x28** on **Wire** (Pin 20/21). |
| 1 | ToF Sensor | **SparkFun VL53L5CX** | Laser Time-of-Flight 8×8 multi-zone ranging. Mounted under the robot chassis pointing downward. Used for wider hole/ground detection and seed drop verification. I2C address **0x29** on **Wire** (Qwiic connector). |
| 1 | Local ToF Sensor | **Arduino Modulino Distance (VL53L4CD, ABX00102)** | Mounted directly above the seed-dispensing chute/hole in the planting subsystem, pointing down through the opening. Measures the local distance profile to calibrate whether the planter is centered over the arena hole before releasing a seed. Default I2C address is **0x29** in 7-bit Arduino notation; the board silk **0x52** is the 8-bit form. |

### 4. Navigation Sensors
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | Line Sensor | **Pololu QTR-HD-09RC Reflectance Array** | 9 IR LED + phototransistor pairs. Detects black lines on white surface for PID line-following. Must be mounted 1-3mm above ground. Uses RC timing mode on digital pins. Two CTRL pins control odd/even LED groups. If the physical part is QTR-HD-09A instead, rewire to analog inputs or an external ADC. |
| 3 | Ultrasonic | **HC-SR04** | Obstacle avoidance. Range 2-400cm, 15° cone. Mounted front, left, right. **Echo pin outputs 5V — MUST use logic level converter.** Trig can connect directly (3.3V is sufficient). |

### 5. Attitude & Environmental Sensing
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | IMU | **Pimoroni ICM20948** | 9-axis: accelerometer + gyroscope + magnetometer. Tracks robot heading/orientation for turns, diagonal movement, and dead reckoning on the unmarked (right) half of the arena. I2C address **0x68** on **Wire**. 2-5V tolerant. |
| 1 | Light Sensor | **LDR (Photoresistor) + Voltage Divider Board** | Detects surface light intensity for the bonus mission: finding the brightest areas to report as solar panel locations. Analog output to **A0**. Powered by 3.3V. |

### 6. Control Switches & Indicators
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | Mechanical Kill Switch | **Cherry MX Mechanical Switch** | Top-mounted low-current kill input. It does **not** physically cut robot power; when pressed, GIGA reads the switch and sets the same software `killFlag` used by WiFi kill, so the robot stops actuators and does not run the main mission loop. |
| 1 | Revival Button | **Cherry MX Mechanical Switch** | Mounted on robot front. When another robot pushes this button, the LED changes from red to green, signaling a successful rescue. Connected to digital pin for state detection. |
| 1 | LED Module | **Arduino Modulino Pixels** | RGB LED module for revival indication. Default red (stranded/normal), turns green when revival button is pressed. I2C address **0x6C** on **Wire**. |
| 1 | WiFi Kill Switch | **Software (via GIGA WiFi)** | Remote kill command from the server. It sets the same software `killFlag` as the mechanical kill switch; robot remains powered but stays in a safe idle state instead of entering/running the main mission loop. |

### 7. Protection Modules
| Qty | Component | Model | Purpose |
|-----|-----------|-------|---------|
| 1 | Level Shifter | **SparkFun Logic Level Converter (4-channel)** | Bidirectional 5V↔3.3V conversion. Used for HC-SR04 Echo signals (5V→3.3V). Has 4 channels: 3 used for 3× HC-SR04 Echo lines, 1 spare. |

---

## Complete Wiring List

### Controller and Common Ground
| Component | Connection |
|-----------|------------|
| Arduino GIGA R1 WiFi (microcontroller) | Main controller. All GPIO signals are 3.3V logic. |
| Common GND (shared ground reference) | Connect GIGA GND, Motoron GND, motor supply negative, servo regulator negative, 5V sensor rail GND, and all sensor GND pins together. |

### I2C Bus `Wire` - GIGA D20 SDA / D21 SCL
| Component | GIGA / Power Connection | I2C Address | Notes |
|-----------|-------------------------|-------------|-------|
| M5Stack RFID2 WS1850S (RFID reader) | SDA -> D20, SCL -> D21, VCC -> 3.3V, GND -> common GND | `0x28` | Reads arena/base RFID tags. |
| SparkFun VL53L5CX (8x8 ToF distance sensor) | SDA -> D20, SCL -> D21, 3V3 -> 3.3V, GND -> common GND | `0x29` | Wider downward hole/ground sensing. |
| Arduino Modulino Distance VL53L4CD ABX00102 (single-zone ToF distance sensor) | SDA -> D20, SCL -> D21, 3V3 -> 3.3V, GND -> common GND | default `0x29`; board silk `0x52` is 8-bit form | Mounted above the seed chute/hole for local planting alignment. |
| Pimoroni ICM20948 (9-axis IMU) | SDA -> D20, SCL -> D21, VCC -> 3.3V, GND -> common GND | `0x68` or `0x69` | Heading/orientation sensing. Confirm actual address with I2C scanner. |
| Arduino Modulino Pixels ABX00109 (RGB LED module) | SDA -> D20, SCL -> D21, 3V3 -> 3.3V, GND -> common GND | `0x6C` | Revival/status LED. |

**ToF address note**: SparkFun VL53L5CX and Arduino Modulino Distance both default to 7-bit `0x29`. Use only one ToF at that address, move one to another bus/multiplexer, or use `XSHUT`/address sequencing before running both together.

### I2C Bus `Wire1` - Motoron Shield SDA1 / SCL1
| Component | Connection | I2C Address | Notes |
|-----------|------------|-------------|-------|
| Pololu Motoron M3S550 (triple motor controller shield) | Plugs into the shield I2C bus; motor power goes to Motoron `VIN/GND` terminal pair | `0x11` verified by motor test | In code, use `Wire1.begin()`, `mc.setBus(&Wire1)`, and address `0x11`. The scanner result at `0x60` is not the address that made the Motoron library drive the motors. |

### Digital Inputs and Outputs
| Component | Component Pin / Wire | GIGA Pin | Notes |
|-----------|----------------------|----------|-------|
| Pololu QTR-HD-09RC (line sensor array) | CTRL odd | D2 | Controls odd-numbered emitters 1/3/5/7/9. |
| Pololu QTR-HD-09RC (line sensor array) | CTRL even | D3 | Controls even-numbered emitters 2/4/6/8. |
| HC-SR04 Front (ultrasonic distance sensor) | Trig | D8 | Direct 3.3V trigger output. |
| HC-SR04 Front (ultrasonic distance sensor) | Echo | D11 through level shifter LV1/HV1 | Echo is 5V, must be level-shifted. |
| HC-SR04 Left (ultrasonic distance sensor) | Trig | D9 | Direct 3.3V trigger output. |
| HC-SR04 Left (ultrasonic distance sensor) | Echo | D12 through level shifter LV2/HV2 | Echo is 5V, must be level-shifted. |
| HC-SR04 Right (ultrasonic distance sensor) | Trig | D10 | Direct 3.3V trigger output. |
| HC-SR04 Right (ultrasonic distance sensor) | Echo | D13 through level shifter LV3/HV3 | Echo is 5V, must be level-shifted. |
| Cherry MX (revival button input) | One switch terminal -> D31, other terminal -> GND | D31 | `INPUT_PULLUP`, LOW when pressed by another robot. |
| Cherry MX (mechanical kill switch input) | One switch terminal -> D32, other terminal -> GND | D32 | `INPUT_PULLUP`, LOW when pressed; sets software `killFlag`. This does not cut power. |
| DS-R005 300 degree positional servo (seed-release servo) | Signal wire | D33 | PWM control for seed mechanism. 0-300 degree range mapped to about 500-2500us; software steps 60 degrees each release. |
| Pololu QTR-HD-09RC (line sensor array) | OUT1-OUT9 | D22-D30 | Manual RC timing read. If actual part is QTR-HD-09A, use analog inputs or an external ADC instead. |
| Left Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Encoder C1 | D34 | Quadrature encoder channel 1. |
| Left Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Encoder C2 | D35 | Quadrature encoder channel 2. |
| Right Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Encoder C1 | D36 | Quadrature encoder channel 1. |
| Right Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Encoder C2 | D37 | Quadrature encoder channel 2. |
| Arduino Modulino Distance VL53L4CD ABX00102 (single-zone ToF distance sensor) | XSHUT | D38 optional | Use only if shutdown/address sequencing is needed for the `0x29` ToF conflict. GPIO1 can be left unconnected unless interrupt mode is used. |

### Analog Input
| Component | Connection | GIGA Pin | Notes |
|-----------|------------|----------|-------|
| GL5528 LDR with voltage divider (photoresistor light sensor) | Divider midpoint / signal | A0 | Power divider from 3.3V and GND so A0 never exceeds 3.3V. |

### Motoron Motor Power and Motor Outputs
| Component | Wire / Terminal | Connection |
|-----------|-----------------|------------|
| Pololu Motoron M3S550 (triple motor controller shield) | `VIN` | External motor supply positive, 11.1V battery / 12V nominal motor rail for the selected Waveshare motors. |
| Pololu Motoron M3S550 (triple motor controller shield) | `GND` | External motor supply negative and common robot GND. |
| Left Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Motor M1 | Motoron `M1A`. |
| Left Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Motor M2 | Motoron `M1B`. |
| Right Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Motor M1 | Motoron `M2A`. |
| Right Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Motor M2 | Motoron `M2B`. |

### Motor Encoder Power
| Component | Wire | Connection |
|-----------|------|------------|
| Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Encoder VCC | GIGA 3.3V. |
| Waveshare DCGM-N20-12V-EN-200RPM (N20 DC gearmotor with AB Hall encoder) | Encoder GND | Common GND. |

### Logic Level Converter and Ultrasonic Echo
| Component | Pin | Connection |
|-----------|-----|------------|
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | LV | GIGA 3.3V. |
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | LV GND | GIGA/common GND. |
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | HV | 5V sensor rail used by HC-SR04. |
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | HV GND | Common GND. |
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | HV1 / LV1 | HC-SR04 Front Echo -> HV1, LV1 -> GIGA D11. |
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | HV2 / LV2 | HC-SR04 Left Echo -> HV2, LV2 -> GIGA D12. |
| SparkFun BOB-12009 (4-channel bidirectional logic level converter) | HV3 / LV3 | HC-SR04 Right Echo -> HV3, LV3 -> GIGA D13. |

### Servo Power
| Component | Wire | Connection |
|-----------|------|------------|
| DS-R005 300 degree positional servo (seed-release servo) | VCC | External 5V-6V servo regulator positive. |
| DS-R005 300 degree positional servo (seed-release servo) | GND | Servo regulator negative and common GND. |
| DS-R005 300 degree positional servo (seed-release servo) | Signal | GIGA D33. |

### Kill Switch Logic
| Component | Connection | Behavior |
|-----------|------------|----------|
| Cherry MX (mechanical kill switch input) | One side to GIGA D32, other side to GND | Pressed = LOW. Software sets `killFlag = true`. Robot remains powered, stops actuators, disables seed dispensing, and stays out of the main mission loop. |
| GIGA WiFi command (WiFi kill switch input) | Server command received by Arduino GIGA R1 WiFi | Sets the same `killFlag = true` as the mechanical kill switch. |

### Power Summary
| Rail | Components |
|------|------------|
| 3.3V | M5Stack RFID2 WS1850S (RFID reader), SparkFun VL53L5CX (8x8 ToF distance sensor), Arduino Modulino Distance VL53L4CD ABX00102 (single-zone ToF distance sensor), Pimoroni ICM20948 (9-axis IMU), Arduino Modulino Pixels ABX00109 (RGB LED module), Pololu QTR-HD-09RC (line sensor array), GL5528 LDR divider (photoresistor light sensor), SparkFun BOB-12009 LV side (logic level converter), motor encoder VCC |
| 5V | HC-SR04 x3 (ultrasonic distance sensors), SparkFun BOB-12009 HV side (logic level converter) |
| External 5V-6V | DS-R005 300 degree positional servo |
| External 11.1V / 12V nominal motor rail | Pololu Motoron M3S550 VIN/GND motor rail and Waveshare DCGM-N20-12V-EN-200RPM motor power |

All power rails must share a common GND with the Arduino GIGA.

---

## Software Libraries

### Required Libraries (Install via Arduino Library Manager)
| Library | Author | Purpose | Search Term |
|---------|--------|---------|-------------|
| Motoron | Pololu | Motor driver control | `Motoron` |
| MFRC522_I2C | arozcan | RFID tag reading | `MFRC522_I2C` |
| SparkFun VL53L5CX | SparkFun | ToF multi-zone distance | `SparkFun VL53L5CX` |
| Adafruit ICM20X | Adafruit | IMU attitude sensing for ICM20948 | `Adafruit ICM20X` |
| Modulino | Arduino | Modulino Pixels LED control and Modulino Distance readings | `Modulino` |

**Note**: When installing `Adafruit ICM20X`, also install its Arduino Library Manager dependencies if prompted, especially `Adafruit BusIO` and `Adafruit Unified Sensor`.

### Built-in Libraries (No installation needed)
| Library | Purpose |
|---------|---------|
| Wire | I2C communication |
| Servo | Servo motor PWM control |
| WiFi | Server communication, remote kill switch |

### Libraries NOT Used (and why)
| Library | Reason |
|---------|--------|
| QTRSensors (Pololu) | Causes hardfault on GIGA R1. Use manual RC read implementation instead (see code notes). |

---

## Known GIGA R1 WiFi Quirks

### Voltage
- **3.3V logic only.** Any 5V signal input will damage GPIO pins. Always use level shifting for 5V devices.

### Analog Pins
- **A0–A7**: Work normally with `analogRead()`. A6 maps to digital pin 83, A7 to pin 84.
- **A8–A11**: "PureAnalogPin" type. Cannot be cast to `uint8_t`, cannot be used with standard `analogRead()` in certain contexts. Avoid these pins.
- Always call `analogReadResolution(12)` for 12-bit ADC (0–4095). Default is 10-bit.

### I2C Buses
- **Wire**: Default I2C on Pin 20 (SDA) / Pin 21 (SCL). Use for RFID, ToF, IMU, and Modulino modules.
- **Wire1**: Secondary I2C shield bus. Use for the Motoron shield with `mc.setBus(&Wire1)`.
- **ToF address conflict**: VL53L5CX and Modulino Distance both default to 7-bit `0x29`; the Modulino board label `0x52` is the 8-bit address form. Resolve this before running both sensors together.

### Upload Issues
- DFU suffix warning ("Invalid DFU suffix signature") is normal and harmless.
- If upload fails with "No DFU capable USB device available", double-tap the RST button to enter bootloader mode, then upload within 3 seconds.

### QTR Sensor Compatibility
- The QTRSensors Pololu library triggers hardfaults when receiving serial input on GIGA. Use manual RC-mode implementation: charge pin HIGH, switch to INPUT, measure discharge time. Works reliably on all digital pins.

---

## Design Decisions & Strategy Notes

### Drivetrain
Four-wheel differential drive with two Waveshare DCGM-N20-12V-EN-200RPM motors (each motor drives two wheels on one side). Provides good traction and allows zero-radius turning by spinning wheels in opposite directions. The 1:150 gear ratio was chosen for high torque and precise low-speed navigation.

### Seed Dispensing Mechanism
DS-R005 300 degree positional servo controls the gate/slider and releases one seed at a time from a hopper holding 5 seeds. The mechanism resets to 0 degrees, then advances 60 degrees per release until 300 degrees; after that it resets to 0 degrees and waits before continuing. The Modulino Distance sensor is mounted directly above the seed-dispensing chute/hole, looking down through the same opening used by the seed. Before release, software compares the measured distance against calibrated "flat floor" and "open arena hole" readings; the planter only drops a seed when the reading indicates the chute is centered over the hole. The VL53L5CX can still be used as a wider downward sensor under the chassis, while the Modulino Distance provides the local planter alignment check.

### Navigation Strategy
- **Left half of arena (with lines)**: PID line-following using QTR-HD-09RC array
- **Right half of arena (no lines)**: Dead reckoning using IMU (ICM20948) heading + encoder odometry
- **RFID checkpoints**: Position correction whenever an RFID tag is read
- **Obstacle avoidance**: Three HC-SR04 sensors (front/left/right) provide 360° forward awareness

### Airlock Coordination
Robots communicate via WiFi through the arena server. When a robot outside needs to enter, it sends a "Let me in" message. A robot stationed inside base drives over RFID tag A to open the door. Strategy: always keep at least one robot inside as a "doorman."

### Emergency Response
On receiving WiFi emergency broadcast, all robots immediately navigate to nearest tunnel entrance. Formation movement ("tailgating") allows multiple robots to enter the tunnel simultaneously (fits up to 5). Robots use distance sensors to maintain safe following distance.

### Revival Mechanism
Front-mounted Cherry MX button. When pressed by another robot's chassis, Modulino Pixels LED changes from red to green. Button press detected on Pin 31, triggering LED color change and WiFi notification to server.

### Light Detection (Bonus Mission)
LDR photoresistor on analog pin A0 measures ambient light. Robot maps light levels to RFID grid coordinates and reports brightest locations to server as recommended solar panel sites.

---

## Schedule

| Week | Key Milestones |
|------|---------------|
| 1 (Apr 27–May 3) | Kick-off. CAD design, component testing, team rep selection |
| 2 (May 4–10) | Assembly, integration testing, basic line following |
| 3 (May 11–17) | **Trial runs #1 (May 15)**, **Interim report due May 15 4PM** |
| 4 (May 18–24) | Refinement, PID tuning, WiFi integration |
| 5 (May 25–31) | Full system testing, strategy optimization |
| 6 (Jun 1–7) | **Trial runs #2 (Jun 2)**, **Finals Day (Jun 4)**, **Final report due Jun 5 4PM** |

---

## Robot Requirements Checklist

- [ ] Two code-level kill inputs: mechanical top button + WiFi command
- [ ] Line-following sensors (QTR-HD-09RC, or analog wiring if using QTR-HD-09A)
- [ ] Distance/wall sensors (HC-SR04 ×3)
- [ ] Battery powered (for finals)
- [ ] Soldered boards and PCBs (no breadboards in finals)
- [ ] Tidy wiring and cable management
- [ ] Seed hopper (5 seeds max) with dispensing mechanism
- [ ] Front revival button with LED (red→green)
- [ ] Max dimensions: 20 × 20 × 15 cm
