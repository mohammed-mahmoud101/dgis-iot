# S7S Quadruped Robot — IoT Subsystem

IoT control stack for a 12-servo quadruped robot built around an **ESP32**, **PCA9685** servo driver, and **MQTT**-based communication between a laptop control station, a phone gyro bridge, and a Bluetooth gamepad.

> This repository covers the **embedded firmware**, **desktop control station**, **standalone Arduino test sketches**, and **mechanical CAD files** for the robot chassis.

<p align="center">
  <img src="images/3D%20Model%20Render.jpg" alt="3D CAD Render" width="48%"/>
  &nbsp;&nbsp;
  <img src="images/Real%20Image.jpg" alt="Assembled Robot" width="48%"/>
</p>
<p align="center">
  <em>Left: FreeCAD 3D render &nbsp;|&nbsp; Right: Assembled quadruped with ESP32, PCA9685, and 12 servos</em>
</p>

---

## System Architecture

```
┌─────────────────┐         Bluetooth (Bluepad32)
│  Xbox / Gamepad  │ ──────────────────────────────────┐
└─────────────────┘                                    │
                                                       ▼
┌─────────────────┐   clicks UI    ┌──────────────┐   ┌──────────────────────────┐
│   Laptop User   │ ─────────────► │  Tkinter App │   │  ESP32 + PCA9685         │
└─────────────────┘                │  (Python)    │   │  + 12× SG90/MG90S Servos │
                                   └──────┬───────┘   └────────────▲─────────────┘
                                          │                        │
                                          ▼                        │ USB Serial
                                   ┌──────────────┐               │ (JSON lines)
                                   │  MQTT Broker │               │
                                   │  (embedded   │               │
                                   │   or Mosquitto)│              │
                                   └──────┬───────┘               │
                                          │  WiFi                  │
                                          ▼                        │
                                   ┌──────────────┐               │
                                   │  Phone App   │ ──────────────┘
                                   │  (USB↔MQTT   │
                                   │   bridge)    │
                                   └──────────────┘
```

**Priority arbitration** lives on the ESP32: laptop MQTT commands override the gamepad for ~2 seconds after the last UI command.

---

## Directory Layout

```
dgis-iot/
├── esp32-firmware/           # PlatformIO project — main ESP32 firmware
│   ├── src/main.cpp          #   Bluepad32 gamepad → MQTT + USB serial bridge
│   ├── platformio.ini        #   Board: esp32doit-devkit-v1, Bluepad32 framework
│   ├── include/
│   ├── lib/
│   └── test/
│
├── control-station/          # Python desktop app (Tkinter)
│   ├── s7s_control_station.py#   Calibration, gait control, telemetry, logging
│   └── requirements.txt     #   paho-mqtt, amqtt, matplotlib
│
├── arduino-sketches/         # Standalone Arduino IDE sketches (dev/test)
│   ├── single-leg-ik-demo/   #   Inverse kinematics + Bézier curve single-leg demo
│   │   └── single-leg-ik-demo.ino
│   └── trot-gait-forward/    #   Full 4-leg trot gait (diagonal-pair walking)
│       └── trot-gait-forward.ino
│
├── cad/                      # FreeCAD mechanical design files
│   ├── *.FCStd               #   Assembly & part models (v1.2, v1.3)
│   ├── *.FCBak               #   FreeCAD auto-backups
│   ├── dxfs/                 #   DXF exports for laser cutting
│   ├── nesting/              #   Nesting layout ZIPs for sheet material
│   └── parts_weights.txt    #   Measured component weights
│
└── README.md                 # ← You are here
```

---

## Hardware

| Component | Description |
|---|---|
| **ESP32 DevKit V1** | Main MCU — runs WiFi, MQTT client, Bluepad32 BT stack |
| **PCA9685** | 16-channel I²C PWM driver (address `0x40`) |
| **12× Servos** | 4 legs × 3 joints (abductor / flex / knee) |
| **Gamepad** | Any Bluepad32-compatible controller (Xbox, PS4, etc.) |
| **Phone** | Runs a Kotlin app that bridges USB serial ↔ MQTT |
| **Laptop** | Runs the Python control station (Tkinter GUI) |

### Leg Geometry (from IK sketch)

| Segment | Length |
|---|---|
| Coxa (hip) | 85 mm |
| Femur (upper leg) | 104 mm |
| Tibia (lower leg) | 154 mm |

---

## Getting Started

### 1. ESP32 Firmware (`esp32-firmware/`)

**Prerequisites:** [PlatformIO CLI](https://platformio.org/install/cli) or the PlatformIO VS Code extension.

```bash
cd esp32-firmware

# Edit src/main.cpp — set your WiFi credentials and MQTT broker IP:
#   const char* ssid        = "YOUR_WIFI_SSID";
#   const char* password    = "YOUR_WIFI_PASSWORD";
#   const char* mqtt_server = "YOUR_MQTT_BROKER_IP";

# Build and upload
pio run -t upload

# Monitor serial output
pio device monitor -b 115200
```

The firmware:
- Connects to WiFi and an MQTT broker
- Accepts Bluetooth gamepads via Bluepad32
- Maps D-pad and buttons to MQTT messages (`quadpod/gamepad`)
- Forwards USB serial data from the phone bridge to MQTT

### 2. Control Station (`control-station/`)

**Prerequisites:** Python 3.8+

```bash
cd control-station
pip install -r requirements.txt

# Run with embedded MQTT broker (zero config)
python s7s_control_station.py

# Or use an external Mosquitto broker
python s7s_control_station.py --external-mqtt 192.168.1.10:1883
```

The GUI has four tabs:

| Tab | Purpose |
|---|---|
| **Calibration** | Motor offset sliders (±1.5 rad) + gyro zero-point capture |
| **Controls** | Gait buttons (trot, turn, arc), speed slider, E-STOP |
| **Telemetry** | Live 12-joint angle readout + matplotlib bar plot |
| **Log** | Scrolling log viewer with color-coded levels |

### 3. Arduino Sketches (`arduino-sketches/`)

Open in Arduino IDE (requires `Wire.h` and `Adafruit_PWMServoDriver` library):

- **`single-leg-ik-demo/`** — Tests a single leg using inverse kinematics and cubic Bézier trajectory. Useful for verifying servo wiring, PCA9685 communication, and tuning leg geometry constants.

- **`trot-gait-forward/`** — Full 4-leg trot gait using diagonal pairs (FL+BR, FR+BL). Hardcoded angles with smooth interpolation. Useful for validating mechanical assembly before the full firmware.

### 4. CAD Files (`cad/`)

Open in [FreeCAD](https://www.freecad.org/) (v0.21+). Key assemblies:

| File | Description |
|---|---|
| `quadruped_robot_1.3.FCStd` | Full robot assembly (latest) |
| `assembly_1.3.1.FCStd` | Body frame assembly |
| `leg_assembly.FCStd` | Single leg assembly |
| `first_arm_assembly.FCStd` | Shoulder/coxa arm |
| `side_arm_assembly.FCStd` | Side arm linkage |
| `assembly__battery.FCStd` | Battery compartment |
| `mechanism.FCStd` | Joint mechanism study |

The `dxfs/` folder contains 23 DXF part outlines ready for laser cutting. The `nesting/` folder has sheet nesting ZIPs.

---

## MQTT Topic Reference

### Commands (Laptop → ESP32)

| Topic | Payload | Description |
|---|---|---|
| `quadpod/cmd/gait` | gait name or `"stand"` | Set active gait |
| `quadpod/cmd/speed` | float `0.1–3.0` | Gait speed multiplier |
| `quadpod/cmd/estop` | `"1"` | Emergency stop |
| `quadpod/cmd/calib/motor/start` | _(empty)_ | Enter motor calibration mode |
| `quadpod/cmd/calib/motor/set` | `{"joint":int,"offset":float}` | Set joint offset |
| `quadpod/cmd/calib/motor/save` | _(empty)_ | Save offsets to NVS |
| `quadpod/cmd/calib/motor/load` | _(empty)_ | Load offsets from NVS |
| `quadpod/cmd/calib/motor/end` | _(empty)_ | Exit calibration mode |
| `quadpod/cmd/calib/gyro/set` | `{"yaw":f,"pitch":f,"roll":f}` | Set gyro zero offset |
| `quadpod/cmd/calib/gyro/clear` | _(empty)_ | Clear gyro offset |
| `quadpod/cmd/calib/gyro/save` | _(empty)_ | Save gyro offset to NVS |
| `quadpod/cmd/ping` | _(empty)_ | Connectivity check |

### Gamepad (ESP32 → Broker)

| Topic | Payload | Description |
|---|---|---|
| `quadpod/gamepad` | `"move:forward"`, `"action:A"`, etc. | Button press events |
| `quadpod/system` | status string | Connect/disconnect events |
| `quadpod/logs` | string | USB serial forwarded data |

### State (ESP32 → Laptop)

| Topic | Payload | Description |
|---|---|---|
| `quadpod/state/telemetry` | JSON: gait, pose[12], offsets[12], … | Full telemetry snapshot |
| `quadpod/state/gyro` | JSON: yaw, pitch, roll, valid | Raw gyro from phone |
| `quadpod/state/status` | JSON: esp32Connected, uptime, … | System status |
| `quadpod/log` | JSON: level, msg, t | Structured log messages |

---

## Available Gaits

| ID | Description |
|---|---|
| `stand` | Stand / idle pose |
| `trot_forward` | Trot forward |
| `trot_backward` | Trot backward |
| `turn_left` | Turn left (in place) |
| `turn_right` | Turn right (in place) |
| `arc_left` | Arc left (forward + turn) |
| `arc_right` | Arc right (forward + turn) |

---

## Joint Map

12 servos organized as 4 legs × 3 joints:

| Index | Label | Joint Type |
|---|---|---|
| 0 | `FR_abd` | Front-Right Abductor |
| 1 | `FL_abd` | Front-Left Abductor |
| 2 | `RL_abd` | Rear-Left Abductor |
| 3 | `RR_abd` | Rear-Right Abductor |
| 4 | `FR_flex` | Front-Right Flexor |
| 5 | `FL_flex` | Front-Left Flexor |
| 6 | `RL_flex` | Rear-Left Flexor |
| 7 | `RR_flex` | Rear-Right Flexor |
| 8 | `FR_knee` | Front-Right Knee |
| 9 | `FL_knee` | Front-Left Knee |
| 10 | `RL_knee` | Rear-Left Knee |
| 11 | `RR_knee` | Rear-Right Knee |

---

## Part of DGIS

This repo is a component of the larger DGIS system. See the [main repo](https://github.com/AhmedMohamady1/DGIS) for full architecture.
