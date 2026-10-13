# FlightCompV3_5 - Rocketry Flight Computer

## Overview

FlightCompV3_5 is the rocketry flight computer running on the **Arduino Portenta H7 (STM32H747XI - Cortex-M7 Core)**. It features high-speed **SPI communication** for onboard flight sensors and radio telemetry, alongside dedicated **UART GPS positioning**.

### Key Features:

* **Active Stabilization:** Uses a dual-axis PID control loop to actuate servos for aerodynamic flight correction.
* **Sensor Fusion:** Employs complementary and Kalman filters to process raw IMU data from the MPU6500 over high-speed SPI.
* **Altitude Sensing:** Utilizes a BMP280 barometer over SPI for real-time altitude tracking and ground-reference relative apogee calculations.
* **GPS Tracking:** Interfaces a u-blox NEO-M8N module over hardware `Serial1` (D13/D14) configured for 115200 baud and 10 Hz multi-GNSS tracking.
* **LoRa Telemetry:** Broadcasts live CSV telemetry packets (`alt,maxAlt,accelMag,lat,lng,chute`) over 433 MHz LoRa via the SX1278 transceiver.
* **Automated Recovery:** Detects apogee based on altitude drop and motor burnout acceleration thresholds to trigger parachute deployment via servo.
* **Modular Architecture:** Clean separation between configuration, hardware drivers, communication, and control logic.

### Hardware & Pinout Mapping (Portenta H7)

| Component / Function | Signal | Portenta Pin | Description |
| :--- | :--- | :--- | :--- |
| **Shared SPI1 Bus** | SCK | `D9` / `PI1` | Shared Hardware SPI Clock |
| **Shared SPI1 Bus** | MOSI / COPI | `D8` / `PC3` | Shared Hardware SPI Master Out |
| **Shared SPI1 Bus** | MISO / CIPO | `D10` / `PC2` | Shared Hardware SPI Master In |
| **MPU6500 IMU** | NCS (CS) | `D7` | SPI Chip Select for MPU6500 |
| **BMP280 Barometer** | CSB (CS) | `D5` | SPI Chip Select for BMP280 |
| **SX1278 LoRa** | NSS (CS) | `D4` | SPI Chip Select for SX1278 |
| **SX1278 LoRa** | DIO0 (IRQ) | `D3` | Packet TX Done Interrupt |
| **SX1278 LoRa** | RST | `D2` | Hardware Reset |
| **GPS (NEO-M8N)** | TX → MCU RX | `D13` | UART1 RX (`Serial1`) |
| **GPS (NEO-M8N)** | RX → MCU TX | `D14` | UART1 TX (`Serial1`) |
| **Servo X** | PWM | `D0` | X-Axis TVC / Fin Actuation Servo |
| **Servo Y** | PWM | `D1` | Y-Axis TVC / Fin Actuation Servo |
| **Chute Servo** | PWM | `D11` | Parachute Deployment Mechanism |

*Note: All pin numbers are configurable in `src/Config/flightConfig.h`.*

---

### Code Structure:

```text
FlightCompV3_5/
├── platformio.ini                    <- PlatformIO target & dependency config
├── README.md                         <- Documentation
└── src/
    ├── FlightCompV3_5.cpp            <- Main setup() and loop() entry point
    ├── Config/
    │   └── flightConfig.h            <- Pin definitions, PID gains, thresholds
    ├── Comms/
    │   ├── Telemetry.cpp             <- LoRa SX1278 433MHz packet broadcast
    │   └── Telemetry.h
    ├── Control/
    │   ├── flightController.cpp      <- Servo control with PID
    │   ├── flightController.h
    │   ├── kalman.cpp                <- 1D Kalman state estimator
    │   └── kalman.h
    ├── Drivers/
    │   ├── driver_BMP280.cpp         <- SPI Barometer driver (Adafruit BMP280)
    │   ├── driver_BMP280.h
    │   ├── driver_GPS.cpp            <- UART GPS driver with UBX 10Hz config
    │   ├── driver_GPS.h
    │   ├── driver_MPU6500.cpp        <- SPI IMU driver wrapper
    │   └── driver_MPU6500.h
    └── libs/
        ├── MPU6500_SPI.cpp           <- Custom high-speed SPI MPU6500 driver
        ├── MPU6500_SPI.h
        ├── PID_v1.cpp                <- Standard PID library
        └── PID_v1.h
```

---

## Development Environment Setup

### 1. Visual Studio Code & PlatformIO (Recommended)

The project is configured for **PlatformIO** targeting the Portenta H7 M7 core:

1. **Install VS Code**: Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. **Install PlatformIO**: Go to the Extensions view in VS Code and install "PlatformIO IDE".
3. **Open Project**: Select "Open Project" and choose the `FlightCompV3_5` directory.
4. **Build & Upload**:
   - Click the checkmark icon (`✓`) in the PlatformIO bottom bar to build firmware.
   - Click the arrow icon (`→`) to upload to the Portenta H7 connected via USB-C.

### 2. Arduino IDE (Alternative)

To compile using Arduino IDE (version 2.x or later):

1. **Install Board Core**: In Boards Manager, search for **Arduino Mbed OS Portenta Boards** and click Install.
2. **Select Board**: Under `Tools -> Board`, choose **Arduino Portenta H7 (M7 core)**.
3. **Install Libraries**: Open Library Manager (`Sketch -> Include Library -> Manage Libraries...`) and install:
   - `Adafruit BMP280 Library` (and its dependencies: `Adafruit Unified Sensor`, `Adafruit BusIO`)
   - `sandeepmistry/LoRa`
   - `mikalhart/TinyGPSPlus`
   - `Servo`
4. **Rename Main File**: In Arduino IDE, the sketch folder and main file must match. Rename `src/FlightCompV3_5.cpp` to `src/FlightCompV3_5.ino`.