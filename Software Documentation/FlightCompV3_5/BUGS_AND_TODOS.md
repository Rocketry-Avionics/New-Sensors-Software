# FlightCompV3.5 — Known Bugs & What Needs Fixing (TODOs)

Hey team! Here is the plain-spoken breakdown of everything that is broken, sketchy, or dangerous in the FlightCompV3.5 firmware right now. 

We ran the board through a 100-test simulation harness and uncovered a bunch of architectural defects that will crash the airframe, shred parachutes, or lock up the board if flown as-is.

Use this document as the master pre-flight punch list.

---

## 🚨 Critical Flight Safety Bugs (Will Crash the Rocket)

### 1. [ ] TVC has zero Y-axis control (Pitch only, no Yaw stabilization)
* **Where:** [`src/Control/flightController.cpp:66`](src/Control/flightController.cpp#L66) and [`src/FlightCompV3_5.cpp:205`](src/FlightCompV3_5.cpp#L205)
* **What's broken:** We wired up Pitch (X) and Roll (Z), but Yaw (Y) is completely disconnected. `src/FlightCompV3_5.cpp` doesn't pass Y to the controller, and in `src/Control/flightController.cpp`, `pidY.Compute()` is literally commented out. The servo mixer only mixes X and Z.
* **Why it matters:** If the rocket tilts or catches a wind gust in the Y plane, the TVC servos do **literally nothing (0.0° response)**. The rocket will tumble out of control during motor burn.
* **Action:**
  - [ ] In `src/FlightCompV3_5.cpp:205`, update the call to pass `imu.getRawDiffY()`.
  - [ ] In `src/Control/flightController.cpp`, uncomment `pidY.Compute()`.
  - [ ] Update the servo mixer in `src/Control/flightController.cpp:69-70` to use both X and Y:
    ```cpp
    servo_x = constrain(90 - (0.707 * (outputY - outputX)), 0, 180);
    servo_y = constrain(90 - (0.707 * (outputY + outputX)), 0, 180);
    ```

---

### 2. [x] Default ±2g Accelerometer Saturation Bug *(Patched in current working tree)*
* **Where:** [`src/Drivers/driver_MPU6500.cpp:7`](src/Drivers/driver_MPU6500.cpp#L7)
* **What was broken:** Calling `mpu.begin()` with no arguments defaulted the MPU6500 to its lowest ±2g range. The sensor register saturates at 1.999g. But our liftoff threshold in `src/Config/flightConfig.h` is 2.5g! The rocket could be pulling 10g on the pad, but the sensor maxed out at 1.999g, so the board thought it was still sitting on the pad forever and never fired the parachute.
* **Current status:** Patched with `mpu.begin(1, 3)` (gyro ±500 dps, accel ±16g).
* **Action:**
  - [x] Verified `mpu.begin(1, 3)` sets ±16g full-scale rocketry range.

---

### 3. [ ] Sensor boot failure freezes the board in an infinite loop (`while(1);`)
* **Where:** [`src/Drivers/driver_BMP280.cpp:13`](src/Drivers/driver_BMP280.cpp#L13) and [`src/Drivers/driver_MPU6500.cpp:10`](src/Drivers/driver_MPU6500.cpp#L10)
* **What's broken:** If either the barometer or the IMU fails to respond over SPI when booting up (loose jumper wire, power glitch, bus contention), the code runs `while(1);`.
* **Why it matters:** The microcontroller just sits there completely frozen. No telemetry packets get broadcast, no warning buzzer sounds, no watchdog reset happens. If a connector jiggles loose on the launch rail, the avionics die silently.
* **Action:**
  - [ ] Remove `while(1);` from both sensor drivers.
  - [ ] Return `false` and log a clear error over radio and serial.
  - [ ] Allow the board to enter a degraded safe recovery mode or retry sensor init periodically.

---

### 4. [ ] Low-altitude flights crater into the ground before the backup timer fires (<60m flights)
* **Where:** [`src/FlightCompV3_5.cpp:150-164`](src/FlightCompV3_5.cpp#L150)
* **What's broken:** Primary apogee detection requires `maxAltitude > 60m` (`MIN_ARM_ALTITUDE`). If a small motor, high drag, or partial burn/CATO peaks at 35m or 50m, apogee detection never arms! The backup safety timer takes 10.0 full seconds post-burnout to trigger.
* **Why it matters:** A rocket falling from 35m hits the ground in ~2.7 seconds. Ground impact happens around T+3.2s, which is **6.8 seconds before the backup timer expires**. The parachute will deploy inside the crash crater.
* **Action:**
  - [ ] Add velocity-based descent detection: if altitude has dropped steadily for multiple samples, deploy regardless of the 60m threshold.
  - [ ] Lower `MIN_ARM_ALTITUDE` to ~15m–20m, or use a dynamic timer based on motor burnout time.

---

### 5. [ ] High-altitude flights will shred the parachute during ascent (>1000m flights)
* **Where:** [`src/Config/flightConfig.h:39`](src/Config/flightConfig.h#L39)
* **What's broken:** The backup timer (`APOGEE_BACKUP_TIMEOUT_MS`) is hardcoded to 10.0 seconds post-burnout.
* **Why it matters:** On high-power flights (coasting for 14–20 seconds to reach 1500m), at 10 seconds post-burnout the rocket is still climbing upward at over 160 km/h (45 m/s). The backup timer blindly triggers and deploys the parachute into high-speed dynamic ascent, shredding the canopy.
* **Action:**
  - [ ] Invalidate the backup timer if vertical velocity is positive or barometer altitude is actively climbing.
  - [ ] Scale the backup timer based on burnout velocity: $t_{\text{backup}} \approx (v_{\text{burnout}} / g) + 2.0\text{s}$.

---

### 6. [ ] In-flight brownout / power glitch wipes state and re-locks the parachute
* **Where:** [`src/FlightCompV3_5.cpp:47-90`](src/FlightCompV3_5.cpp#L47)
* **What's broken:** If TVC servo current draw causes a momentary voltage sag or battery contacts glitch from vibration, the Portenta H7 resets. On reboot, it runs `setup()` from scratch with zero memory of the flight.
* **Why it matters:**
  - If it reboots during boost: it zeroes out ground pressure at 40m altitude and resets to `STATE_PAD`.
  - If it reboots during coast: it zeroes out altitude at 180m; as the rocket descends below 180m, relative altitude reads as negative, permanently blocking the 60m apogee arming check.
  - If it reboots under parachute: `setup()` forces `parachuteServo.write(90)`, **actively re-locking the parachute servo in mid-air**.
* **Action:**
  - [ ] Save the current `flightState`, ground reference pressure, and burnout timestamp into Portenta RTC backup registers or non-volatile FRAM.
  - [ ] In `setup()`, check if the MCU just rebooted from an in-flight state and resume the state machine.

---

## ⚡ Performance Bottlenecks & Timing Hazards

### 7. [ ] Serial debug at 9600 baud starves the 250 Hz control loop by 17x
* **Where:** [`src/FlightCompV3_5.cpp:53`](src/FlightCompV3_5.cpp#L53)
* **What's broken:** `Serial.begin(9600)` prints ~65 bytes of debug text every 100ms. Sending 65 bytes at 9600 baud takes **67.7 milliseconds**.
* **Why it matters:** Our TVC control loop budget is **4.0 milliseconds** (250 Hz). Once the hardware 64-byte serial buffer fills up, `Serial.print()` blocks the CPU for up to 60ms. The PID controller freezes and misses 15 control updates every 100ms.
* **Action:**
  - [ ] Bump `Serial.begin(115200);` or `921600;` to drop print transmission time to ~5ms or <1ms.

---

### 8. [ ] Servo Y 153° trim snaps to 90° on boot
* **Where:** [`src/Control/flightController.cpp:9, 34, 62`](src/Control/flightController.cpp#L9)
* **What's broken:** In `src/Control/flightController.cpp`, `servo_y` is initialized to 153° in constructor and `init()`. But on the very first loop pass, the deadband check forces `servo_y = 90`.
* **Why it matters:** The moment the board boots, Servo Y violently snaps by **63 degrees**. If 153° is the physical airframe neutral trim, the controller centers around 90°, introducing a permanent 63° steering bias.
* **Action:**
  - [ ] Verify physical servo linkage on the bench.
  - [ ] If neutral is 90°, initialize to 90°. If neutral is 153°, update the mixer and deadband logic to center around 153°.

---

### 9. [ ] SD card flash erase block stalls freeze the TVC PID loop
* **Where:** [`src/Logging/Blackbox.cpp:108-112`](src/Logging/Blackbox.cpp#L108)
* **What's broken:** Synchronous SD card writes. MicroSD cards periodically trigger internal flash block erase cycles that take 150ms to 250ms.
* **Why it matters:** When a flash erase occurs during boost, the entire MCU blocks waiting on SPI/SDMMC, causing the TVC servos to freeze in place mid-burn for 60 loop cycles.
* **Action:**
  - [ ] Implement a double-buffered ring buffer so writes happen in the background via DMA.
  - [ ] Alternatively, offload SD logging to the Portenta H7 secondary Cortex-M4 core via RPC.

---

### 10. [ ] Short-burn pulse motor race condition
* **Where:** [`src/FlightCompV3_5.cpp:116`](src/FlightCompV3_5.cpp#L116)
* **What's broken:** Liftoff detection requires `az > 2.5g && altitude > 5.0m` at the same instant.
* **Why it matters:** Fast-burning pulse motors (150ms burn) finish burning before the rocket reaches 2 meters. By the time the rocket coasts past 5 meters, thrust is already zero ($a_z \le 0g$). The board never transitions to `STATE_BOOST` and stays in `STATE_PAD`.
* **Action:**
  - [ ] Latch liftoff if `az > 2.5g` is maintained on the rail for 4 samples, OR evaluate velocity/altitude separately without requiring simultaneous peak acceleration above 5m.

---

## 🧹 Code Quality & Cleanup TODOs

- [ ] **Fix compiler member reordering warning:** In `src/Control/flightController.h`, `outputZ` is declared before `servo_x`, which causes GCC constructor reorder warnings. Declare members in the same order as the constructor list.
- [ ] **Clean up unused variables:** Remove unused `inputY`, `outputY`, `pidY`, and `kalmanY` declarations or properly wire them into the 2-axis TVC pipeline.
- [ ] **Document physical frame orientations:** Clearly label +X (Pitch) and +Y (Yaw) axis directions on the PCB relative to the servo horns to prevent inverted control responses during assembly.
