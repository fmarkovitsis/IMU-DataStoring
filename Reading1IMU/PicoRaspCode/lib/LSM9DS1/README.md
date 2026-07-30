LSM9DS1 Gyro/Magnetometer Helper
=================================

Purpose
-------
This file documents the helper functions implemented in `gyro_sensor.cpp` (part of the `lib/LSM9DS1` library). It explains how to initialize and read fused orientation values (roll, pitch, yaw) using `GyroSetup()` and `GyroLoop()`.

Requirements
------------
- Arduino (PlatformIO) project
- `Wire` and `Adafruit_LSM9DS1` libraries
- `KalmanFilter` implementation (present in this library)

Files
-----
- `lib/LSM9DS1/gyro_sensor.cpp` — implementation
- `lib/LSM9DS1/gyro_sensor.h` — header (include this in your sketch)

Key Functions
-------------
- `void GyroSetup()`
  - Initializes the `Adafruit_LSM9DS1` object and configures accelerometer, gyroscope, and magnetometer ranges.
  - Calls `calibrateSensors()` to compute gyro and accel offsets.
  - Prints an error to `Serial` if initialization fails (no return value).
  - Should be called once from `setup()`.

- `void GyroLoop(float &roll, float &pitch, float &yaw)`
  - Reads the IMU (accel, gyro, mag), applies stored offsets and magnetometer calibration values, and computes fused orientation:
    - `roll` and `pitch` are computed using a Kalman filter (degrees).
    - `yaw` is computed from magnetometer data and corrected for current roll/pitch (0..360 degrees).
  - Updates `prevTime` internally to compute `dt` for the Kalman filter.
  - Contains `delay(10)` internally (approx 100 Hz update rate). This is a blocking call — be aware when integrating with other timing-critical code.
  - Usage: call from `loop()` and pass three `float` variables by reference.

Other Useful Functions (internal/public helpers)
-----------------------------------------------
- `void calibrateSensors()` — collects ~500 samples and computes `gx_offset`, `gy_offset`, `gz_offset`, `ax_offset`, `ay_offset`, `az_offset`.
- `bool gyroSelfTest()` — runs full IMU + magnetometer self-test and returns `true` if the self-test passes.
- I2C helper functions in the file: `writeRegister8`, `readRegister8`, `readRaw`, `avgRaw`.

Configuration Notes
-------------------
- Magnetometer calibration constants are defined near the top of `gyro_sensor.cpp`:
  - `mx_offset`, `my_offset`, `mz_offset`
  - `mx_scale`, `my_scale`, `mz_scale`
  Adjust these values if you perform your own mag calibration.

- Accelerometer and gyro offsets are computed by `calibrateSensors()` and stored in global variables (`gx_offset`, etc.). If you want to persist offsets across boots, store them externally and assign the globals before calling `GyroLoop()`.

Units & Assumptions
-------------------
- The Adafruit sensor library returns acceleration in `m/s^2` and gyro in `degrees/second` (as provided by `sensors_event_t`). The code treats those values accordingly:
  - `az` is expected to be ~9.8 m/s^2 when the board is level; `calibrateSensors()` subtracts 9.8 from the averaged `az` to zero-center the Z accel offset.
  - Kalman filter `getAngle(accelAngle, gyroRate, dt)` expects angle in degrees and gyro rate in degrees/sec.

Example Usage
-------------
Include the header and call setup once, then read values in `loop()`:

```cpp
#include "gyro_sensor.h"

void setup() {
  Serial.begin(115200);
  GyroSetup(); // initializes and calibrates (blocking while calibrating)
}

void loop() {
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  GyroLoop(roll, pitch, yaw);
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print("  Pitch: "); Serial.print(pitch);
  Serial.print("  Yaw: "); Serial.println(yaw);
  // `GyroLoop` already delays ~10 ms; add extra delay only if needed
}
```

Integration Tips
----------------
- `GyroSetup()` will call `calibrateSensors()` which samples ~500 readings with small delays; that can take a few seconds. Call it once at start (for example, inside `setup()`).
- `GyroLoop()` contains a `delay(10)` to target ~100 Hz. If you need non-blocking behavior, copy the contents and replace `delay()` with millis-based scheduling.
- If magnetometer readings look skewed, update `mx_offset/my_offset/mz_offset` and scale factors after performing a proper mag calibration.
- If you want to persist offsets between boots, save the computed offsets to EEPROM or flash and set the global offset variables on startup before calling `GyroLoop()`.

Debugging
---------
- If the sensor fails to initialize, `GyroSetup()` prints "Failed to initialize LSM9DS1!" to `Serial`.
- Use `gyroSelfTest()` to perform a hardware self-test (returns boolean pass/fail).