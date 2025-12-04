# IMU Orientation & Gesture Recognition (Simple)

A compact, interview-friendly IMU project that computes Roll & Pitch (and basic yaw if magnetometer present) and supports simple gesture detection (tilt, shake).  
This sketch is intentionally simple so you can run it quickly in Arduino IDE and explain every line during interviews.

---

## Features
- Reads accelerometer + gyroscope (MPU-6050 / MPU-9250 / ICM-20948 compatible)
- Computes Roll & Pitch using a Complementary Filter (stable & lightweight)
- Simple gesture detection: tilt left/right, shake (threshold-based)
- Serial telemetry for live plotting/inspection
- Optional: swap complementary filter for Kalman filter (notes below)

---

## Hardware (typical)
- Arduino Uno / Nano / Pro Mini (or STM32 bluepill with Arduino core)
- MPU-6050 (6DOF) or MPU-9250 / ICM-20948 (9DOF if you want magnetometer)
- Jumper wires, breadboard
- USB cable for serial output

**I2C wiring (MPU -> Arduino):**
- VCC → 5V (or 3.3V for some modules)
- GND → GND
- SDA → A4 (Uno/Nano) or SDA pin on your board
- SCL → A5 (Uno/Nano) or SCL pin on your board
- INT (optional) → any digital pin if you want interrupts

---

## Quick start
1. Open `IMU_Orientation_Gesture.ino` in Arduino IDE.
2. Install library (optional but recommended):
   - `MPU6050` or `MPU9250` / `Wire` (Wire is built-in)
3. Upload to board.
4. Open Serial Monitor at **115200** baud to see Roll/Pitch values and detected gestures.

---

## How it works (simple)
- Read raw accelerometer & gyro via I2C.
- Accelerometer gives absolute orientation (noisy).
- Gyroscope gives smooth rate (drifts over time).
- Complementary filter fuses both: `angle = alpha*(angle + gyro*dt) + (1-alpha)*acc_angle`.
- Gesture detection uses thresholds on roll/pitch rate and acceleration magnitude.

---

## Notes on Kalman filter
- A Kalman filter gives better performance but is more math-heavy.
- You can replace the complementary step with a Kalman implementation (many open-source C/C++ snippets available).
- For interviews, describing why you used complementary vs Kalman is sufficient.

---

}


