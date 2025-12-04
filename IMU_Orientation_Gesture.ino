
#include <Wire.h>

// If you have the MPU6050 library installed you can use it.
// This sketch uses direct I2C reads to keep dependencies minimal.

// --------- CONFIG ----------
const int MPU_ADDR = 0x68; // default I2C address for MPU-6050
const float GYRO_SCALE = 131.0;   // LSB/deg/s for ±250deg/s
const float ACC_SCALE = 16384.0;  // LSB/g for ±2g

// Complementary filter coefficient (0..1). Higher => trust gyro more.
const float ALPHA = 0.98;

unsigned long lastTime = 0;
float roll = 0, pitch = 0;

// Gesture detection thresholds
const float TILT_ANGLE_THRESHOLD = 20.0; // degrees
const float SHAKE_ACCEL_THRESHOLD = 1.8; // g (approx) - tune as per sensor/module

// ---------- HELPERS ----------
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count);
  int i = 0;
  while (Wire.available() && i < count) {
    dest[i++] = Wire.read();
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  // Wake up MPU-6050 (clear sleep bit)
  writeRegister(0x6B, 0x00);

  // Optional: set gyro range / acc range here if needed

  lastTime = millis();
  Serial.println("IMU Orientation & Gesture Demo");
  Serial.println("Roll,Pitch,Gesture");
}

// ---------- MAIN LOOP ----------
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // Read raw accel + gyro (14 bytes from 0x3B)
  uint8_t buf[14];
  readRegisters(0x3B, 14, buf);

  // Accel raw (signed 16-bit)
  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];

  // Temp skipped (buf[6],buf[7])

  // Gyro raw
  int16_t gx = (buf[8] << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  // Convert to physical units
  float ax_g = ax / ACC_SCALE;
  float ay_g = ay / ACC_SCALE;
  float az_g = az / ACC_SCALE;

  float gx_dps = gx / GYRO_SCALE;
  float gy_dps = gy / GYRO_SCALE;
  float gz_dps = gz / GYRO_SCALE;

  // Compute accel angles (degrees)
  float rollAcc = atan2(ay_g, az_g) * 180.0 / PI;
  float pitchAcc = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0 / PI;

  // Integrate gyro rates to angles
  float rollGyro = roll + gx_dps * dt;
  float pitchGyro = pitch + gy_dps * dt;

  // Complementary filter
  roll = ALPHA * rollGyro + (1 - ALPHA) * rollAcc;
  pitch = ALPHA * pitchGyro + (1 - ALPHA) * pitchAcc;

  // Gesture detection (simple)
  String gesture = "None";
  if (roll > TILT_ANGLE_THRESHOLD) gesture = "TiltRight";
  else if (roll < -TILT_ANGLE_THRESHOLD) gesture = "TiltLeft";

  float totalAccel = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  if (totalAccel > SHAKE_ACCEL_THRESHOLD) gesture = "Shake";

  // Telemetry (CSV-style)
  Serial.print(roll, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.println(gesture);

  delay(10); // small delay to stabilize serial output
}
