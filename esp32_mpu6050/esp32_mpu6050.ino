#define ACCELE_RANGE 4.0
#define GYROSC_RANGE 500.0

#include <Wire.h>

// ESP32-S3 I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

const int MPU_addr = 0x69; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  Serial.begin(115200);  // Higher baud rate for ESP32
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C with custom pins

  // Wake up MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() {
  // Request sensor data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // Request 14 registers

  // Read accelerometer values
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();  // Temperature (raw)
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Convert raw values to proper units
  float ax = (AcX / 32768.0) * ACCELE_RANGE;
  float ay = (AcY / 32768.0) * ACCELE_RANGE;
  float az = (AcZ / 32768.0) * ACCELE_RANGE;
  float gx = (GyX / 32768.0) * GYROSC_RANGE;
  float gy = (GyY / 32768.0) * GYROSC_RANGE;
  float gz = (GyZ / 32768.0) * GYROSC_RANGE;
  float temp = Tmp / 340.00 + 36.53;  // Convert temperature to Celsius

  // Print sensor data
  Serial.print("AcX: "); Serial.print(ax); Serial.print("g ");
  Serial.print("| AcY: "); Serial.print(ay); Serial.print("g ");
  Serial.print("| AcZ: "); Serial.print(az); Serial.println("g ");

  Serial.print("GyX: "); Serial.print(gx); Serial.print(" d/s ");
  Serial.print("| GyY: "); Serial.print(gy); Serial.print(" d/s ");
  Serial.print("| GyZ: "); Serial.println(gz);

  Serial.print("Temperature: "); Serial.print(temp); Serial.println(" °C\n");

  delay(500);
}
