#include <Wire.h>
#include "GY_85.h"
#define X_CALIB 0
#define Y_CALIB 1
#define Z_CALIB 2
#define MODE_CALIB X_CALIB

// int ADXL345 = 0x53; // The ADXL345 sensor I2C address

GY_85 GY85;

void setup() {
  Serial.begin(230400);
  GY85.init();
  Wire.begin();
  // Note: You should calibrate upon re-powering the sensor
  // Uncomment if you want to calibrate
  Serial.println("Beginning Calibration1");
  calibrateADXL345(); // Calibrate the sensor
  Serial.println("Beginning Calibration2");
}

void calibrateADXL345() {
  float X_out, Y_out, Z_out; // Outputs
  int X_offset = 0, Y_offset = 0, Z_offset = 0; // Offset values
  // Serial.println("Beginning Calibration");
  // You should calibrate with the positive axis pointed upwards against gravity
  // You should calibrate each axis separately and change the offset constant, 
  // You need to run this code three times if you want to calibrate all three axes
  float numReadings = 1000;

#if MODE_CALIB == X_CALIB
  float xSum = 0;
#endif

#if MODE_CALIB == Y_CALIB
  float ySum = 0;
#endif

#if MODE_CALIB == Z_CALIB
  float zSum = 0;
#endif

  Serial.println("Beginning Calibration");
  for (int i = 0; i < numReadings; i++) {
    Serial.println(i);

    // Read data from sensors
    int16_t* accelerometerReadings = GY85.readFromAccelerometer();
    int16_t ax = GY85.accelerometer_x(accelerometerReadings);
    int16_t ay = GY85.accelerometer_y(accelerometerReadings);
    int16_t az = GY85.accelerometer_z(accelerometerReadings);

    // Масштабирование (для ±4g: 128 LSB/g)
    X_out = (float)ax / 128.0;
    Y_out = (float)ay / 128.0;
    Z_out = (float)az / 128.0;

#if MODE_CALIB == X_CALIB
    xSum += X_out;
#endif

#if MODE_CALIB == Y_CALIB
    ySum += Y_out;
#endif

#if MODE_CALIB == Z_CALIB
    zSum += Z_out;
#endif

  }

#if MODE_CALIB == X_CALIB
  // X_offset = (128 - xSum / numReadings) / 4;
  X_offset = xSum / numReadings / 128.0;
  Serial.print("X_offset= " );
  Serial.print(X_offset, 6);
#endif

#if MODE_CALIB == Y_CALIB
  // Y_offset = (128 - ySum / numReadings) / 4;
  Y_offset = ySum / numReadings / 128.0;
  Serial.print("Y_offset= " );
  Serial.print(Y_offset, 6);
#endif

#if MODE_CALIB == Z_CALIB
  // Z_offset = (128 - (zSum / numReadings)) / 4;
  Z_offset = zSum / numReadings / 128.0;
  Serial.print("Z_offset= " );
  Serial.print(Z_offset, 6);
#endif

  Serial.println();
  delay(1000);
  // We need to add it to the offset channel respectively =
  // Once you write to the offset channel you do not need to do it again,
  // unless you unpower and power the device
  // Each offset address is different for X, Y, and Z

#if MODE_CALIB == X_CALIB
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1E);
  Wire.write(X_offset);
  Wire.endTransmission();
#endif

#if MODE_CALIB == Y_CALIB
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1F);
  Wire.write(Y_offset);
  Wire.endTransmission();
#endif

#if MODE_CALIB == Z_CALIB
  Wire.beginTransmission(ADXL345);
  Wire.write(0x20);
  Wire.write(Z_offset);
  Wire.endTransmission();
#endif
}

void loop() {
  // Read data from sensors
  int16_t* accelerometerReadings = GY85.readFromAccelerometer();
  int16_t ax = GY85.accelerometer_x(accelerometerReadings);
  int16_t ay = GY85.accelerometer_y(accelerometerReadings);
  int16_t az = GY85.accelerometer_z(accelerometerReadings);

  // Масштабирование (для ±4g: 128 LSB/g)
  float X_out = (float)ax / 128.0;
  float Y_out = (float)ay / 128.0;
  float Z_out = (float)az / 128.0;

  Serial.print("X_out = ");
  Serial.print(X_out);
  Serial.print("  Y_out = ");
  Serial.print(Y_out);
  Serial.print("  Z_out = ");
  Serial.println(Z_out);
  delay(50);
}