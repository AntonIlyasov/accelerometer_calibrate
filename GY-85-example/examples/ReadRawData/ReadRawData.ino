#include "GY_85.h"

// Create module object
GY_85 GY85;

void setup() {
    // Initialize the serial communication:
    Serial.begin(115200);

    // Initialize module
    GY85.init();
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
    
    int16_t* compassReadings = GY85.readFromCompass();
    int16_t cx = GY85.compass_x(compassReadings);
    int16_t cy = GY85.compass_y(compassReadings);
    int16_t cz = GY85.compass_z(compassReadings);

    int16_t* gyroReadings = GY85.readGyro();
    float gx = GY85.gyro_x(gyroReadings);
    float gy = GY85.gyro_y(gyroReadings);
    float gz = GY85.gyro_z(gyroReadings);
    float gt = GY85.temp(gyroReadings);
    
    // Log it to serial port
    Serial.print("accelerometer");
    Serial.print(" x:");
    Serial.print(X_out);
    Serial.print(" y:");
    Serial.print(Y_out);
    Serial.print(" z:");
    Serial.print(Z_out);
    
    Serial.print("\t compass");
    Serial.print(" x:");
    Serial.print(cx);
    Serial.print(" y:");
    Serial.print(cy);
    Serial.print(" z:");
    Serial.print(cz);
    
    Serial.print("\t  gyro");
    Serial.print(" x:");
    Serial.print(gx);
    Serial.print(" y:");
    Serial.print(gy);
    Serial.print(" z:");
    Serial.print(gz);
    Serial.print("\t gyro temp:");
    Serial.println(gt);
    
    // Make delay between readings
    delay(50);
}