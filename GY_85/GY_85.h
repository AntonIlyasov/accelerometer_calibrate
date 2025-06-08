#ifndef GY_85_h
#define GY_85_h

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

//----------addresses----------//
#define ADXL345 (0x53)         // Device address as specified in data sheet //ADXL345 accelerometer
#define DATAX0  (0x32)         //X-Axis Data 0
//#define DATAX1 0x33          //X-Axis Data 1
//#define DATAY0 0x34          //Y-Axis Data 0
//#define DATAY1 0x35          //Y-Axis Data 1
//#define DATAZ0 0x36          //Z-Axis Data 0
//#define DATAZ1 0x37          //Z-Axis Data 1
#define HMC5883 (0x1E)         //gyro
#define ITG3200 (0x68)         //compass


class GY_85 {
    
private:
    void GyroCalibrate();
    void SetGyro();
    void SetCompass();
    void SetAccelerometer();
    
public:
    void   init();
    int16_t*   readFromAccelerometer();
    int16_t*   readFromCompass();
    int16_t* readGyro();
    
    //callback functions
    inline int16_t accelerometer_x( int16_t* a ){ return *(   a ); }
    inline int16_t accelerometer_y( int16_t* a ){ return *( 1+a ); }
    inline int16_t accelerometer_z( int16_t* a ){ return *( 2+a ); }
    
    //-----------------------------------
    
    inline int16_t compass_x( int16_t* a ){ return *(   a ); }
    inline int16_t compass_y( int16_t* a ){ return *( 1+a ); }
    inline int16_t compass_z( int16_t* a ){ return *( 2+a ); }
    
    //-----------------------------------
    
    inline float gyro_x( int16_t* a ){ return *(   a ) / 14.375; }
    inline float gyro_y( int16_t* a ){ return *( 1+a ) / 14.375; }
    inline float gyro_z( int16_t* a ){ return *( 2+a ) / 14.375; }
    inline float temp  ( int16_t* a ){ return  35+( *( 3+a )+13200 ) / 280; }
};

#endif
