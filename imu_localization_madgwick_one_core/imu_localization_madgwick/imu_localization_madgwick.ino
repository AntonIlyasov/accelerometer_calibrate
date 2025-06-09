#include <MadgwickAHRS.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "integrator.h"
#include "GY_85.h"
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>

#define CONTROLLER (0x08)            // Device address of controller

float kalman_x = 1.0;
float kalman_y = 1.0;
float kalman_z = 1.0;

// Общие переменные
volatile int aclFromIMU   = 0;
volatile int velFromIMU   = 0;
volatile int moveFromIMU  = 0;

TinyGPSPlus gps;

const int up   = 1;
const int down = -1;

int update_freq = 200;             // Hz

GY_85 GY85;
Madgwick filter;

float a11 = 0.995684022834473;
float a12 = 0.000565235492969873;
float a13 = 0.0908517828526542;

float a21 = -0.00108257651877768;
float a22 = 0.998128421513835;
float a23 = 0.0100622721774157;

float a31 = -0.0914653605347865;
float a32 = 0.000546663798240473;
float a33 = 0.997877385059952;

float tx = -0.0446083373642039;
float ty = 0.0106734191616384;
float tz = -0.10686798215027;

float ax = 0;
float ay = 0.000110613889490412;
float az = -0.000022656581599363;

float kx = 0.957211964747618;
float ky = 0.959487777177509;
float kz = 1.00593041448924;

float bx = 0.000030813401878492;
float by = 0.00758626638948153;
float bz = 0.000350538306800816;

struct Drone_State
{
public:

  Drone_State(){
    integrator_ax = new Integrator(0.0);
    integrator_ay = new Integrator(0.0);
    integrator_az = new Integrator(0.0);
    integrator_vx = new Integrator(0.0);
    integrator_vy = new Integrator(0.0);
    integrator_vz = new Integrator(0.0);
    integrator_gx = new Integrator(0.0);
    integrator_gy = new Integrator(0.0);
    integrator_gz = new Integrator(0.0);
  }

  void madgwick_filter(){    
    filter.updateIMU(drone_state.gx, drone_state.gy, drone_state.gz, drone_state.ax, drone_state.ay, drone_state.az);
    if (drone_state.firstMadgwickMeasure){
      drone_state.offset_roll_x   = filter.getRoll();
      drone_state.offset_pitch_y  = filter.getPitch();
      drone_state.offset_yaw_z    = filter.getYaw();
      drone_state.firstMadgwickMeasure = false;
    }
    drone_state.roll_x_from_Madgwick  = filter.getRoll()  - drone_state.offset_roll_x;
    drone_state.pitch_y_from_Madgwick = filter.getPitch() - drone_state.offset_pitch_y;
    drone_state.yaw_z_from_Madgwick   = filter.getYaw()   - drone_state.offset_yaw_z;
  }

  void kalman_filter(){
    drone_state.ax = (1 - kalman_x) * drone_state.ax_prev + kalman_x * drone_state.ax;
    drone_state.ay = (1 - kalman_y) * drone_state.ay_prev + kalman_y * drone_state.ay;
    drone_state.az = (1 - kalman_z) * drone_state.az_prev + kalman_z * drone_state.az;

    drone_state.ax_prev = drone_state.ax;
    drone_state.ay_prev = drone_state.ay;
    drone_state.az_prev = drone_state.az;
  }

void get_location()
{
  if (gps.location.isValid())
  {
    drone_state.gps_lat = gps.location.lat();
    drone_state.gps_lon = gps.location.lng();
    drone_state.gps_alt = gps.altitude.meters();
  }
}

  void gps_update(){
    while (Serial1.available() > 0){
      gps.encode(Serial1.read());
    }

    if (gps.location.isUpdated()){
      get_location();
    }
  }

  void update_state(float dt){        // dt - [sec]

    // Serial.println(dt, 6);

    // gps_update();

    // Serial.println("update_state update_state update_state update_state update_state update_state update_state update_state");

            // START COMMUNICATION WITH IMU //
            
            // GET DATA FROM SENSOR //

    // get current linear acceleration
    int16_t X_out, Y_out, Z_out;  // Outputs
    int16_t* accelerometerReadings = GY85.readFromAccelerometer();
    X_out = GY85.accelerometer_x(accelerometerReadings);
    drone_state.ax = (float)X_out / 128.0;
    Y_out = GY85.accelerometer_y(accelerometerReadings);
    drone_state.ay = (float)Y_out / 128.0;
    Z_out = GY85.accelerometer_z(accelerometerReadings);
    drone_state.az = (float)Z_out / 128.0;

    // Serial.print("drone_state.ax:");
    // Serial.print(drone_state.ax);
    // Serial.print("\tdrone_state.ay:");
    // Serial.print(drone_state.ay);
    // Serial.print("\tdrone_state.az:");
    // Serial.println(drone_state.az);

    // get current angular velocities
    int16_t* gyroReadings = GY85.readGyro();
    drone_state.gx = GY85.gyro_x(gyroReadings);                 // получаем сырые угловые скорости
    drone_state.gy = GY85.gyro_y(gyroReadings);
    drone_state.gz = GY85.gyro_z(gyroReadings);

    // Serial.print("drone_state.gx:");
    // Serial.print(drone_state.gx);
    // Serial.print("\tdrone_state.gy:");
    // Serial.print(drone_state.gy);
    // Serial.print("\tdrone_state.gz:");
    // Serial.println(drone_state.gz);

                // CALCULATE //

    // do_offset_accelerations();                                  // получаю сырые ускорения в СК робота
    // kalman_filter();                                            // получаю отфильтрованные ускорения в СК робота
    madgwick_filter();                                          // получаю ориентацию робота

    Serial.print("roll_x_from_Madgwick:");
    Serial.print(drone_state.roll_x_from_Madgwick);
    Serial.print("\tpitch_y_from_Madgwick:");
    Serial.print(drone_state.pitch_y_from_Madgwick);
    Serial.print("\tyaw_z_from_Madgwick:");
    Serial.println(drone_state.yaw_z_from_Madgwick);

    // get current angular movements
    drone_state.droll_x  = integrator_gx->update(drone_state.gx, dt);   // получаем углы через угловые скорости
    drone_state.dpitch_y = integrator_gy->update(drone_state.gy, dt);
    drone_state.dyaw_z   = integrator_gz->update(drone_state.gz, dt);

    drone_state.roll_x  += drone_state.droll_x;
    drone_state.pitch_y += drone_state.dpitch_y;
    drone_state.yaw_z   += drone_state.dyaw_z;

    // Serial.print("     roll_x:");
    // Serial.print(drone_state.roll_x);
    // Serial.print("     pitch_y:");
    // Serial.print(drone_state.pitch_y);
    // Serial.print("     yaw_z:");
    // Serial.println(drone_state.yaw_z);

    calculate_aW();                                                     // получаем мировые ускорения

    // if (abs(drone_state.axW) <= 0.03) drone_state.axW = 0;              // фильтруем мировые ускорения
    // if (abs(drone_state.ayW) <= 0.03) drone_state.ayW = 0;
    // if (abs(1 - drone_state.azW) <= 0.05) drone_state.azW = 1;

    // get current linear velocities in GSK
    drone_state.dvxW = integrator_ax->update(drone_state.axW*9.81,        dt);    // получаем линейные скорости через ускорения
    drone_state.dvyW = integrator_ay->update(drone_state.ayW*9.81,        dt);
    drone_state.dvzW = integrator_az->update(drone_state.azW*9.81 - 9.81, dt);

    // Serial.print("     dvxW:");
    // Serial.print(drone_state.dvxW, 6);
    // Serial.print("     dvyW:");
    // Serial.print(drone_state.dvyW, 6);
    // Serial.print("     dvzW:");
    // Serial.print(drone_state.dvzW, 6);   

    drone_state.vxW += drone_state.dvxW;
    drone_state.vyW += drone_state.dvyW;
    drone_state.vzW += drone_state.dvzW;

    // get current linear movement
    drone_state.dxW = integrator_vx->update(drone_state.vxW, dt);     // получаем линейные перемещения через скорости
    drone_state.dyW = integrator_vy->update(drone_state.vyW, dt);
    drone_state.dzW = integrator_vz->update(drone_state.vzW, dt);

    // Serial.print("     dxW:");
    // Serial.print(drone_state.dxW, 6);
    // Serial.print("     dyW:");
    // Serial.print(drone_state.dyW, 6);
    // Serial.print("     dzW:");
    // Serial.print(drone_state.dzW, 6);        !!

    drone_state.xW += drone_state.dxW;
    drone_state.yW += drone_state.dyW;
    drone_state.zW += drone_state.dzW;

    aclFromIMU   = trunc(drone_state.axW*1000);
    velFromIMU   = trunc(drone_state.vxW*1000);
    moveFromIMU  = trunc(drone_state.xW*1000);





    // Serial.print("\tax:");
    // Serial.print(drone_state.ax);
    // Serial.print("\tay:");
    // Serial.print(drone_state.ay);
    // Serial.print("\taz:");
    // Serial.println(drone_state.az);

    // Serial.print("\taxW:");
    // Serial.print(drone_state.axW);
    // Serial.print("     ayW:");
    // Serial.print(drone_state.ayW);
    // Serial.print("     azW:");
    // Serial.println(drone_state.azW);

    // Serial.print("\tgps_lat:");
    // Serial.print(drone_state.gps_lat);
    // Serial.print("     gps_lon:");
    // Serial.print(drone_state.gps_lon);
    // Serial.print("     gps_alt:");
    // Serial.print(drone_state.gps_alt);

    // Serial.print("\tvxW:");
    // Serial.print(drone_state.vxW);
    // Serial.print("     vyW:");
    // Serial.print(drone_state.vyW);
    // Serial.print("     vzW:");
    // Serial.println(drone_state.vzW);

    // Serial.print("\taclFromIMU:");
    // Serial.print(aclFromIMU);
    // Serial.print("\tvelFromIMU:");
    // Serial.println(velFromIMU);

    // Serial.print("angular velocities");
    // Serial.print("     x:");
    // Serial.print(drone_state.gx);
    // Serial.print("     y:");
    // Serial.print(drone_state.gy);
    // Serial.print("     z:");
    // Serial.print(drone_state.gz);

    // Serial.print("     angulars:");
    // Serial.print("x:");
    // Serial.print(drone_state.roll_x);
    // Serial.print("     y:");
    // Serial.print(drone_state.pitch_y);
    // Serial.print("     z:");
    // Serial.println(drone_state.yaw_z);

    // Serial.print("xW:");
    // Serial.print(drone_state.xW,3);
    // Serial.print("     yW:");
    // Serial.println(drone_state.yW,3);
    // Serial.print("     zW:");
    // Serial.println(drone_state.zW,3);
    // Serial.print("     up:");
    // Serial.println(up);
    // Serial.print("     down:");
    // Serial.println(down);

    // int xWmm = trunc(drone_state.xW*1000);
    // int yWmm = trunc(drone_state.yW*1000);
    // int zWmm = trunc(drone_state.zW*1000);

    // Serial.print("xWmm:");
    // Serial.print(xWmm);
    // Serial.print("     yWmm:");
    // Serial.print(yWmm);
    // Serial.print("     zWmm:");
    // Serial.println(zWmm);

    sendPositionToControllerUART();
    // sendPositionToControllerI2C();

  }

  struct state
  {
  public:
    // linear
    float xW = 0;
    float yW = 0;
    float zW = 0;
    float dxW;        // расчетное
    float dyW;        // расчетное
    float dzW;        // расчетное
    float vxW = 0;
    float vyW = 0;
    float vzW = 0;
    float dvxW;       // расчетное
    float dvyW;       // расчетное
    float dvzW;       // расчетное
    float ax;         // получаем с датчика
    float ay;         // получаем с датчика
    float az;         // получаем с датчика
    float ax_prev = 0;
    float ay_prev = 0;
    float az_prev = 0;
    float axW;        // расчетное
    float ayW;        // расчетное
    float azW;        // расчетное
    
    // angular
    float roll_x  = 0;
    float pitch_y = 0;
    float yaw_z   = 0;
    float roll_x_from_ax_rad  = 0;
    float pitch_y_from_ay_rad = 0;
    float yaw_z_from_az_rad   = 0;

    float roll_x_from_Madgwick  = 0;
    float pitch_y_from_Madgwick = 0;
    float yaw_z_from_Madgwick   = 0;

    float droll_x;
    float dpitch_y;
    float dyaw_z;
    float gx;
    float gy;
    float gz;

    bool firstMadgwickMeasure = true;
    float offset_roll_x   = 0;
    float offset_pitch_y  = 0;
    float offset_yaw_z    = 0;

    float gps_lat = 0;
    float gps_lon = 0;
    float gps_alt = 0;
  };
  
private:
  Integrator* integrator_ax;
  Integrator* integrator_ay;
  Integrator* integrator_az;
  Integrator* integrator_vx;
  Integrator* integrator_vy;
  Integrator* integrator_vz;
  Integrator* integrator_gx;
  Integrator* integrator_gy;
  Integrator* integrator_gz;
  state       drone_state;

  float deg2rad(float x){
    return x * PI / 180;
  }

  float rad2deg(float x){
    return x * 180 / PI;
  }

  void calculate_aW(){

    float roll_x_rad  = deg2rad(drone_state.roll_x_from_Madgwick);
    float pitch_y_rad = deg2rad(drone_state.pitch_y_from_Madgwick);
    float yaw_z_rad   = deg2rad(drone_state.yaw_z_from_Madgwick);

    float sr = sin(roll_x_rad);
    float cr = cos(roll_x_rad);
    float sp = sin(pitch_y_rad);
    float cp = cos(pitch_y_rad);
    float sy = sin(yaw_z_rad);
    float cy = cos(yaw_z_rad);

    BLA::Matrix<3, 3> Rx = {1,  0,   0, 
                            0, cr, -sr, 
                            0, sr, cr};

    BLA::Matrix<3, 3> Ry = {cp,  0, sp, 
                             0,  1,  0, 
                           -sp,  0, cp};

    BLA::Matrix<3, 3> Rz = {cy, -sy, 0, 
                            sy,  cy, 0, 
                             0,   0, 1};

    BLA::Matrix<3, 3> R = Rz*Ry*Rx;     // Rx*Ry*Rz;      // Rz*Ry*Rx;

    BLA::Matrix<3> aL = {drone_state.ax,
                         drone_state.ay,
                         drone_state.az};

    BLA::Matrix<3> aW = R*aL;
    drone_state.axW = aW(0);
    drone_state.ayW = aW(1);
    drone_state.azW = aW(2);

    // Serial.print("roll_x_rad:");
    // Serial.print(roll_x_rad);
    // Serial.print("     pitch_y_rad:");
    // Serial.print(pitch_y_rad);
    // Serial.print("     yaw_z_rad:");
    // Serial.print(yaw_z_rad);
  }

  void do_offset_accelerations(){
    BLA::Matrix<4, 4> M_Rot_Trans = { a11, a12, a13, tx,
                                      a21, a22, a23, ty,
                                      a31, a32, a33, tz,
                                      ax,  ay,  az,  1};

    BLA::Matrix<4> input = {kx*drone_state.ax+bx,
                            ky*drone_state.ay+by,
                            kz*drone_state.az+bz,
                            1};

    BLA::Matrix<4> result = M_Rot_Trans*input;

    drone_state.ax = result(0);
    drone_state.ay = result(1);
    drone_state.az = result(2);
  }

  void sendPositionToControllerUART(){

    int axmm = trunc(drone_state.ax*1000);
    int aymm = trunc(drone_state.ay*1000);
    int azmm = trunc(drone_state.az*1000);

    long lat_from_gps = trunc(drone_state.gps_lat*1000000);
    long lon_from_gps = trunc(drone_state.gps_lon*1000000);
    long alt_from_gps = trunc(drone_state.gps_alt*1000000);

    // Отправляем три числа последовательно
    // Serial.write((uint8_t*)&axmm, sizeof(axmm));
    // Serial.write((uint8_t*)&aymm, sizeof(aymm));
    // Serial.write((uint8_t*)&azmm, sizeof(azmm));
    // Serial.write((uint8_t*)&lat_from_gps, sizeof(lat_from_gps));
    // Serial.write((uint8_t*)&lon_from_gps, sizeof(lon_from_gps));
    // Serial.write((uint8_t*)&alt_from_gps, sizeof(alt_from_gps));

    // Serial.print("\taxWmm:");
    // Serial.print(axWmm);
    // Serial.print("\tayWmm:");
    // Serial.print(ayWmm);
    // Serial.print("\tazWmm:");
    // Serial.println(azWmm);

    // Serial.print("\tlat_from_gps:");
    // Serial.print(lat_from_gps);
    // Serial.print("\tlon_from_gps:");
    // Serial.print(lon_from_gps);
    // Serial.print("\talt_from_gps:");
    // Serial.println(alt_from_gps);
  }

  void sendPositionToControllerI2C(){

    int xWmm = trunc(drone_state.xW*1000);
    int vxWmm = trunc(drone_state.vxW*1000);
    int axWmm = trunc(drone_state.axW*1000);

    Wire.beginTransmission(CONTROLLER);
    Wire.write((uint8_t*)&xWmm, sizeof(xWmm));
    Wire.write((uint8_t*)&vxWmm, sizeof(vxWmm));
    Wire.write((uint8_t*)&axWmm, sizeof(axWmm));
    Wire.endTransmission();

  }
};

// Drone state object
Drone_State drone;


void setup() {
  Serial.begin(230400);
  // Serial1.begin(9600);  // connect gps sensor
  // while(!Serial1) delay(10);
  while (Serial.available()) Serial.read();
  GY85.init();
  Wire.begin();
  filter.begin(update_freq);
}

void loop() {
  unsigned long start = millis(); 
  drone.update_state(1./update_freq);
  // Serial.print("     time [ms]: ");
  unsigned long end = millis();
  delay(1000/update_freq - (end - start));
  // Serial.println(millis() - start);
}