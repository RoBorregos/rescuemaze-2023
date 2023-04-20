#ifndef BNO_h
#define BNO_h

#include <Wire.h>
#include "MUX2C.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "CommonK.h"
#include <EEPROM.h>

#define BNO_ADDR 0x28

// Sensor with gyroscope, accelerometer, and magnetometer
class BNO
{
  friend class GeneralChecks;

private:
  Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
  sensors_event_t angVelocityData, linearAccelData;
  imu::Quaternion quat;

  float yaw_ = 0.0;
  float yaw_vel_ = 0.0;
  float x_accel = 0.0;
  float y_accel = 0.0;
  float z_accel = 0.0;

public:
  BNO();

  void init();

  void displaySensorDetails(void);
  void displaySensorStatus(void);
  void displayCalStatus(void);
  bool isCalibrated();
  void restoreCalibration();

  // Events with quaternion.
  void updateEvents();
  float getQuat_x();
  float getQuat_y();
  float getQuat_z();
  float getQuat_w();
  float getAngVel_x();
  float getAngVel_y();
  float getAngVel_z();
  float getLinAcc_x();
  float getLinAcc_y();
  float getLinAcc_z();

  // Events with Euler angles.
  void updateBNO();
  float getYaw();
  float getYawVel();
  float getXAccel();
  float getYAccel();
  float getZAccel();

  void anglesInfo();
  void setExtCUse();
  void getAll(float &x, float &y, float &z);
  float getAngleX();
  float getAngleY();
  float getAngleZ();
};

#endif
