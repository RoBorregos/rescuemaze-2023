#ifndef BNO_h
#define BNO_h

#include <Wire.h>
#include "MUX2C.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNOADDR 0x28

// Sensor con giroscopio, acelerometro y magnetometro.

class BNO{
  private:
    Adafruit_BNO055 bno = Adafruit_BNO055(55, BNOADDR); 
    sensors_event_t angVelocityData, linearAccelData;
    imu::Quaternion quat;
    MUX2C mux;
    uint8_t tcaPos; 
  public:

    BNO();
    BNO(uint8_t posMux);
    
    void init();
    void setMux(uint8_t posMux);

    void displaySensorDetails(void);
    void displaySensorStatus(void);

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

    void anglesInfo();
    float getAngleX();
    float getAngleY();
    float getAngleZ();
};

#endif
