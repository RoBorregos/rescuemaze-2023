#ifndef VLX_h
#define VLX_h

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include "MUX2C.h"

// Sensor de distancia.

class VLX{
  private:
    Adafruit_VL53L0X vlx = Adafruit_VL53L0X(); 
    MUX2C mux;
    uint8_t tcaPos; //POSICION EN MUX 
    VL53L0X_RangingMeasurementData_t measure;

    // Unit conversion
    const float kMm_in_M = 0.001;

  public:

    VLX();
    VLX(uint8_t posMux);

    void setMux(uint8_t posMux);
    double getDistance();
    float getRawDistance();
    void init();
    void printDistance();
};

#endif
