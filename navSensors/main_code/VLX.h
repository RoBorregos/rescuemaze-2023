#ifndef VLX_h
#define VLX_h

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include "MUX2C.h"
#include "CommonK.h" 

// Distance sensor

#define VLX_ADDR 0x29

class VLX
{
  friend class GeneralChecks;

private:
  Adafruit_VL53L0X vlx = Adafruit_VL53L0X();
  MUX2C mux;
  VL53L0X_RangingMeasurementData_t measure;

  // Unit conversion
  const float kMm_in_M = 0.001;

public:
  VLX();

  VLX(uint8_t posMux);

  void setMux(uint8_t posMux);

  // Retuns distance in meters.
  double getDistance();

  // Returns distance in millimeters.
  float getRawDistance();
  void init();

  // Prints distance in meters.
  void printDistance();
};

#endif
