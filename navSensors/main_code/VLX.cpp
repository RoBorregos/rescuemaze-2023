#include "VLX.h"

VLX::VLX()
{
}

VLX::VLX(uint8_t posMux)
{
  mux.setTcaPos(posMux);
}

void VLX::setMux(uint8_t posMux)
{
  mux.setTcaPos(posMux);
}

void VLX::init()
{
  mux.tcaSelect();

  if (!vlx.begin())
  {
    // if (!CK::kusingROS)
    //   Serial.println("ERROR VLX");
  }

  // vlx.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
}


/*

typedef enum {
    VL53L0X_SENSE_DEFAULT = 0,
    VL53L0X_SENSE_LONG_RANGE,
    VL53L0X_SENSE_HIGH_SPEED,
    VL53L0X_SENSE_HIGH_ACCURACY
  } VL53L0X_Sense_config_t;
*/

float VLX::getRawDistance()
{
  mux.tcaSelect();
  vlx.rangingTest(&measure, false);

  return measure.RangeMilliMeter;
}

double VLX::getDistance()
{
  mux.tcaSelect();
  vlx.rangingTest(&measure, false);

  double dist = measure.RangeMilliMeter / 1000.000;

  /* Ignore values greater than 3 meters.
  if (dist > 3)
  {
    dist = prevDist;
  }
  else
  {
    prevDist = dist;
  }*/
  // Sometimes vlx detects long distances as 0. In this case, replace value with max possible.
  return dist;
}

void VLX::printDistance()
{
  if (CK::kusingROS)
    return;

  // Serial.print("Distancia: ");
  // Serial.print(VLX::getDistance());
  // Serial.println(" M");
}
