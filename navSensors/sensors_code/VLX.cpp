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
    Serial.println("ERROR VLX");
    mux.setChannel(VLX_ADDR);
  }
}

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

  return (measure.RangeMilliMeter / 1000.000);
}

void VLX::printDistance()
{
  Serial.print("Distancia: ");
  Serial.print(VLX::getDistance());
  Serial.println(" M");
}
