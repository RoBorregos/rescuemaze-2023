#include "VLX.h"


VLX::VLX(){
  tcaPos = 0;
}

VLX::VLX(uint8_t posMux){
  tcaPos = posMux;
}

void VLX::setMux(uint8_t posMux){
  tcaPos = posMux;
}

void VLX::init(){
  mux_.tcaSelect(tcaPos);
    
  if (!vlx.begin()){
    Serial.println("ERROR VLX");
  }
}

float VLX::getRawDistance(){
  mux.tcaSelect(tcaPos);
  vlx.rangingTest(&measure, false);

  return measure.RangeMilliMeter; // Devuelve la distancia en milimetros
}

// Regresa distanica en metros
double VLX::getDistance(){
  mux.tcaSelect(tcaPos);
  vlx.rangingTest(&measure, false);
  
  
  return (measure.RangeMilliMeter/1000.000);
}

void VLX::printDistance(){
  Serial.print("Distancia: ");
  Serial.print(VLX::getDistance());
  Serial.println(" M");
}
