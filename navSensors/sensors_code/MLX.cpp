#include "MLX.h"

MLX::MLX(uint8_t posMux){
  tcaPos = posMux;
}

MLX::MLX(){
  tcaPos = 0;
}

float MLX::getTemp(){
  mux.tcaSelect(tcaPos);
  return mlx.readObjectTempC();
}

void MLX::init(){
  mux.tcaSelect(tcaPos);
  if(!mlx.begin()){
    Serial.println("ERROR MLX");
  }
}

void MLX::setMux(uint8_t posMux){
  tcaPos = posMux;
}

void MLX::printTemp(){
  Serial.print("Temperature (C): ");
  Serial.println(MLX::getTemp());
}
