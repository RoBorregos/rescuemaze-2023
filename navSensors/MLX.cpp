#include "MLX.h"

MLX::MLX(uint8_t posMux){
  tcaPos_ = posMux;
  init();
}

MLX::MLX(){
  tcaPos_ = 0;
  init();
}

// Regresa la temperatura obtenida.
float MLX::getTemp(){
  mux_.tcaSelect(tcaPos_);
  return mlx_.readObjectTempC();
}

void MLX::init(){
  mux_.tcaSelect(tcaPos_);
  if(!mlx_.begin()){
    Serial.println("ERROR");
  }
}

void MLX::setMux(uint8_t posMux){
  tcaPos_ = posMux;
}

void MLX::printTemp(){
  Serial.print("Temperatura (C): ");
  Serial.println(MLX::getTemp());
}
