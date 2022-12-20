#include "MLX.h"

MLX::MLX()
{
}

MLX::MLX(uint8_t posMux)
{
  mux.setTcaPos(posMux);
}

float MLX::getTemp()
{
  mux.tcaSelect();
  return mlx.readObjectTempC();
}

void MLX::init()
{
  mux.tcaSelect();
  if (!mlx.begin())
  {
    Serial.println("ERROR MLX");
    mux.setChannel(MLX_ADDR);
  }
}

void MLX::setMux(uint8_t posMux)
{
  mux.setTcaPos(posMux);
}

void MLX::printTemp()
{
  Serial.print("Temperature (C): ");
  Serial.println(MLX::getTemp());
}
