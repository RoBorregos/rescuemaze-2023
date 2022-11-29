#ifndef MLX_H
#define MLX_H

#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include "MUX2C.h"

class MLX
{
private:
  Adafruit_MLX90614 mlx_;
  uint8_t tcaPos_; // MUXposition
  MUX2C mux_;

public:
  MLX(uint8_t posMux);
  MLX();

  void setMux(uint8_t posMux);
  void init();
  float getTemp();
  void printTemp();
};

#endif
