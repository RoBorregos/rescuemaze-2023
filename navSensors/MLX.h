#ifndef MLX_H
#define MLX_H

#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include "MUX2C.h"

// Sensor de temperatura

class MLX
{
private:
  Adafruit_MLX90614 mlx;
  uint8_t tcaPos; // MUXposition
  MUX2C mux;

public:
  MLX(uint8_t posMux);
  MLX();

  void setMux(uint8_t posMux);
  void init();
  float getTemp();
  void printTemp();
};

#endif
