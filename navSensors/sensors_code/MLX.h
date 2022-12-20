#ifndef MLX_H
#define MLX_H

#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include "MUX2C.h"

// Temperature sensor

#define MLX_ADDR 30

class MLX
{
private:
  Adafruit_MLX90614 mlx;
  MUX2C mux;

public:
  // Creates object with MUX position.
  // @param posMux multiplexor position.
  MLX(uint8_t posMux);

  MLX();

  // Sets MUX position.
  // @param posMux new mutliplexor position.
  void setMux(uint8_t posMux);

  // Calls .begin() for MLX sensor.
  void init();

  // Returns obtained temperature in Celsius.
  float getTemp();

  // Prints temperature in Celsius to serial monitor.
  void printTemp();
};

#endif
