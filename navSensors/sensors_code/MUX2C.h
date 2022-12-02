#ifndef MUX2C_H
#define MUX2C_H

#include <Arduino.h>
#include <Wire.h>

// Multiplexor: Sirve para evitar problemas cuando dos dispositivos i2c
// tienen la misma dirección.
// Tutorial: https://www.youtube.com/watch?v=vV42fCpmCFg

// Dirección default de Multiplexor tca9548
#define TCAADDR 0x70

class MUX2C
{

private:
public:
  MUX2C();

  void encontrarI2C();

  void tcaSelect(uint8_t pos);
  
};

#endif
