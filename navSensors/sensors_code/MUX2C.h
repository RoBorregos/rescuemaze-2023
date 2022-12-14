#ifndef MUX2C_H
#define MUX2C_H

#include <Arduino.h>
#include <Wire.h>

// Multiplexor: Used to select a device when there are several I2C devices
// with the same address.
// Tutorial: https://www.youtube.com/watch?v=vV42fCpmCFg

// Default address of multiplexor tca9548
#define TCAADDR 0x70

class MUX2C
{

private:
public:
  MUX2C();

  // Finds the I2C addresses of the connected devices
  // at any of the channels.
  void findI2C();

  // Selects one of the channels of the multiplexor.
  // @param pos The selected channel. Range: [0,7]
  void tcaSelect(uint8_t pos);
};

#endif
