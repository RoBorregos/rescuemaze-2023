#ifndef MUX2C_H
#define MUX2C_H

#include <Arduino.h>
#include <Wire.h>

// Multiplexor: Used to select a device when there are several I2C devices
// with the same address.
// Tutorial: https://www.youtube.com/watch?v=vV42fCpmCFg

// Default address of multiplexor tca9548
#define TCA_ADDR 0x70

class MUX2C
{

private:
  uint8_t tcaPos;

public:
  MUX2C();

  // Finds the I2C addresses of the connected devices
  // at any of the channels.
  // @param scan True to print all I2C addresses. False to use as helper method
  // for setMatching().
  // @param address The address when using as helper function for setMatching().
  void findI2C(bool scan = true, uint8_t address = 0);

  // Selects one of the channels of the multiplexor and registers it as
  // the new default channel.
  // @param tcaPos The selected channel. Range: [0,7]
  void tcaSelect(uint8_t tcaPos);

  // Getter for tcaPos
  uint8_t getTcaPos();

  // Selects the registered channel of the multiplexor.
  void tcaSelect();

  // Registers a new channel as the default channel for selections.
  // @param tcaPos The selected channel. Range: [0,7]
  void setTcaPos(uint8_t tcaPos);

  // Searches for the given address among the channels and registers the
  // channel containing the address as the new one. It is better to enter
  // address manually because 2 sensors may have the same address.
  // @param address The sensor's address.
  void setMatching(uint8_t address);
  
  // Searches and sets an available channel. Same functionaly as setMatching()
  // but prints debug messages.
  // @param address The sensor's address.
  void setChannel(uint8_t address);

};

#endif
