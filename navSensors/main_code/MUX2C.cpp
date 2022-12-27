#include "MUX2C.h"

MUX2C::MUX2C()
{
  tcaPos = 0;
}

void MUX2C::setTcaPos(uint8_t tcaPos)
{
  this->tcaPos = tcaPos;
}

void MUX2C::tcaSelect(uint8_t tcaPos)
{
  setTcaPos(tcaPos);
  tcaSelect();
}

void MUX2C::tcaSelect()
{
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << tcaPos);
  Wire.endTransmission();
}

void MUX2C::findI2C(bool scan, uint8_t address)
{
  Wire.begin();
  
  if (scan)
    Serial.println("\nScanning I2C positions");

  for (uint8_t t = 0; t < 8; t++)
  {
    tcaSelect(t);
    if (scan)
    {
      Serial.print("Scanning ");
      Serial.println(t);
    }

    for (uint8_t addr = 0; addr <= 127; addr++)
    {
      if (addr == TCA_ADDR)
        continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission())
      {
        if (scan)
        {
          Serial.print("  - I2C found at 0x");
          Serial.println(addr, HEX);
        }
        else if (addr == address)
        {
          return; // The last tcaSelect(t) would set the matching channel.
        }
      }
    }
  }
  if (scan)
    Serial.println("End of I2C scan");
  else
    tcaPos = 8; // Used to detect errors since valid range is [0-7].
}

void MUX2C::setMatching(uint8_t address)
{
  findI2C(false, address);
}

void MUX2C::setChannel(uint8_t address){
  Serial.println("Searching for channel...");
    setMatching(address);

    uint8_t channel = getTcaPos();

    if (channel == 8)
    {
      Serial.println("Channel not found. Check connections");
    }
    else
    {
      Serial.print("Channel ");
      Serial.print(channel);
      Serial.println(" found and set.");
      Serial.println("Warning: manually set channels because automatic selection may cause errors ");
      Serial.println("in case 2 devices have the same I2C address.");
    }
}


uint8_t MUX2C::getTcaPos()
{
  return tcaPos;
}
