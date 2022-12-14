#include "MUX2C.h"

MUX2C::MUX2C()
{
}

void MUX2C::tcaSelect(uint8_t i)
{
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void MUX2C::findI2C()
{
  Wire.begin();
  Serial.println("\nScanning I2C positions");

  for (uint8_t t = 0; t < 8; t++)
  {
    tcaSelect(t);
    Serial.print("Scanning ");
    Serial.println(t);
    for (uint8_t addr = 0; addr <= 127; addr++)
    {
      if (addr == TCAADDR)
        continue;
      Wire.beginTransmission(addr);
      if (!Wire.endTransmission())
      {
        Serial.print("  - I2C found at 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("End of I2C scan");
}
