#include "MUX2C.h"

MUX2C::MUX2C()
{
}

// Selecciona uno de los canales del multiplexor para que pueda usarse.
// @param i Canal que se quiere usar. Rango: [0,7]
void MUX2C::tcaSelect(uint8_t i)
{
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Encuentra las direcciones i2c de los dispostivos conectados
// en cualquiera de los canales del MUX.
void MUX2C::encontrarI2C()
{
  Wire.begin();
  Serial.println("\nTCA escaner listo");

  for (uint8_t t = 0; t < 8; t++)
  {
    tcaSelect(t);
    Serial.print("Escaneando salida ");
    Serial.println(t);
    for (uint8_t addr = 0; addr <= 127; addr++)
    {
      if (addr == TCAADDR)
        continue;
      Wire.beginTransmission(addr);
      if (!Wire.endTransmission())
      {
        Serial.print("  - Encontrado I2C 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("Finalizado");
}
