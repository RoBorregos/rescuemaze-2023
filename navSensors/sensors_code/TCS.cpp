#include "TCS.h"

TCS::TCS()
{
  setDefValues();
}

TCS::TCS(uint8_t posMux)
{
  setDefValues();
  mux.setTcaPos(posMux);
}

TCS::TCS(uint8_t posMux, uint8_t precision)
{
  setDefValues();
  mux.setTcaPos(posMux);
  this->precision = precision;
}

void TCS::init()
{
  mux.tcaSelect();
  if (!tcs.begin())
  {
    Serial.println("ERROR TCS.");
    mux.setChannel(TCS_ADDR);
  }
}

void TCS::init(uint8_t colors[][3], uint8_t colorAmount)
{
  this->colors = colors;
  this->colorAmount = colorAmount;
  init();
}

void TCS::init(uint8_t colors[][3], uint8_t colorAmount, char colorList[])
{
  this->colorList = colorList;
  init(colors, colorAmount);
}

void TCS::setDefValues()
{
  red = 0;
  green = 0;
  blue = 0;

  char tempC[4] = {"wgb"};
  colorList = tempC;
  colors = nullptr;
  precision = 10;
}

void TCS::updateRGB()
{
  mux.tcaSelect();
  tcs.setInterrupt(false); // turn on LED
  delay(50);               // 50ms recomendado, testear en 10
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true); // turn off LED
}

void TCS::printRGB()
{
  updateRGB();

  Serial.print("R: ");
  Serial.print(red);
  Serial.print("  G: ");
  Serial.print(green);
  Serial.print("  B: ");
  Serial.println(blue);
}

void TCS::printColor()
{
  Serial.print("Color: ");
  char color = (colors) ? getColorWithPrecision() : getColor();
  Serial.println(color);
}

void TCS::setMux(uint8_t posMux)
{
  mux.setTcaPos(posMux);
}

void TCS::setPrecision(uint8_t precision)
{
  this->precision = precision;
}

char TCS::getColor()
{
  updateRGB();
  char color_letter;
  if (red > 80 && green > 87 && blue > 82)
  {
    color_letter = 'w';
  }
  else if (red > 89 && green > 86 && blue < 80)
  {
    color_letter = 'g';
  }
  else if (red < 110 && green > 80 && blue > 70)
  {
    color_letter = 'b';
  }
  return color_letter;
}

bool TCS::inRange(uint8_t input, uint8_t registered)
{
  return (registered - precision <= input && input <= registered + precision);
}

char TCS::getColorWithPrecision()
{
  if (colors == nullptr)
  {
    Serial.println("The colors aren't declared, getColor() will be used.");
    return getColor();
  }

  updateRGB();

  for (uint8_t i = 0; i < colorAmount; i++)
  {
    if (inRange(red, colors[i][0]) && inRange(green, colors[i][1]) && inRange(blue, colors[i][2]))
    {
      return colorList[i];
    }
  }

  // In case no color is detected.
  return 'u';
}
