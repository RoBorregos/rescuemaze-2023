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
    Serial.println("TCS Error.");
  }
}

void TCS::init(const uint8_t colors[][3], const uint8_t colorAmount)
{
  this->colors = colors;
  this->colorAmount = colorAmount;
  init();
}

void TCS::init(const uint8_t colors[][3], const uint8_t colorAmount, const char colorList[])
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

char TCS::getColorKReps(int reps)
{
  char start = getColorWithPrecision();
  
  if (start == 'u')
    return start;

  // Confirm that the initial color received is stable.
  for (int i = 0; i < reps; i++)
  {
    char current = getColorWithPrecision();
    if (current != start)
      return 'u';
  }

  return start;
}

char TCS::getColorMode(int sampleSize, double certainity)
{
  int repetitions[colorAmount];

  for (int i = 0; i < colorAmount; i++)
  {
    repetitions[i] = 0;
  }

  for (int i = 0; i < sampleSize; i++)
  {
    char detection = getColorWithPrecision();
    for (int j = 0; j < colorAmount; j++)
    {
      if (colorList[j] == detection)
      {
        repetitions[j]++;
        break;
      }
    }
  }

  // Count the number of unknowns vs the number of mode.

  int mode = 0, unknown = sampleSize;

  for (int i = 0; i < colorAmount; i++)
  {
    if (repetitions[mode] < repetitions[i])
      mode = i;

    unknown -= repetitions[i];
  }

  double probability = repetitions[mode] / (double) sampleSize;

  if (repetitions[mode] > unknown && probability > certainity){
    return colorList[mode];
  }

  return 'u';
}

void TCS::printColorMatrix()
{
  if (colors == nullptr)
  {
    Serial.println("Color matrix is null.");
    return;
  }

  Serial.println("Printing color matrix:");

  for (uint8_t i = 0; i < colorAmount; i++)
  {
    Serial.println(String(colors[i][0]) + " " + String(colors[i][1]) + " " + String(colors[i][2]));
  }
  Serial.println(" ");
}

void TCS::printColorList()
{
  if (colorList == nullptr)
  {
    Serial.println("Color list is null.");
    return;
  }

  Serial.println("Printing color list:");

  for (uint8_t i = 0; i < colorAmount; i++)
  {
    Serial.print(colorList[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}
