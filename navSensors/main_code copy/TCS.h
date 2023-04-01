#ifndef TCS_H
#define TCS_H

#include "Adafruit_TCS34725.h"
#include "MUX2C.h"

#define TCS_ADDR 0x29

// Color sensor

class TCS
{
private:
  Adafruit_TCS34725 tcs;
  float red;
  float green;
  float blue;
  MUX2C mux;
  uint8_t precision;
  const char *colorList; // Array with color initials. 
  uint8_t colorAmount = 3;

  // Matrix with colors detected by sensor
  // rows: amount of colors
  // columns:  R G B value registered for each color
  const uint8_t (*colors)[3];
  
  /*
  for color at colorList[i], 
    colors[i][0] = amount of red.
    colors[i][1] = amount of green.
    colors[i][2] = amount of blue.
  registered during manual color calibration.
  */

  // Checks if detected color is within range.
  bool inRange(uint8_t color, uint8_t colorRegistered);

  void setDefValues();

public:
  TCS();
  TCS(uint8_t posMux);
  TCS(uint8_t posMux, uint8_t precision);

  // Calls .begin() for the TCS object.
  void init();

  // Methods used to set colors, colorAmount, and colorList after object declaration.

  // Sets colors and colorAmount for TCS object.
  // @param colors[][3] 2D array with RGB values.
  // @param colorAmount Number of colors registered in colors[][]; rows in colors array.
  void init(const uint8_t colors[][3], const uint8_t colorAmount);

  // Sets colors, colorAmount and colorList for TCS object.
  // @param colors[][3] 2D array with RGB values.
  // @param colorAmount Number of colors registered in colors[][]; rows in colors array.
  // @param colorList[] Array of initials of registered colors.
  void init(const uint8_t colors[][3], const uint8_t colorAmount, const char colorList[]);

  // Sets MUX position.
  // @param posMux new mutliplexor position.
  void setMux(uint8_t posMux);

  // Modifies the precision used for color detection.
  // @param precision Represents how greater or lower a detected color can be to considered as matching.
  void setPrecision(uint8_t precision);

  // Prints the RGB values of the sensor
  void printRGB();
  
  // Prints the color detected by the sensor.
  void printColor();

  // Updates the RGB values of the sensor
  void updateRGB();

  // Returns the color letter that the sensor detected
  char getColor();

  // Returns color letter detected with the use of precision ranges.
  char getColorWithPrecision();

  // Prints the colors matrix to check if the colors were saved successfully.
  void printColorMatrix();

  // Prints the colors list to check if the colors were saved successfully.
  void printColorList();
};

#endif
