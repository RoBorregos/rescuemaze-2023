#ifndef TCS_H
#define TCS_H

#include "Adafruit_TCS34725.h"
#include "MUX2C.h"
#include "CommonK.h"

#define TCS_ADDR 0x29

// Color sensor

class TCS
{
  friend class GeneralChecks;

private:
  // Test with different integration times (2.4ms, 24ms, 50ms, 101ms, 154ms, 700ms). Note: integration time increases detection time.
  // Test with different gain values (1x, 4x, 16x, 60x)
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
  float red;
  float green;
  float blue;

  MUX2C mux;
  int precision;
  const char *colorList; // Array with color initials.
  uint8_t colorAmount = 3;

  // Matrix with colors detected by sensor
  // rows: amount of colors
  // columns:  R G B value registered for each color
  const int (*colors)[3];

  /*
  for color at colorList[i],
    colors[i][0] = amount of red.
    colors[i][1] = amount of green.
    colors[i][2] = amount of blue.
  registered during manual color calibration.
  */

  const int (*colorThresholds)[6];

  /*
  for color at colorList[i],
    colors[i][0] = minRed.
    colors[i][1] = MaxRed.
    colors[i][2] = minGreen.
    colors[i][3] = maxGreen.
    colors[i][4] = minBlue.
    colors[i][5] = maxBlue.
  registered during manual color calibration.
 */

  // Checks if detected color is within range.
  bool inRange(double color, double colorRegistered);

  bool inRangeThreshold(double lowerBound, double colorDetection, double upperBound);

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
  void init(const int colors[][3], const uint8_t colorAmount);

  // Sets colors, colorAmount and colorList for TCS object.
  // @param colors[][3] 2D array with RGB values.
  // @param colorAmount Number of colors registered in colors[][]; rows in colors array.
  // @param colorList[] Array of initials of registered colors.
  void init(const int colors[][3], const uint8_t colorAmount, const char colorList[]);

  // Sets colors, colorAmount and colorList for TCS object.
  // @param colors[][3] 2D array with RGB values.
  // @param colorAmount Number of colors registered in colors[][]; rows in colors array.
  // @param colorList[] Array of initials of registered colors.
  // @param colorThresholds[] Array of thresholds for detecting each color.
  void init(const int colors[][3], const uint8_t colorAmount, const char colorList[], const int colorThresholds[][6]);

  // Sets MUX position.
  // @param posMux new mutliplexor position.
  void setMux(uint8_t posMux);

  // Modifies the precision used for color detection.
  // @param precision Represents how greater or lower a detected color can be to considered as matching.
  void setPrecision(uint8_t precision);

  // Prints the RGB values of the sensor
  void printRGB();

  // Prints the RGBC values of the sensor
  void printRGBC();

  // Prints the color detected by the sensor.
  void printColor();

  // Updates the RGB values of the sensor
  void updateRGB();

  // Updates the RGB values but includes the Clear section.
  void updateRGBC();

  // Returns the color letter that the sensor detected
  char getColor();

  // Returns color letter detected with the use of precision ranges.
  char getColorWithPrecision();

  // Returns color letter detected with the use of precision ranges.
  char getColorWithThresholds();

  // Return the mode obtained after sampleSize detections, if the probability is above
  // the specified threshold.
  char getColorMode(int sampleSize, double certainity = 0);

  // Calls getColorWithPrecision k times at most. If the initial reading differs from
  // any of the subsequent k calls, the function returns 'u'. Otherwise, it returns the start character.
  char getColorKReps(int reps);

  // Prints the colors matrix to check if the colors were saved successfully.
  void printColorMatrix();

  // Prints the colors list to check if the colors were saved successfully.
  void printColorList();
};

#endif
