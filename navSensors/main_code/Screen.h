#ifndef SCREEN_h
#define SCREEN_h

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c // initialize with the I2C addr 0x3C Typically eBay OLED's
// #define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    //   QT-PY / XIAO

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH 16

// Sensor with gyroscope, accelerometer, and magnetometer
class Screen
{
    friend class GeneralChecks;

private:
    Adafruit_SH1106G screen = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    

public:
    Screen();

    void init();

    void display(String message, int x=0, int y=0);

    void display(double num, String message, String divider=": ", int x=0, int y=0);
    void display(String message, double num, String divider=": ", int x=0, int y=0);

    void resetLine(int line);
    void resetScreen();
    void setStyle();
    void testdraw();
};

#endif
