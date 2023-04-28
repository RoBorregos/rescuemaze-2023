#ifndef Sensors_h
#define Sensors_h

#include <Wire.h>
#include "MUX2C.h"
#include "BNO.h"
#include "VLX.h"
#include "TCS.h"
#include "Screen.h"
#include "CommonK.h"
// #include "RosBridge.h"
#include "RosBridge2.h"

// SENSOR DATA

#define vlx_right 1
#define vlx_left 2
#define vlx_front 0
#define vlx_back 3

#define dist_front 0
#define dist_back 1
#define dist_left 2
#define dist_right 3

class RosBridge2; // Forward declaration of RosBridge

// Class used to initialize, manage, and recieve information from all the needed sensors.
class Sensors
{
  friend class GeneralChecks;

  // Sensors Count
  static constexpr int kBNOCount = 10;                                   // BNO sensor data count
  static constexpr int kVLXCount = 4;                                    // VLX sensors data count
  static constexpr int kTCSCount = 1;                                    // TCS sensors data count
  static constexpr int kSensorCount = kBNOCount + kVLXCount + kTCSCount; // Total sensors data count

  // TCS constants
  static constexpr int kTCSPrecision = 100;                  // Precision for matching color values.
  static constexpr uint8_t colorAmount = 3;                  // Number of colors
  static constexpr char colorList[colorAmount + 1] = {"NAG"}; // List of color initials
  // RGB values for each color. Check TCS class for more details.
  static constexpr int colors[colorAmount][3] = {
      {100, 100, 100},
      {300, 500, 550}};

  // Each row represents the upper and lower limits for detecting a color.
  // colorThresholds[0] = {redMin, redmax, greenMin, greenMax, blueMin, blueMax}
  static constexpr int colorThresholds[colorAmount][6] = {
      {0, 30, 0, 20, 0, 15},
      {20, 30, 20, 25, 26, 30},
      {180, 240, 190, 210, 180, 210}};

  bool usingVLX; // Used to decide if VLX will be initialized.
  bool usingBNO; // Used to decide if BNO will be initialized.

  long int maxBNOTime = 10000;    // Max time for BNO to calibrate.
  double timeToPlaceRobot = 6000; // Time to place robot after BNO calibration.

  // Sensors.
  BNO *bno;
  RosBridge2 *rosBridge = nullptr;
  VLX vlx[3];
  TCS tcs;
  Screen screen;
  bool rightLedOn = false;
  bool leftLedOn = false;

  int lidarAttemptCount = 0;

  int kMuxVLX = 3;
  // Sensor Pins.

  // Front is in pin 7.
  // Right is in pin 0.
  // Left is in pin 1.
  /*
  #define vlx_right 1
  #define vlx_left 2
  #define vlx_front 0
*/
  int kMuxPins[3] = {7, 0, 1}; // VLX multiplexor pins
  int kMuxTCS = {2};           // TCS multiplexor pin

  // Limit Switches
  static constexpr uint8_t kDigitalPinsLimitSwitch[2] = {24, 25}; // Left, Right limit switches
  static constexpr uint8_t kMotorPin = 23;
  // Leds
  static constexpr uint8_t kDigitalPinsLEDS[2] = {41, 42};

public:
  // Lidar distances
  float wallDistances[4] = {0, 0, 0, 0}; // front, back, left, right
  String vlxNames[3] = {"Front", "Right", "Left"};
  bool usingLidar = true;

  // Constructor

  // Constuctor for using sensors with BNO connected to arduino.
  // @param *bno pointer to BNO object.
  // @param usingVLX if true, vlx will be initialized.
  Sensors(BNO *bno, bool usingVLX = true);

  // Constuctor for using sensors with external BNO.
  // @param usingVLX if true, vlx will be initialized.
  Sensors(bool usingVLX = true);

  // Initialization

  void initSensors();

  void initSwitches();

  void setRosBridge(RosBridge2 *rosBridge);

  // Sensor Methods

  // Limit switches

  void debugLimitSwitches(); // Prints limit switches data.
  bool rightLimitSwitch();
  bool leftLimitSwitch();
  void getLimitSwitches(bool &right, bool &left); // Returns limit switches data.

  // Initializes indicator leds.
  void initLeds();

  // LED methods
  void toggleRightLed();
  void toggleLeftLed();
  void toggleBothLeds();
  void turnRightLedOn();
  void turnRightLedOff();

  void bothLedOn();
  void bothLedOff();

  // Oled methods
  void logActive(String s, bool oled = true, int x = 0, int y = 0, bool absolute = false);
  void logActive(String s, double n, String divider = ": ", bool oled = true, int x = 0, int y = 0);
  void logActive(double n, String s, String divider = ": ", bool oled = true, int x = 0, int y = 0);

  // BNO data

  // Returns the BNO x axis data for quatertion.
  float getQuatX();

  // Returns the BNO y axis data for quatertion.
  float getQuatY();

  // Returns the BNO z axis data for quatertion.
  float getQuatZ();

  // Returns the BNO w axis data for quatertion.
  float getQuatW();

  // Returns the BNO x axis data for angular velocity.
  float getAngVelX();

  // Returns the BNO y axis data for angular velocity.
  float getAngVelY();

  // Returns the BNO z axis data for angular velocity.
  float getAngVelZ();

  // Returns the BNO x axis data for Linear acceleration.
  float getLinAccX();

  // Returns the BNO y axis data for Linear acceleration.
  float getLinAccY();

  // Returns the BNO z axis data for Linear acceleration.
  float getLinAccZ();

  // Returns the BNO x axis data for angle.
  float getAngleX();

  // Returns the BNO y axis data for angle.
  float getAngleY();

  // Returns the BNO z axis data for angle.
  float getAngleZ();

  // Returns the VLX distance depending on the position of the VLX sensor
  float getVLXInfo(int posVLX);

  // Returns the TCS color detected
  char getTCSInfo();

  // Print the rgb values detected by the tcs.
  void rgbTCS();

  // Print the rgbc values detected by the tcs.
  void rgbTCSClear();

  void updateDistLidar(float front, float back, float left, float right);

  void updateDistLidar(float front);

  // Prints sensor information in the serial monitor.
  // @param bno True to display bno angles.
  // @param vlx True to print vlx distances.
  // @param tcs True to print detected color.
  void printInfo(bool bno = true, bool vlx = true, bool tcs = true, bool limitSwitches = true);

  // Returns BNO angles through reference variables.
  void bnoAngles(float &x, float &y, float &z);

  void bnoPrint();

  void getLidarDistances(double &front, double &back, double &left, double &right);

  // Reads if motors are turned on.
  bool readMotorInit();

  // Make general checks to ensure TCS is working correctly.
  void checkTCS();

  // Use function as toplevel for distances. Decides between using lidar
  // or vlx. If lidar fails repeatedly, it will switch to vlx. (back not available for vlx)
  // @param direction 0 for front, 1 for back, 2 for left, 3 for right.
  float getDistInfo(int direction);

  bool isValid(double d);

  void resetScreen();
};

#endif
