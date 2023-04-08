#ifndef Sensors_h
#define Sensors_h

#include <Wire.h>
#include "MUX2C.h"
#include "BNO.h"
#include "VLX.h"
#include "TCS.h"

// SENSOR DATA

#define vlx_right 1
#define vlx_left 2
#define vlx_front 0
#define vlx_back 3

// Class used to initialize, manage, and recieve information from all the needed sensors.
class Sensors
{
  // Sensors Count
  static constexpr int kBNOCount = 10;                                   // BNO sensor data count
  static constexpr int kVLXCount = 4;                                    // VLX sensors data count
  static constexpr int kTCSCount = 1;                                    // TCS sensors data count
  static constexpr int kSensorCount = kBNOCount + kVLXCount + kTCSCount; // Total sensors data count

  // TCS constants
  static constexpr int kTCSPrecision = 10;                   // Precision for matching color values.
  static constexpr uint8_t colorAmount = 2;                  // Number of colors
  static constexpr char colorList[colorAmount + 1] = {"NA"}; // List of color initials
  // RGB values for each color. Check TCS class for more details.
  static constexpr uint8_t colors[colorAmount][3] = {
      {137, 78, 58},
      {64, 85, 128}};

  bool usingVLX; // Used to decide if VLX will be initialized.
  bool usingBNO; // Used to decide if BNO will be initialized.

  // Sensors.
  BNO *bno;
  VLX vlx[3];
  TCS tcs;

  int kMuxVLX = 3;
  // Sensor Pins.

  // Front is in pin 1.
  // Left is in pin 0.
  // Right is in pin 7.
  int kMuxPins[3] = {1, 7, 0}; // VLX multiplexor pins
  int kMuxTCS = {2};           // TCS multiplexor pin

  // Limit Switches
  static constexpr uint8_t kDigitalPinsLimitSwitch[2] = {24, 25}; // Left, Right limit switches

  friend class GeneralChecks;

public:
  String vlxNames[3] = {"Front", "Left", "Right"};

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

  // Sensor Methods

  // Limit switches

  void debugLimitSwitches();
  int rightLimitSwitch();
  int leftLimitSwitch();

  void getLimitSwitches(int &right, int &left);

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

  // Prints sensor information in the serial monitor.
  // @param bno True to display bno angles.
  // @param vlx True to print vlx distances.
  // @param tcs True to print detected color.
  void printInfo(bool bno = true, bool vlx = true, bool tcs = true, bool limitSwitches = true);

  // Returns BNO angles through reference variables.
  void bnoAngles(float &x, float &y, float &z);

  void bnoPrint();

  // Make general checks to ensure TCS is working correctly.
  void checkTCS();
};

#endif
