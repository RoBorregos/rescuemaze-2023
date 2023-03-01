#ifndef Sensors_h
#define Sensors_h

#include <Wire.h>
#include "MUX2C.h"
#include "BNO.h"
#include "VLX.h"
#include "TCS.h"

// SENSOR DATA

#define vlx_right 0
#define vlx_left 1
#define vlx_front 2
#define vlx_back 3

// Class used to initialize, manage, and recieve information from all the needed sensors.
class Sensors
{
  // Sensors Count
  static constexpr int kBNOCount = 10;                                               // BNO sensor data count
  static constexpr int kVLXCount = 4;                                                // VLX sensors data count
  static constexpr int kTCSCount = 1;                                                // TCS sensors data count
  static constexpr int kSensorCount = kBNOCount + kVLXCount + kTCSCount;             // Total sensors data count

  // TCS constants
  static constexpr int kTCSPrecision = 10; // Precision for matching color values.
  static constexpr uint8_t colorAmount = 7; // Number of colors
  static constexpr char colorList[colorAmount + 1] = {"wgbgbnr"};   // List of color initials
  // RGB values for each color. Check TCS class for more details.
  static constexpr uint8_t colors[colorAmount][3] = {
      {19, 31, 58},
      {28, 120, 82},
      {16, 64, 31},
      {73, 132, 73},
      {110, 70, 108},
      {155, 111, 57},
      {220, 156, 150}};

  // Sensors.
  
  BNO *bno;
  VLX vlx[kVLXCount];
  TCS tcs;

  // Sensor Pins.
  int kMuxVLX[kVLXCount] = {1, 7, 5, 3}; // VLX multiplexor pins
  int kMuxTCS = 6;                       // TCS multiplexor pin

public:
  // Constructor

  Sensors(BNO *bno);

  // Initialization

  void initSensors();

  // Sensor Methods

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

  // Prints sensor information in the serial monitor.
  // @param bno True to display bno angles.
  // @param vlx True to print vlx distances.
  // @param tcs True to print detected color.
  void printInfo(bool bno = true, bool vlx = true, bool tcs = true);

  // Returns BNO angles through reference variables.
  void bnoAngles(float &x, float &y, float &z);

  void bnoPrint();

  void checkTCS();
};

#endif
