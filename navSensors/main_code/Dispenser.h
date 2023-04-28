#ifndef Dispenser_h
#define Dispenser_h

#include <Servo.h>
#include <Arduino.h>
#include "CommonK.h"

// Class used to control kit dispensers.

class Dispenser
{
  friend class GeneralChecks;
private:
  Servo dispenser;

  uint8_t servoPin;

  // Single Dispenser

  static constexpr int kleftDrop = 0;
  static constexpr int krightDrop = 0;
  static constexpr int kstartSpot = 90;
  static constexpr int kgetKit = 180;

  static constexpr int dispenserDelay = 500; 
  // static constexpr int leftDelay = 200;

public:
  // Constructor

  Dispenser();
  Dispenser(uint8_t servoPin);

  // Initialization

  void initServo();

  void stop();

  // Dispenser Functions

  // Drops the kit (decider dispenser) to the right
  void rightDrop();

  // Drops the kit (decider dispenser) to the left
  void leftDrop();

  void dropNKits(int kits);
  
  void write(int angle);
};

#endif
