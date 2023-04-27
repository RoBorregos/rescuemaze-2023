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

  static constexpr int kLeftMovement = 0;
  static constexpr int kStopMovement = 90;
  static constexpr int kRightMovement = 180;

  static constexpr int rightDelay = 2000; 
  static constexpr int leftDelay = 200;

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
};

#endif
