#ifndef Dispenser_h
#define Dispenser_h

#include <Servo.h>
#include <Arduino.h>

#define right 1
#define left -1

// Class used to control kit dispensers.

class Dispenser
{
private:
  Servo dispenser;

  uint8_t servoPin;

  // Single Dispenser

  static constexpr int kLeftMovement = 0;
  static constexpr int kStopMovement = 90;
  static constexpr int kRightMovement = 180;

  static constexpr int rightDelay = 20; 
  static constexpr int leftDelay = 20;

public:
  // Constructor

  Dispenser();
  Dispenser(uint8_t servoPin);

  // Initialization

  void initServo();

  // Dispenser Functions

  // Drops the kit (decider dispenser) to the right
  void rightDrop();

  // Drops the kit (decider dispenser) to the left
  void leftDrop();
};

#endif
