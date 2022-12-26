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

  static constexpr int kInitAngle = 0;
  static constexpr int kDropAngle = 180;

  // Decider dispenser

  static constexpr int kMidAngle = 90;
  static constexpr int kRightAngle = 0;  // To be determined
  static constexpr int kLeftAngle = 179; // To be determined

  static constexpr int kTime2Drop = 20; // Time needed for servo to drop kit
public:
  // Constructor

  Dispenser();
  Dispenser(uint8_t servoPin);

  // Initialization

  void initServo();

  // Dispenser Functions

  // Restarts the dispenser to the initial position
  void restart();

  // Drops the kit (non-decider dispenser)
  void kitDrop();

  // Drops the kit (decider dispenser) to the right
  void rightDrop();

  // Drops the kit (decider dispenser) to the left
  void leftDrop();
};

#endif
