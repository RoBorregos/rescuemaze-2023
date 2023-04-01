#include "Dispenser.h"

// Constructors

Dispenser::Dispenser()
{
  servoPin = 0;
}

Dispenser::Dispenser(uint8_t servoPin)
{
  this->servoPin = servoPin;
}

// Initialization

void Dispenser::initServo()
{
  dispenser.attach(servoPin);
}

// Dispenser Functions

void Dispenser::rightDrop()
{
  dispenser.write(kRightMovement);
  delay(rightDelay);
  dispenser.write(kLeftMovement);
  delay(leftDelay);
  dispenser.write(kStopMovement);
}

void Dispenser::leftDrop()
{
  dispenser.write(kLeftMovement);
  delay(leftDelay);
  dispenser.write(kRightMovement);
  delay(rightDelay);
  dispenser.write(kStopMovement);
}
