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
  // dispenser.write(kRightMovement);
  // delay(rightDelay);
  // dispenser.write(kLeftMovement);
  // delay(rightDelay);
  // dispenser.write(kStopMovement);
  // dispenser.write(kLeftMovement);
  dispenser.write(kRightMovement);
  delay(rightDelay);
  dispenser.write(kLeftMovement);
  delay(rightDelay);
  dispenser.write(kStopMovement);
}

void Dispenser::stop()
{
  dispenser.write(kStopMovement);
}

void Dispenser::leftDrop()
{
  dispenser.write(kLeftMovement);
  delay(leftDelay);
  dispenser.write(kRightMovement);
  delay(leftDelay);
  dispenser.write(kStopMovement);
}

// Gets sign which refers to where should a kit be dropped
void Dispenser::dropNKits(int kits)
{

  while (kits > 0)
  {
    rightDrop();
    kits--;
  }

  while (kits < 0)
  {
    leftDrop();
    kits++;
  }
}