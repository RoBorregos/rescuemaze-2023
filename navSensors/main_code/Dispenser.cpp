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
  dispenser.write(kgetKit);
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
  dispenser.write(kgetKit);
  delay(dispenserDelay);
  dispenser.write(krightDrop);
  delay(dispenserDelay);
  dispenser.write(kgetKit);
}

void Dispenser::stop()
{
  dispenser.write(kgetKit);
}

void Dispenser::leftDrop()
{
  dispenser.write(kgetKit);
  delay(dispenserDelay);
  dispenser.write(kleftDrop);
  delay(dispenserDelay);
  dispenser.write(kgetKit);
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

void Dispenser::write(int angle)
{
  dispenser.write(angle);
}