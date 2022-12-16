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

void Dispenser::restart()
{
  dispenser.write(kMidAngle);
}

void Dispenser::kitDrop()
{
  for (int current_angle = 0; current_angle < kDropAngle; current_angle += 10)
  {
    dispenser.write(current_angle);
    delay(kTime2Drop);
  }
  dispenser.write(kInitAngle);
}

void Dispenser::rightDrop()
{
  for (int current_angle = 0; current_angle < kRightAngle; current_angle += 10)
  {
    dispenser.write(current_angle);
    delay(kTime2Drop);
  }
  restart();
}

void Dispenser::leftDrop()
{
  for (int current_angle = 0; current_angle < kLeftAngle; current_angle += 10)
  {
    dispenser.write(current_angle);
    delay(kTime2Drop);
  }
  restart();
}
