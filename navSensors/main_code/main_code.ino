// Usar bno y vlx para robot.
// Tmb poner el dispense

// https://prod.liveshare.vsengsaas.visualstudio.com/join?AB1AC08928A0F68DBD97A45B571C4353E54F

#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "MUX2C.h"
#include "BNO.h"

// Macros for vlx
#define front_vlx 0
#define right_vlx 1
#define left_vlx 2

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
BNO bno; // X is yaw, Y is pitch, and Z is roll.

double distancefront;
double distanceright;
double distanceleft;

int rDirection = 0;

double yaw;

double northYaw;
double southYaw;
double eastYaw;
double westYaw;

double pitch;

bool frontBlack = false;
bool rightBlack = false;
bool leftBlack = false;

char color = 'B';

// Setup de todos los sensores. Set pins en Sensors.h
void setup()
{
  Serial.begin(57600);

  // Set some sensor or i2c device
  bool setTcs = false;
  bool seti2c = false;
  bool setVLX = false;
  bool setBNO = true;
  bool testMotors = false;

  bno.init();

  initAll(&bno, true, true);
  setupData(setTcs, seti2c, setVLX, setBNO, testMotors);

  northYaw = 0;
  southYaw = 180;
  eastYaw = 90;
  westYaw = 270;
}
void exploreDFS()
{
  // Explore the maze

  // Check distance to the front
  distancefront = getFrontDistance();
  distanceright = getRightDistance();
  distanceleft = getLeftDistance();

  if (distancefront > 0.15)
  {
    // Go forward
    if (!forward())
    {
      return;
    }
    checkBlue();
    exploreDFS();
    backward();
  }
  else if (distanceleft > 0.15)
  {
    // Turn left
    turnLeft();
    if (!forward())
    {
      return;
    }
    checkBlue();
    exploreDFS();
    backward();
    turnRight();
  }
  else if (distanceright > 0.15)
  {
    // Turn right
    turnRight();
    if (!forward())
    {
      return;
    }
    checkBlue();
    exploreDFS();
    backward();
    turnLeft();
  }
}

void loop()
{
  // Get distance to the front
  exploreDFS();
}

void loop_()
{
  // Follow right wall
  while (true) //(distancefront > 0.15 && color != 'N' && distanceright < 0.15)
  {
    // Check distance and color
    distancefront = getFrontDistance();
    // distanceright = s->getVLXInfo(1);
    // distanceleft = s->getVLXInfo(2);

    color = s->getTCSInfo();

    // Check limit switches
    if (robot->rightLimitSwitch())
    {
      // Go back and turn left
      robot->advanceXMeters(-0.03);
      robot->goToAngle(dirToAngle[rDirection], false);
    }
    else if (robot->leftLimitSwitch())
    {
      // Go back and turn right
      robot->advanceXMeters(-0.03);
      robot->goToAngle(dirToAngle[rDirection], true);
    }
    else if (false) //(color == 'A') // Blue tile
    {
      // Go forward and wait 5 seconds
      robot->advanceXMeters(0.1);
      delay(5000);
    }
    else if (false) //(color == 'N')
    {
      // Go backward
      robot->advanceXMeters(-0.1);

      frontBlack = true;
    }

    // Check distance
    turnRight();
    distanceright = s->getVLXInfo(0);
    if (distanceright > 0.05)
    {
      if (false) //(distanceleft < 0.15)
      {
        turnRight();
        robot->advanceXMeters(-0.1);
        robot->advanceXMeters(0.1);
      }
      // Turn right
      // turnRight();

      if (frontBlack)
      {
        frontBlack = false;
        leftBlack = true;
      }
    }
    else
    {
      turnLeft();
    }
    if (distancefront < 0.03 || frontBlack)
    {
      // Turn left
      turnLeft();

      frontBlack = false;
      rightBlack = true;
    }

    // Go forward
    robot->advanceXMeters(0.1);
  }
}

int dirToAngle(int rdirection)
{
  switch (rdirection)
  {
  case 0:
    0;
    break;

  case 1:
    90;
    break;

  case 2:
    180;
    break;

  case 3:
    270;
    break;

  default:
    break;
  }
}

int getTurnDirection(int turn)
{
  if (turn) // right
  {
    if (rDirection == 3)
    {
      return 0;
    }
    else
    {
      return rDirection + 1;
    }
  }
  else // left
  {
    if (rDirection == 0)
    {
      return 3;
    }
    else
    {
      return rDirection - 1;
    }
  }

  return -1;
}

double getFrontDistance()
{
  return s->getVLXInfo(0); // Get front distance
}

double getLeftDistance()
{
  turnLeft();
  double distance = s->getVLXInfo(0); // Get front distance
  turnRight();

  return distance;
}

double getRightDistance()
{
  return s->getVLXInfo(1); // Get front distance
}

void checkBlue()
{
  // Check if the tile is blue
  if (s->getTCSInfo() == 'A')
  {
    // wait 5 seconds
    delay(5000);
  }
}

int forward()
{
  for (int i = 0; i < 10; i++)
  {
    robot->advanceXMeters(0.03);
    if (s->getTCSInfo() == 'N')
    {
      // Go back the distance traveled
      robot->advanceXMeters(-0.03 * i);

      return 0;
    }
  }

  return 1;
}

void backward()
{
  robot->advanceXMeters(-0.3);
}

void turnLeft()
{
  // Turn left
  robot->goToAngle(dirToAngle[getTurnDirection(0)], false);

  if (rDirection == 0)
  {
    rDirection = 3;
  }
  else
  {
    rDirection--;
  }
}

void turnRight()
{
  // Turn right
  robot->goToAngle(dirToAngle[getTurnDirection(1)], true);

  if (rDirection == 3)
  {
    rDirection = 0;
  }
  else
  {
    rDirection++;
  }
}

// Inicializar todos los sensores.
void initAll(BNO *bno, bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(bno, useVLX);
  s = &sensors;

  static Movement movement(bno, s, setIndividualConstants);
  robot = &movement;
}

// Funciones de apoyo

// Funcion que ayuda para calibrar sensores, encontrar idc.
void setupData(bool tcsSet, bool i2c, bool setVLX, bool setBNO, bool testMotors)
{

  if (i2c)
    mux.findI2C();

  if (tcsSet)
  {
    while (true)
    {
      s->rgbTCS();
      // Serial.println(sensors.getTCSInfo());
      if (setVLX)
      {
        Serial.println(s->getVLXInfo(0));
      }
    }
  }

  if (setVLX)
  {
    while (true)
    {
      Serial.println(s->getVLXInfo(0));
    }
  }

  if (setBNO)
  {
    while (true)
    {
      bno.anglesInfo();
    }
  }

  if (testMotors)
    robot->testAllMotors();

  if (i2c)
    while (true)
      delay(100);
}
