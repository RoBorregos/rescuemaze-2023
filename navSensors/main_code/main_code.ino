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

#define useleftvlx true
#define userightvlx true

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
BNO bno; // X is yaw, Y is pitch, and Z is roll.

double distancefront;
double distanceright;
double distanceleft;

int rDirection = 0;

double yaw;

double northYaw = 0;
double southYaw = 180;
double eastYaw = 90;
double westYaw = 270;

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
  /*
  while (true){
    s->printInfo(true, true, true);
  }
  */

  // Center of tile: 0.0620 VLX sensor 2: 0.0590 VLX sensor 3: 0.0550, use to find tile.
}

int newAngle = 0;

void loop()
{
  //checkLimitSwitches();
  // backward(1);
  // delay (1000);
  exploreFollowerWall();
  //robot->goToAngle(90, true);
  //robot->advanceXMeters(0.3, dirToAngle(rDirection));  
  
  //girosIzquierda();
  //delay(1000);
  /*
  girosIzquierda();
  delay(1000);

  robot->girarDeltaAngulo(-90);
  delay(1000);*/

  // while (true){
  //   s->printInfo(false, true, false);
  // }
  
  // robot->advanceXMeters(0.3, 90);
  // delay(200);

  // delay(100);
}

void exploreDFS()
{
  distancefront = getFrontDistance();
  distanceright = getRightDistance();
  distanceleft = getLeftDistance();

  if (distancefront > 0.15)
  {
    if (forwardTile())
    {
      exploreDFS();
      backwardTile();
    }
  }
  if (distanceright > 0.15)
  {
    turnRight();
    if (forwardTile())
    {
      exploreDFS();
      backwardTile();
    }

    turnLeft();
  }
  if (distanceleft > 0.15)
  {
    turnLeft();
    if (forwardTile())
    {
      exploreDFS();
      backwardTile();
    }
    turnRight();
  }
}

void exploreFollowerWall()
{
  while (true)
  {
      
    distancefront = getFrontDistance();
    distanceright = getRightDistance();
    distanceleft = getLeftDistance();

    if (distanceright > 0.15)
    {
      forward(3);
      turnRight();
      forwardTile();  
    }
    else if (distancefront < 0.08)
    {
      turnLeft();
    }
    // else if (distancefront < 0.07)
    // {
    //   turnLeft();
    // }
    else 
    {
      forward(1);
    }
  }
}

void testGiros(){
  robot->goToAngle(90, true);
  delay(1000);

  robot->goToAngle(180, true);
  delay(1000);

  robot->goToAngle(270, true);
  delay(1000);

  robot->goToAngle(360, true);
  delay(1000);

  robot->goToAngle(270, false);
  delay(1000);

  robot->goToAngle(180, false);
  delay(1000);

  robot->goToAngle(90, false);
  delay(1000);

  robot->goToAngle(0, false);
  delay(1000);  
}

void girosIzquierda()
{
  while (true)
  {
    int errorD = bno.getAngleX();
    int errorFiltrado = errorD + newAngle;
    Serial.println(errorD);
    if (errorFiltrado > 265 && errorFiltrado < 270)
    {
      robot->stop();
      newAngle -= 90;
      break;
    }
    else
    {
      double angle = 90;
      double angleNew = bno.getAngleX();
      robot->turnPID(90, (angle - angleNew), -1);
    }
  }
}

void girosDerecha()
{
  while (true)
  {
    int errorD = 90 - bno.getAngleX() + newAngle;
    if (errorD <= 0 && errorD > -5)
    {
      Serial.println("Angulo correecto");
      newAngle += 90;
    }
    else
    {
      double angle = 90;
      double angleNew = bno.getAngleX();
      robot->turnPID(90, angle - angleNew, 1);
    }
  }
}

// Maps rdirecion to angle
int dirToAngle(int rdirection)
{
  switch (rdirection)
  {
  case 0:
    return northYaw;
    break;

  case 1:
    return eastYaw;
    break;

  case 2:
    return southYaw;
    break;

  case 3:
    return westYaw;
    break;

  default:
    break;
  }
}

// Returns new rdirection given turn sign.
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
  return s->getVLXInfo(front_vlx); // Get front distance
}

double getLeftDistance()
{
  if (useleftvlx)
    return s->getVLXInfo(left_vlx); // Get front distance
  else
  {
    turnLeft();
    double distance = s->getVLXInfo(0); // Get front distance
    turnRight();

    return distance;
  }

  return 0;
}

double getRightDistance()
{
  if (userightvlx)
    return s->getVLXInfo(right_vlx); // Get front distance
  else
  {
    turnRight();
    double distance = s->getVLXInfo(0); // Get front distance
    turnLeft();

    return distance;
  }

  return 0;
}

void giroAbajo()
{
  int errorD = 180 - bno.getAngleX();
  if (errorD <= 0 && errorD > -5)
  {
    Serial.println("Angulo correecto");
  }
  else
  {
    double angle = 90;
    double angleNew = bno.getAngleX();
    robot->turnPID(90, angle - angleNew, 1);
  }
}

void forward(int times)
{
  for (int i = 0; i < times; i++)
  {
    robot->advanceXMeters(0.01, dirToAngle(rDirection));
    
    checkRamp();

    if (checkLimitSwitches())
      break;
  }
}

void forward()
{
  robot->advanceXMeters(0.01, dirToAngle(rDirection));
}

int forwardTile()
{
  // forward(18);
  double curDistance = getFrontDistance();
  double targetDistance = curDistance - 0.3;

  while (curDistance < targetDistance)
  {
    checkRamp();
    if (checkColors() == 'N')
    {
      return 0;
    }
    checkLimitSwitches();

    forward();
    curDistance = getFrontDistance();
  }

  return 1;
}

void backward(int times)
{
  robot->advanceXMeters(-0.01 * times, dirToAngle(rDirection));
}

void backward()
{
  robot->advanceXMeters(-0.01, dirToAngle(rDirection));
}

void backwardTile()
{
  double curDistance = getFrontDistance();
  double targetDistance = curDistance + 0.3;

  while (curDistance > targetDistance)
  {
    backward();
    curDistance = getFrontDistance();
  }
}

void turnLeft()
{
  // Check if there's a wall to the right
  bool wallRight = (getRightDistance() < 0.15);
  
  // Turn left
  robot->goToAngle(dirToAngle(getTurnDirection(0)), false);

  if (wallRight)
    backward(10);
   
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
  // Check if there's a wall 
  bool wallLeft = (getLeftDistance() < 0.15);
  
  // Turn right
  robot->goToAngle(dirToAngle(getTurnDirection(1)), true);

  if (rDirection == 3)
  {
    rDirection = 0;
  }
  else
  {
    rDirection++;
  }
}

void relativeTurn(double angle, bool goRight)
{
  double curAngle = s->getAngleX();
  double goAngle = curAngle + angle;
  
  if (angle + curAngle > 360)
  {
    goAngle -= 360;
  }
  else if (angle + curAngle < 0)
  {
    goAngle += 360;
  }
  
  robot->goToAngle(goAngle, goRight);
}

char checkColors()
{
  char color = s->getTCSInfo();
  
  if (color == 'N')
  {
    backward(3);
  }
  else if (color == 'A')
  {
    delay(5000);
  }

  return color;
}

void checkRamp()
{
  // Check pitch from imu
  pitch = s->getAngleY();

  // Check if pitch is greater than 10 degrees
  if (pitch > 15)
  {
    while (pitch > 15)
    {
      pitch = s->getAngleY();
      robot->advanceXMeters(0.10, dirToAngle(rDirection));
    }
  }
}

int checkLimitSwitches()
{
  if (robot->leftLimitSwitch() && robot->rightLimitSwitch())
  {
    backward(2);

    return 1;
  }
  else if (robot->leftLimitSwitch())
  {
    backward(2);
    relativeTurn(20, true);
    robot->advanceXMetersNoAngle(2);

    return 1;

//    if (northYaw < 359)
//    {
//      northYaw += 0;
//    }
//    else
//    {
//      northYaw = 0;
//    } 
//    if (eastYaw < 359)
//    {
//      eastYaw += 0;
//    }
//    else
//    {
//      eastYaw = 0;
//    }
//    if (southYaw < 359)
//    {
//      southYaw += 0;
//    }
//    else
//    {
//      southYaw = 0;
//    }
//    if (westYaw < 359)
//    {
//      westYaw += 0;
//    }
//    else
//    {
//      westYaw = 0;
//    }
  }
  else if (robot->rightLimitSwitch())
  {
    backward(2);
    relativeTurn(-20, false);
    forward(2);

    return 1;
//    if (northYaw > 0)
//    {
//      northYaw -= 0;
//    }
//    else
//    {
//      northYaw = 359;
//    } 
//    if (eastYaw > 0)
//    {
//      eastYaw -= 0;
//    }
//    else
//    {
//      eastYaw = 359;
//    }
//    if (southYaw > 0)
//    {
//      southYaw -= 0;
//    }
//    else
//    {
//      southYaw = 359;
//    }
//    if (westYaw > 0)
//    {
//      westYaw -= 0;
//    }
//    else
//    {
//      westYaw = 359;
//    }
  }

  return 0;
}

// Inicializar todos los sensores.
void initAll(BNO *bno, bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(bno, useVLX);
  s = &sensors;

  static Movement movement(bno, s, setIndividualConstants);
  robot = &movement;
}