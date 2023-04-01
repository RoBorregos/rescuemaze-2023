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
#define front_vlx 1
#define right_vlx 7
#define left_vlx 0

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
  
  while (true){
    s->printInfo(true, true, true);
  }

  setupData(setTcs, seti2c, setVLX, setBNO, testMotors);

  // East
  if (northYaw > -90)
    eastYaw = northYaw - 90;
  else
    eastYaw = northYaw + 90 * 3;

  // South
  if (northYaw > 0)
    southYaw = northYaw - 180;
  else
    southYaw = northYaw + 180;

  // West
  if (northYaw > 90)
    westYaw = northYaw - 90 * 3;
  else
    westYaw = northYaw + 90;
}
void exploreDFS()
{
  // Explore the maze

  // Check distance to the front
  distancefront = s->getVLXInfo(0);
  distanceright = s->getVLXInfo(1);
  distanceleft = s->getVLXInfo(2);

  if (distancefront > 0.15)
  {
    // Go forward
    forward();
    exploreDFS();
    backward();
  }
  else if (distanceleft > 0.15)
  {
    // Turn left
    turnLeft();
    forward();
    exploreDFS();
    backward();
    turnRight();
  }
  else if (distanceright > 0.15)
  {
    // Turn right
    turnRight();
    forward();
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
  while (true)//(distancefront > 0.15 && color != 'N' && distanceright < 0.15)
  {
    // Check distance and color
    distancefront = s->getVLXInfo(0);
    // distanceright = s->getVLXInfo(1);
    // distanceleft = s->getVLXInfo(2);

    color = s->getTCSInfo();

    // Check limit switches
    if (robot->rightLimitSwitch())
    {
      // Go back and turn left
      robot->advanceXMeters(-0.03);
      robot->girarDeltaAngulo(10);
    }
    else if (robot->leftLimitSwitch())
    {
      // Go back and turn right
      robot->advanceXMeters(-0.03);
      robot->girarDeltaAngulo(-10);
    }
    else if (false)//(color == 'A') // Blue tile
    {
      // Go forward and wait 5 seconds
      robot->advanceXMeters(0.1);
      delay(5000);
    }
    else if (false)//(color == 'N')
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
      if (false)//(distanceleft < 0.15)
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

void forward()
{
  robot->advanceXMeters(0.3);
}

void backward()
{
  robot->advanceXMeters(-0.3);
}

void turnLeft()
{
  // Turn left
  robot->girarDeltaAngulo(-90);

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
  robot->girarDeltaAngulo(90);

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
        //Serial.println(s->getVLXInfo(2));
      }
    }
  }

  if (setVLX)
  {
    while (true)
    {
      
      Serial.println(s->getVLXInfo(1));
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

