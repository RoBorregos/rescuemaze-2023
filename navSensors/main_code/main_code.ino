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
  /*
  while (true){
    s->printInfo(true, true, true);
  }*/
}

int newAngle = 0;

void loop()
{
  robot->goToAngle(90, true);
  delay(10000);
  
  //girosIzquierda();
  //delay(1000);
  /*
  girosIzquierda();
  delay(1000);

  robot->girarDeltaAngulo(-90);
  delay(1000);*/
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
    return 0;
    break;

  case 1:
    return 90;
    break;

  case 2:
    return 180;
    break;

  case 3:
    return 270;
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
