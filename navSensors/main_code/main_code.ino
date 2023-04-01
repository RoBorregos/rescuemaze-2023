// Usar bno y vlx para robot.
// Tmb poner el dispense

// https://prod.liveshare.vsengsaas.visualstudio.com/join?AB1AC08928A0F68DBD97A45B571C4353E54F

#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "MUX2C.h"
#include "BNO.h"

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;

double distancefront;
double distanceright;
double distanceleft;

double yaw;

double northYaw;
double southYaw;
double eastYaw;
double westYaw;

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
  bool setBNO = false;
  bool testMotors = false

  initAll(bno, true, true);

  setupData(setTcs, seti2c, setVLX, setBNO, testMotors);

  bno.init();

  northYaw = bno.getAngleZ();

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

void loop()
{
  // Follow right wall
  while (distancefront > 0.15 && color != 'N' && distanceright < 0.15)
  {
    // Check distance and color
    distancefront = sensors->getVLXInfo();
    distanceright = sensors->getVLXInfo();

    color = sensors->getTCSInfo();

    // Check limit switches
    if (robot->rightLimitSwitch)
    {
      // Go back and turn left
      robot->advanceXMeters(-0.03)
          robot->girarDeltaAngulo(10)
    }
    else if (robot->leftLimitSwitch)
    {
      // Go back and turn right
      robot->advanceXMeters(-0.03)
          robot->girarDeltaAngulo(-10)
    }
    else if (color == 'A') // Blue tile
    {
      // Go forward and wait 5 seconds
      robot->advanceXMeters(0.1);
      delay(5000);
    }
    else if (color == 'N')
    {
      // Go backward
      robot->advanceXMeters(-0.1);

      frontBlack = true;
    }

    // Check distance
    if (distanceright > 0.15)
    {
      // Turn right
      robot->girarDeltaAngulo(90);

      if (frontBlack)
      {
        frontBlack = false;
        leftBlack = true;
      }
    }
    else if (distancefront < 0.10 || frontBlack)
    {
      // Turn left
      robot->girarDeltaAngulo(-90);

      frontBlack = false;
      rightBlack = true;
    }

    // Go forward
    robot->advanceXMeters(0.1);
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
      bno->anglesInfo();
    }
  }

  if (testMotors)
    robot->testAllMotors();

  if (i2c)
    while (true)
      delay(100);
}