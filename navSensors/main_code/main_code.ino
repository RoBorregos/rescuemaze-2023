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
  
}

void loop()
{
    while (true)//(distancefront > 0.15 && color != 'N' && distanceright < 0.15)
  {
    forward();
    turnRight();    
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
