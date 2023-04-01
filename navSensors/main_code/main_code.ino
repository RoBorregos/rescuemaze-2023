// Usar bno y vlx para robot.
// Tmb poner el dispense

#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "MUX2C.h"
#include "BNO.h"

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;

// Setup de todos los sensores. Set pins en Sensors.h
void setup()
{
  Serial.begin(57600);
  
  // Set some sensor or i2c device
  bool setTcs = false;
  bool seti2c = false;
  bool setVLX = false;
  bool setBNO = false;

  initAll(bno, true, true);
  
  setupData(setTcs, seti2c, setVLX, setBNO);
  
  bno.init();
  
}

double distancefront;
double distanceright;
double distanceleft;

bool frontBlack = false;

char color = 'B';

void loop()
{

  // Follow right wall
  while (distancefront > 0.15 && color != 'N' && distanceright < 0.15)
  {
    // Check distance and color
    distancefront = sensors->getVLXInfo();
    distanceright = sensors->getVLXInfo();

    color = sensors->getTCSInfo();
    
    // Go forward
    if (color == 'A') // Blue tile
    {
      // Go forward and wait 5 seconds
      
      
      
    }
    else if (color == 'N')
    {
      // Go backward


    }

    // Check distancehttps://prod.liveshare.vsengsaas.visualstudio.com/join?AB1AC08928A0F68DBD97A45B571C4353E54F

    if (distanceright > 0.15)
    {
      // Turn right
    }
    else if (distancefront < 0.15)
    {
      // Turn left
    }
    
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
void setupData(bool tcsSet, bool i2c, bool setVLX, bool setBNO)
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

  if (i2c)
    while (true)
      delay(100);
}