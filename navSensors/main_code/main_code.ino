#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "MUX2C.h"
#include "BNO.h"
#include "GeneralChecks.h"
#include "Plot.h"
#include "CommonK.h"

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

// Setup de todos los sensores. Set pins en Sensors.h
void setup()
{
  Serial.begin(57600);

  bno.init();

  initAll(&bno, true, true);
  GeneralChecks checks(robot);
  checks.test();
  // checks.checkSensorData();

  // Center of tile: 0.0620 VLX sensor 2: 0.0590 VLX sensor 3: 0.0550, use to find tile.
}

void loop()
{
}

// Inicializar todos los sensores.
void initAll(BNO *bno, bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(bno, useVLX);
  s = &sensors;

  static Movement movement(bno, s, setIndividualConstants);
  robot = &movement;
}
