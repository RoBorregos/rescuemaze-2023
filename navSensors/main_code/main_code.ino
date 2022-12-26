
#include "BNO.h"
#include "Movement.h"
#include "Sensors.h"

#define DELAY_MS 500
#define ITERATIONS 1000

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
BNO bno;

int reps;

void setup()
{
  Serial.begin(115200);
  mux.findI2C();

  Serial.println("Ejecutando setup");
  initAll();
  reps = 0;
}

void initAll()
{
  bno.init();
  Sensors sensors(&bno);
  s = &sensors;
}

void loop()
{
  if (reps == ITERATIONS)
    return;
  //           bno,   vlx,  mlx,   tcs
  s->printInfo(true, false, false, false);
  s->printInfo(false, true, false, false);
  // s->printInfo(false, true, false, false);
  // bno.anglesInfo();
  // Serial.println(s->getVLXInfo(0));
  delay(DELAY_MS);
  reps++;
}
