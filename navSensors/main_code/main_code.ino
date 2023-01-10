
#include "BNO.h"
#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "Plot.h"

#define DELAY_MS 500
#define ITERATIONS 1000

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
BNO bno;
Motor *motor = nullptr;

int reps;

void setup()
{
  Serial.begin(57600);
  // mux.findI2C();
  initAll();
  delay(5000);
  moveRoutine();
  reps = 0;
}

void initAll()
{
  bno.init();
  static Sensors sensors(&bno);
  s = &sensors;

  static Movement movement(&bno, s);
  robot = &movement;
}

void loop()
{
  return;
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

void moveRoutine()
{
  return;
  Plot graph(robot);
  graph.startSequence();
  
  while (true)
  {
    robot->updateStraightPID(60);
    graph.plotTargetandCurrent();
  }
}
