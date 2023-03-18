#include <ros.h>

#include "BNO.h"
#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "Plot.h"
#include "MUX2C.h"
#include "TCS.h"
#include "RosBridge.h"

#define DELAY_MS 500
#define ITERATIONS 1000

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
BNO bno;
Motor *motor = nullptr;
TCS tcs(0, 15);

int reps;

void setup()
{
  Serial.begin(57600);

  setupData(false, false);

  specificTest(false);

  ros::NodeHandle nh;
  nh.initNode();
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  // Without ROS

  initAllRos(&nh);

  nh.loginfo("Arduino node initialized");

  reps = 0;
  RosBridge rosbridge(robot, s, &nh);
  rosbridge.run();
}

void initAllRos(ros::NodeHandle* nh)
{
  bno.init();
  static Sensors sensors(&bno);
  s = &sensors;

  static Movement movement(nh, s, true);
  robot = &movement;

  uint8_t colors[3][3] = {
      {137, 78, 58},
      {64, 85, 128},
      {85, 85, 85},
  };

  uint8_t colorAmount = 3;
  char colorList[4] = {"obB"};

  tcs.init(colors, colorAmount, colorList);
}

void initAll()
{
  bno.init();
  static Sensors sensors(&bno);
  s = &sensors;

  static Movement movement(&bno, s, true);
  robot = &movement;

  uint8_t colors[3][3] = {
      {137, 78, 58},
      {64, 85, 128},
      {85, 85, 85},
  };

  uint8_t colorAmount = 3;
  char colorList[4] = {"obB"};
  tcs.init(colors, colorAmount, colorList);
}

void setupData(bool tcsSet, bool i2c)
{
  if (i2c)
    mux.findI2C();

  if (tcsSet)
  {
    while (true)
    {
      tcs.printRGB();
    }
  }
  
  if (i2c)
    while (true)
      delay(100);
}

void specificTest(bool test)
{  
  if (!test)
    return;
        
  initAll();
  
  Serial.println("Specific test");

  while (false)
    tcs.printColor();

  moveRoutine();
  
  while (true)
    testMotor();
  
  robot->advanceXMeters(1);

  while (true) 
    delay(5000);
}

void loop()
{
  if (reps == ITERATIONS)
    return;

  reps++;
  delay(DELAY_MS);
}

void testMotor()
{
  robot->testMotor();
}

void moveRoutine()
{
  Plot graph(robot);
  graph.startSequence();

  while (true)
  {
    robot->updateStraightPID(90);
    graph.plotTargetandCurrent();
  }
}
