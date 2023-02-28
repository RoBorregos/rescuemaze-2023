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
  //mux.findI2C();
  
  initAll();
  setupTest();
  //delay(5000);
  //moveRoutine();

  
  ros::NodeHandle nh;
  nh.initNode();
  while (!nh.connected()){
    nh.spinOnce();
  }

  nh.loginfo("Arduino node initialized");
  
  reps = 0;
  RosBridge rosbridge(robot, s, &nh);
  rosbridge.run();
}

void initAll()
{
  bno.init();
  static Sensors sensors(&bno);
  s = &sensors;

  static Movement movement(&bno, s);
  robot = &movement;
}

void setupTest(){
  uint8_t colors[3][3] = {
    {19, 50, 70},
    {28, 40, 40},    
    {155, 200, 150},
  };

  uint8_t colorAmount = 3;
  char colorList[4] = {"wgb"};
  
  tcs.init(colors, colorAmount, colorList);
}

void loop()
{
  Serial.println("Logg");
  if (reps == ITERATIONS)
    return;
  
  reps++;
  delay(DELAY_MS);

  testMotor();
  // moveRoutine();

  tcs.printRGB();
  
}

void testMotor(){
  robot->testMotor();
}

void moveRoutine()
{
  Plot graph(robot);
  graph.startSequence();
  
  while (true)
  {
    robot->updateStraightPID(60);
    graph.plotTargetandCurrent();
  }
}
