#include <ros.h>

#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "Plot.h"
#include "MUX2C.h"
#include "RosBridge.h"

#define DELAY_MS 500
#define ITERATIONS 1000

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
Motor *motor = nullptr;

void setup()
{
  Serial.begin(57600);

  // Setup options

  // Set some sensor or i2c device
  bool setTcs = false;
  bool seti2c = false;
  bool setVLX = false;

  // Set a specific test
  bool doSpecificTest = false;

  // General options
  bool useVLX = true;
  bool setIndividualConstants = true;

  setupData(setTcs, seti2c, setVLX);

  specificTest(doSpecificTest, useVLX, setIndividualConstants);

  ros::NodeHandle nh;
  nh.initNode();
  
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  // Without ROS

  initAllRos(&nh, useVLX, setIndividualConstants);

  nh.loginfo("Arduino node initialized");

  RosBridge rosbridge(robot, s, &nh);
  rosbridge.run();
}

void initAllRos(ros::NodeHandle *nh, bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(useVLX);
  s = &sensors;

  static Movement movement(nh, s, setIndividualConstants);
  robot = &movement;
}

void initAll(bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(useVLX);
  s = &sensors;

  static Movement movement(s, setIndividualConstants);
  robot = &movement;
}

void setupData(bool tcsSet, bool i2c, bool setVLX)
{
  if (i2c)
    mux.findI2C();

  if (tcsSet)
  {
    static Sensors sensors(setVLX);
    while (true)
    {
      //sensors.rgbTCS();
      Serial.println(sensors.getTCSInfo());
      if (setVLX)
      {
       Serial.println(sensors.getVLXInfo(0)); 
      }
    }
  }

  if (setVLX)
  {
    static Sensors sensors(setVLX);
    while (true)
    {
      Serial.println(sensors.getVLXInfo(0));
    }   
  }

  if (i2c)
    while (true)
      delay(100);
}

void specificTest(bool test, bool useVLX, bool setIndividualConstants)
{
  if (!test)
    return;

  // Limit switches
  /*
  while (true){
    robot->getSwitches();
    delay(200);
  }*/
  //
  initAll(useVLX, setIndividualConstants);

  // Do some specific test. E.g. pid test
  Serial.println("Specific test");

  moveRoutine(); // Test PID
  
  robot->testAllMotors(); // Test motor direction.

  while (true)
    testMotor();

  robot->advanceXMeters(1);

  while (true)
    delay(5000);
}

void loop()
{
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
    // graph.plotPWM();
  }
}