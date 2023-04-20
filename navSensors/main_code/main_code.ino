#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "MUX2C.h"
#include "BNO.h"
#include "GeneralChecks.h"
#include "Plot.h"
#include "CommonK.h"
#include "RosBridge2.h"

// Macros for vlx
#define front_vlx 0
#define right_vlx 1
#define left_vlx 2
#define kusingROS false

#define useleftvlx true
#define userightvlx true

Movement *robot = nullptr;
Sensors *s = nullptr;
MUX2C mux;
BNO bno; // X is yaw, Y is pitch, and Z is roll.

#if kusingROS

#include <ros.h>
#include "RosBridge.h"

// Setup de todos los sensores. Set pins en Sensors.h
void setup()
{
  Serial.begin(57600);


  // General options
  bool useVLX = true;
  bool setIndividualConstants = true;
  
  ros::NodeHandle nh;
  nh.initNode();
  
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  // Without ROS
  bno.init();
  initAllRos(&nh, &bno, useVLX, setIndividualConstants);

  nh.loginfo("Arduino node initialized");

  //RosBridge rosbridge(robot, s, &nh);

  //s->setRosBridge(&rosbridge); // Pass reference to update distance using lidar.

  
  //rosbridge.rosBridgeTest();
  //rosbridge.run();

  // Serial.begin(57600);

  // bno.init();

  // initAll(&bno, true, true);
  GeneralChecks checks(robot);
  // checks.checkWheelDirections();
  // checks.checkAll();
  checks.test();
  // checks.checkSensorData();

  // Center of tile: 0.0620 VLX sensor 2: 0.0590 VLX sensor 3: 0.0550, use to find tile.
}

void initAllRos(ros::NodeHandle *nh, BNO *bno, bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(bno, useVLX);
  s = &sensors;

  static Movement movement(nh, bno, s, setIndividualConstants);
  robot = &movement;
}

void loop()
{
}

#else

// Implementation without ROS.

double distancefront;
double distanceright;
double distanceleft;

int rDirection = 0;

double yaw;

double northYaw = 0;
double eastYaw = 90;
double southYaw = 180;
double westYaw = 270;

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
  const bool seti2c = false;
  bool setVLX = false;
  bool setBNO = true;
  bool testMotors = false;

  //mux.findI2C();
  
  bno.init();
  initAll(&bno, true, true);
  RosBridge2 rosbridge(robot, s, &bno);
  //rosbridge.run();

  GeneralChecks checks(robot);
  // checks.checkWheelDirections();
  //checks.checkAll();
  //checks.printRevolutions();
  checks.test();
}

int newAngle = 0;

void loop()
{
  return;
  s->printInfo(false, true, true, true);
  // forward();
}

void exploreDFS()
{
  distancefront = getFrontDistance();
  distanceright = getRightDistance();
  distanceleft = getLeftDistance();

  if (distancefront > 0.15)
  {
    if (forward())
    {
      exploreDFS();
      backward();
    }
  }
  if (distanceright > 0.15)
  {
    turnRight();
    if (forward())
    {
      exploreDFS();
      backward();
    }

    turnLeft();
  }
  if (distanceleft > 0.15)
  {
    turnLeft();
    if (forward())
    {
      exploreDFS();
      backward();
    }
    turnRight();
  }
}

void exploreFollowerWall()
{
  while (true)
  {

    distancefront = getFrontDistance();
    distanceright = getRightDistance();
    distanceleft = getLeftDistance();

    if (distancefront > 0.15)
    {
      Serial.println("forward");
      forward();
    }
    else if (distanceright < 0.15)
    {
      Serial.println("left");
      turnLeft();
    }
    // else if (distancefront < 0.07)
    // {
    //   turnLeft();
    // }
    else
    {
      Serial.println("right");
      // forward(1);
      turnRight();
    }
  }
}

// Maps rdirecion to angle
int dirToAngle(int rdirection)
{
  switch (rdirection)
  {
  case 0:
    return northYaw;
    break;

  case 1:
    return eastYaw;
    break;

  case 2:
    return southYaw;
    break;

  case 3:
    return westYaw;
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

double getFrontDistance()
{
  return s->getVLXInfo(front_vlx); // Get front distance
}

double getLeftDistance()
{
  if (useleftvlx)
    return s->getVLXInfo(left_vlx); // Get front distance
  else
  {
    turnLeft();
    double distance = s->getVLXInfo(0); // Get front distance
    turnRight();

    return distance;
  }

  return 0;
}

double getRightDistance()
{
  if (userightvlx)
    return s->getVLXInfo(right_vlx); // Get front distance
  else
  {
    turnRight();
    double distance = s->getVLXInfo(0); // Get front distance
    turnLeft();

    return distance;
  }

  return 0;
}

int forward()
{
  // robot->advanceXMeters(0.3, 0);
  return robot->cmdMovement(1);
}

int backward()
{
  return robot->cmdMovement(4);
  // robot->advanceXMeters(-0.3, dirToAngle(rDirection));
}

void forward(int times)
{
  for (int i = 0; i < times; i++)
  {
    robot->advanceXMeters(0.01, 0);
  }
}

void backward(int times)
{
  for (int i = 0; i < times; i++)
  {
    robot->advanceXMeters(-0.01, dirToAngle(rDirection));
  }
}

void turnLeft()
{

  robot->cmdMovement(2, 1);
  return;

  // Check if there's a wall to the right
  bool wallRight = (getRightDistance() < 0.15);

  // Turn left
  robot->goToAngle(dirToAngle(getTurnDirection(0)));

  if (wallRight)
    backward(10);

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

  robot->cmdMovement(3, 1);
  return;

  // Check if there's a wall
  bool wallLeft = (getLeftDistance() < 0.15);

  // Turn right
  robot->goToAngle(dirToAngle(getTurnDirection(1)));

  if (rDirection == 3)
  {
    rDirection = 0;
  }
  else
  {
    rDirection++;
  }
}

void relativeTurn(double angle, bool goRight)
{
  double curAngle = s->getAngleX();
  double goAngle = curAngle + angle;

  if (angle + curAngle > 360)
  {
    goAngle -= 360;
  }
  else if (angle + curAngle < 0)
  {
    goAngle += 360;
  }

  // robot->goToAngle(goAngle, goRight);
}

int checkVictims()
{
  // return;

  if (Serial.available() > 0)
  {
    char buffer[32];
    int numBytes = Serial.readBytesUntil('\n', buffer, 32);

    // process the incoming data as needed
    // for example, split the data into individual values and convert them from strings to integers
    int values[2];
    char *ptr = strtok(buffer, ",");
    int i = 0;
    while (ptr != NULL && i < 2)
    {
      int value = atoi(ptr);
      if (value >= 0 && value <= 2)
      {
        values[i] = value;
        ptr = strtok(NULL, ",");
        i++;
      }
      else
      {
        // invalid input, discard the rest of the message
        break;
      }
    }

    // values[0] gives the number of kits to drop
    // values[1] gives the side of the victim

    int victims = 0;

    if (values[1] == 0) // left
    {
      victims = values[0] * -1;
      // Serial.print("Left");
      // Serial.println(victims);
    }
    else // right
    {
      victims = values[0];
      // Serial.print("Right");
      // Serial.println(victims);
    }

    if (victims != 0)
      robot->cmdMovement(7, victims);
    // while (values[0] > 0) {
    //   robot->dispenser.leftDrop();

    //   values[0]--;
    // }

    // while (values[1] > 0) {
    //   robot->dispenser.rightDrop();

    //   values[1]--;
    // }
  }
}

char checkColors()
{
  char color = s->getTCSInfo();

  if (color == 'N')
  {
    backward(3);
  }
  else if (color == 'A')
  {
    delay(5000);
  }

  return color;
}

void checkRamp()
{
  // Check pitch from imu
  pitch = s->getAngleY();

  // Check if pitch is greater than 10 degrees
  if (pitch > 15)
  {
    while (pitch > 15)
    {
      pitch = s->getAngleY();
      robot->advanceXMeters(0.10, dirToAngle(rDirection));
    }
  }
}

int checkLimitSwitches()
{
  if (s->leftLimitSwitch() && s->rightLimitSwitch())
  {
    backward(2);

    return 1;
  }
  else if (s->leftLimitSwitch())
  {
    backward(2);
    shiftAngles(2);
    // relativeTurn(20, true);
    // robot->advanceXMetersNoAngle(2);

    return 1;
  }
  else if (s->rightLimitSwitch())
  {
    backward(2);
    shiftAngles(-2);
    // relativeTurn(-20, false);
    // forward(2);

    return 1;
  }

  return 0;
}

void shiftAngles(int error)
{
  if (error > 0)
  {

    if (northYaw + error <= 359)
    {
      northYaw += error;
    }
    else
    {
      northYaw = northYaw + error - 360;
    }

    if (eastYaw + error <= 359)
    {
      eastYaw += error;
    }
    else
    {
      eastYaw = eastYaw + error - 360;
    }

    if (southYaw + error <= 359)
    {
      southYaw += error;
    }
    else
    {
      southYaw = southYaw + error - 360;
    }

    if (westYaw + error <= 359)
    {
      westYaw += error;
    }
    else
    {
      westYaw = westYaw + error - 360;
    }
  }
  else
  {
    if (northYaw + error >= 0)
    {
      northYaw += error;
    }
    else
    {
      northYaw = northYaw + error + 360;
    }

    if (eastYaw + error >= 0)
    {
      eastYaw += error;
    }
    else
    {
      eastYaw = eastYaw + error + 360;
    }

    if (southYaw + error >= 0)
    {
      southYaw += error;
    }
    else
    {
      southYaw = southYaw + error + 360;
    }

    if (westYaw + error >= 0)
    {
      westYaw += error;
    }
    else
    {
      westYaw = westYaw + error + 360;
    }
  }
}

#endif

// Inicializar todos los sensores.
void initAll(BNO *bno, bool useVLX, bool setIndividualConstants)
{
  static Sensors sensors(bno, useVLX);
  s = &sensors;

  static Movement movement(bno, s, setIndividualConstants);
  robot = &movement;
}
