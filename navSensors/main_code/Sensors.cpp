#include "Sensors.h"

// Definitions of non-int static constexpr variables. (Declarations and initialization in .h)

constexpr char Sensors::colorList[];
constexpr int Sensors::colors[Sensors::colorAmount][3];
constexpr int Sensors::colorThresholds[Sensors::colorAmount][6];

// Constructors

Sensors::Sensors(bool usingVLX)
{
  this->usingVLX = usingVLX;
  this->usingBNO = false;
  initSensors();
}

Sensors::Sensors(BNO *bno, bool usingVLX) : bno(bno)
{
  this->usingVLX = usingVLX;
  this->usingBNO = true;
  initSensors();
}

// Initialization

void Sensors::initSensors()
{

  if (usingVLX)
  {
    for (int i = 0; i < kMuxVLX; i++)
    {
      vlx[i].setMux(kMuxPins[i]);
    }
    // vlxBack.setMux(6);

    // VLX init
    for (int i = 0; i < kMuxVLX; i++)
    {
      vlx[i].init();
    }
    // vlxBack.init();
  }

  // TCS init
  tcs.setMux(kMuxTCS);
  tcs.setPrecision(kTCSPrecision);
  Wire.begin();
  tcs.init(colors, colorAmount, colorList, colorThresholds);

  initSwitches();
  initLeds();

  if (CK::debugOled)
  {
    screen.init();
    // screen.RBRGS();
  }

  if (CK::calibrateBNO)
  {
    logActive("Calibrating BNO");
    bothLedOn();

    if (!CK::kusingROS && CK::debugBNOCalibration)
    {
      // Serial.println("Calibrating BNO");
    }

    bno->restoreCalibration(); // Load offsets

    long int initialT = millis();

    // Calibrate Magnetometer by moving robot.
    while (!bno->isCalibrated())
    {
      int system, gyro, accel, mag;
      bno->getCalibration(system, gyro, accel, mag);
      logActive("S=" + String(system) + ";Mag=" + String(mag) + ";G=" + String(gyro) + ";A=" + String(accel), true, 0, 3);
      if (!CK::kusingROS && CK::debugBNOCalibration)
        bno->displayCalStatus();

      if (millis() - initialT > maxBNOTime)
      {
        logActive("BNO calibration timed out.");
        // Led blink to indicate that BNO calibration timed out.
        bothLedOff();
        delay(1000);

        bothLedOn();
        delay(1000);

        bothLedOff();
        delay(1000);
        break;
      }
    }
    // Serial.print("BNO calibration: ");
    bno->displayCalStatus();

    if (!CK::kusingROS && CK::debugBNOCalibration)
    {

      if (bno->isCalibrated())
        Serial.println("BNO calibration finished.");
      else
        Serial.println("BNO calibration failed.");
    }

    // Give some time to place robot on the ground. The initial position will be
    // considered as north.
    double timeToPlaceRobot = 5000; // 3 seconds
    initialT = millis();
    toggleRightLed();

    while (millis() - initialT < timeToPlaceRobot)
    {
      toggleBothLeds();
      if (!CK::kusingROS && CK::debugBNOCalibration)
      {

        String m = "Place robot on the ground in " + String((timeToPlaceRobot - (millis() - initialT)) / 1000) + " seconds";
        logActive(m);
      }
      delay(100);
    }

    bothLedOff();
  }
  else
  {
    bno->setExtCUse();
  }
  logActive("Sensors initiated");
}

// Sensor Methods

void Sensors::printInfo(bool bno, bool vlx, bool tcs, bool limitSwitches)
{
  if (bno && usingBNO)
  {
    this->bno->anglesInfo();
    this->bno->displayCalStatus();
  }

  if (vlx && usingVLX && !CK::kusingROS)
  {
    for (int i = 0; i < 4; i++)
    {
      Serial.print(" VLX sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(float(getVLXInfo(i)), 4);
    }
    Serial.println();
  }

  if (tcs && !CK::kusingROS)
  {
    Serial.print("TCS sensor  ");
    this->tcs.printRGB();
    Serial.println(getTCSInfo());
  }

  if (limitSwitches)
    debugLimitSwitches();

  delay(50);
}

float Sensors::getVLXInfo(int posVLX)
{
  if (!usingVLX && !CK::kusingROS)
  {
    // Serial.println("WARNING: invalid method call, VLX marked as not used.");
    return -1;
  }

  return vlx[posVLX].getDistance();
}

void Sensors::updateDistLidar(float front, float back, float left, float right)
{
  // Values weren't updated
  if (wallDistances[0] == front && wallDistances[1] == back && wallDistances[2] == left && wallDistances[3] == right)
  {
    lidarAttemptCount++;
    if (lidarAttemptCount > 15)
    {
      // Lidar is not working, use vlx
      usingLidar = false;
      return;
    }
    rosBridge->readOnce();

    // rosBridge->updateDistLidar();
    return;
  }
  else
  {
    usingLidar = true;
  }
  wallDistances[0] = front;
  wallDistances[1] = back;
  wallDistances[2] = left;
  wallDistances[3] = right;

  // 19*18
  // Update values adding constant error
  // wallDistances[0] = front - 19 / 200.0;
  // wallDistances[1] = back - 19 / 200.0;
  // wallDistances[2] = left - 18 / 200.0;
  // wallDistances[3] = right - 18 / 200.0;
  lidarAttemptCount = 0;

  logActive("Fr: " + String(wallDistances[0]) + " Back: " + String(wallDistances[1]), true, 0, 6);
  logActive("L: " + String(wallDistances[2]) + " R: " + String(wallDistances[3]), true, 0, 7);
}

void Sensors::updateDistLidar(float front)
{
  // Values weren't updated
  if (wallDistances[0] == front)
  {
    lidarAttemptCount++;
    if (lidarAttemptCount > 15)
    {
      // Lidar is not working, use vlx
      usingLidar = false;
      return;
    }
    rosBridge->readOnce();

    // rosBridge->updateDistLidar();
    return;
  }
  else
  {
    usingLidar = true;
  }
  wallDistances[0] = front;

  // 19*18
  // Update values adding constant error
  // wallDistances[0] = front - 19 / 200.0;
  // wallDistances[1] = back - 19 / 200.0;
  // wallDistances[2] = left - 18 / 200.0;
  // wallDistances[3] = right - 18 / 200.0;
  lidarAttemptCount = 0;

  logActive("Fr: " + String(wallDistances[0]), true, 0, 6);
}

void Sensors::getLidarDistances(double &front, double &back, double &left, double &right)
{
  if (rosBridge == nullptr)
    return;

  front = wallDistances[0];
  back = wallDistances[1];
  left = wallDistances[2];
  right = wallDistances[3];
}

bool Sensors::readMotorInit()
{
  int val = digitalRead(kMotorPin);
  return val == HIGH;
}

void Sensors::initLeds()
{
  pinMode(kDigitalPinsLEDS[0], OUTPUT);
  pinMode(kDigitalPinsLEDS[1], OUTPUT);
}

float Sensors::getQuatX()
{
  return bno->getQuat_x();
}

float Sensors::getQuatY()
{
  return bno->getQuat_y();
}

float Sensors::getQuatZ()
{
  return bno->getQuat_z();
}

float Sensors::getQuatW()
{
  return bno->getQuat_w();
}

float Sensors::getAngVelX()
{
  return bno->getAngVel_x();
}

float Sensors::getAngVelY()
{
  return bno->getAngVel_y();
}

float Sensors::getAngVelZ()
{
  return bno->getAngVel_z();
}

float Sensors::getLinAccX()
{
  return bno->getLinAcc_x();
}

float Sensors::getLinAccY()
{
  return bno->getLinAcc_y();
}

float Sensors::getLinAccZ()
{
  return bno->getLinAcc_z();
}

float Sensors::getAngleX()
{
  return bno->getAngleX();
}

float Sensors::getAngleY()
{
  return bno->getAngleY();
}

float Sensors::getAngleZ()
{
  return bno->getAngleZ();
}

char Sensors::getTCSInfo()
{
  return tcs.getColorWithThresholds();
  // return tcs.getColorWithPrecision();
}

// 0 front, 1 back, 2 left, 3 right
float Sensors::getDistInfo(int direction)
{
  // Debug using leds
  if (usingLidar)
  {
    logActive("Usando lidar", true, 0, 5, true);
  }
  else
  {
    logActive("NO usando lidar", true, 0, 5, true);
  }

  if (lidarAttemptCount > 15)
  {
    // Lidar is not working, use vlx
    usingLidar = false;
  }

  usingLidar = false;

  if (usingLidar && rosBridge != nullptr && direction != 2 && direction != 3)
  {
    rosBridge->readOnce();

    if (direction >= 0 && direction <= 3 && usingLidar)
    {
      if (isValid(wallDistances[direction]))
      {
        lidarAttemptCount = 0;
        return wallDistances[direction] - 19 / 200.0; // Subtract distance from lidar to robot's wall
      }
      else
      {
        lidarAttemptCount++;
        return getDistInfo(direction);
      }
    }
  }

  switch (direction)
  {
  case 0:
    return getVLXInfo(vlx_front);
  case 2:
    return getVLXInfo(vlx_left);
  case 3:
    return getVLXInfo(vlx_right);
  default:
    return -1;
  }
}

void Sensors::bnoAngles(float &x, float &y, float &z)
{
  if (!usingBNO && !CK::kusingROS)
  {
    // Serial.println("WARNING: invalid method call, BNO marked as not used.");
    return;
  }
  bno->getAll(x, y, z);
}

void Sensors::bnoPrint()
{
  if (!usingBNO && !CK::kusingROS)
  {
    // Serial.println("WARNING: invalid method call, BNO marked as not used.");
    return;
  }
  bno->anglesInfo();
}

void Sensors::rgbTCS()
{
  tcs.printRGB();
}
void Sensors::rgbTCSClear()
{
  tcs.printRGBC();
}

void Sensors::checkTCS()
{
  tcs.printColorMatrix();
  tcs.printColorList();
}

void Sensors::getLimitSwitches(bool &right, bool &left)
{
  right = rightLimitSwitch();
  left = leftLimitSwitch();
}

bool Sensors::leftLimitSwitch()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[0]);

  return val == HIGH;
}

bool Sensors::rightLimitSwitch()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[1]);

  return val == HIGH;
}

void Sensors::debugLimitSwitches()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[0]);

  if (CK::kusingROS)
    return;

  if (val == HIGH)
  {
    Serial.println("Left switch is 1");
  }
  else
  {
    Serial.println("Left switch is 0");
  }
  int val2 = digitalRead(kDigitalPinsLimitSwitch[1]);
  if (val2 == HIGH)
  {
    Serial.println("Right switch is 1");
  }
  else
  {
    Serial.println("Right switch is 0");
  }
}

void Sensors::initSwitches()
{
  pinMode(kDigitalPinsLimitSwitch[0], INPUT);
  pinMode(kDigitalPinsLimitSwitch[1], INPUT);
}

void Sensors::setRosBridge(RosBridge2 *rosBridge)
{
  this->rosBridge = rosBridge;
}

// LED Methods

void Sensors::toggleRightLed()
{
  if (rightLedOn)
  {
    rightLedOn = false;
    digitalWrite(kDigitalPinsLEDS[1], LOW);
  }
  else
  {
    rightLedOn = true;
    digitalWrite(kDigitalPinsLEDS[1], HIGH);
  }
}

void Sensors::turnRightLedOn()
{
  rightLedOn = true;
  digitalWrite(kDigitalPinsLEDS[1], HIGH);
}
void Sensors::turnRightLedOff()
{
  rightLedOn = false;
  digitalWrite(kDigitalPinsLEDS[1], LOW);
}

void Sensors::toggleLeftLed()
{
  if (leftLedOn)
  {
    leftLedOn = false;
    digitalWrite(kDigitalPinsLEDS[0], LOW);
  }
  else
  {
    leftLedOn = true;
    digitalWrite(kDigitalPinsLEDS[0], HIGH);
  }
}

void Sensors::toggleBothLeds()
{
  toggleLeftLed();
  toggleRightLed();
}

void Sensors::bothLedOn()
{
  leftLedOn = true;
  rightLedOn = true;
  digitalWrite(kDigitalPinsLEDS[0], HIGH);
  digitalWrite(kDigitalPinsLEDS[1], HIGH);
}

void Sensors::bothLedOff()
{
  leftLedOn = false;
  rightLedOn = false;
  digitalWrite(kDigitalPinsLEDS[0], LOW);
  digitalWrite(kDigitalPinsLEDS[1], LOW);
}

bool Sensors::isValid(double d)
{
  if (d < 0.1 || d > 5)
  {
    return false;
  }
  return true;
}

// High level methods to interact with oled.
void Sensors::logActive(String s, bool oled, int x, int y, bool absolute)
{
  bool trace = true;
  if (absolute)
    return;
  if (CK::debugOled && oled)
    screen.display(s, x, y);
  if (trace)
    screen.display(s, x, 8);
  if (!CK::kusingROS)
    Serial.println(s);
}

void Sensors::logActive(String s, double n, String divider, bool oled, int x, int y)
{
  String newMsg = s + divider + String(n);
  logActive(newMsg, oled, x, y);
}

void Sensors::logActive(double n, String s, String divider, bool oled, int x, int y)
{
  String newMsg = String(n) + divider + s;
  logActive(newMsg, oled, x, y);
}

void Sensors::resetScreen()
{
  screen.resetScreen();
}