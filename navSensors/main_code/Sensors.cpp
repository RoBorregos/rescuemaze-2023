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

    // VLX init
    for (int i = 0; i < kMuxVLX; i++)
    {
      vlx[i].init();
    }
  }

  // TCS init
  tcs.setMux(kMuxTCS);
  tcs.setPrecision(kTCSPrecision);
  Wire.begin();
  tcs.init(colors, colorAmount, colorList, colorThresholds);

  initSwitches();
  initLeds();

  if (CK::calibrateBNO)
  {
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
      if (!CK::kusingROS && CK::debugBNOCalibration)
        bno->displayCalStatus();

      if (millis() - initialT > maxBNOTime)
      {
        /*
        if (!CK::kusingROS && CK::debugBNOCalibration)
          Serial.println("BNO calibration timed out.");
*/
        // Led blink to indicate that BNO calibration timed out.
        bothLedOff();
        delay(100);

        bothLedOn();
        delay(100);

        bothLedOff();
        delay(100);
        break;
      }
    }

    if (!CK::kusingROS && CK::debugBNOCalibration)
    {
      /*
      if (bno->isCalibrated())
        Serial.println("BNO calibration finished.");
      else
        Serial.println("BNO calibration failed.");
        */
    }

    // Give some time to place robot on the ground. The initial position will be
    // considered as north.
    double timeToPlaceRobot = 3000; // 3 seconds
    initialT = millis();
    toggleRightLed();

    while (millis() - initialT < timeToPlaceRobot)
    {
      toggleBothLeds();
      if (!CK::kusingROS && CK::debugBNOCalibration)
      {
        /*
        Serial.print("Place robot on the ground in ");
        Serial.print((timeToPlaceRobot - (millis() - initialT)) / 1000);
        Serial.println(" seconds.");
        */
      }
      delay(100);
    }

    bothLedOff();
  }
  else
  {
    bno->setExtCUse();
  }
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
    for (int i = 0; i < 3; i++)
    {
      // Serial.print(" VLX sensor ");
      // Serial.print(i + 1);
      // Serial.print(": ");
      // Serial.print(float(getVLXInfo(i)), 4);
    }
    // Serial.println();
  }

  if (tcs && !CK::kusingROS)
  {
    // Serial.print("TCS sensor  ");
    this->tcs.printRGB();
    // Serial.println(getTCSInfo());
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
  if (this->rosBridge == nullptr || !usingLidar)
    return;
  // Values weren't updated
  if (wallDistances[0] == front && wallDistances[1] == back && wallDistances[2] == left && wallDistances[3] == right)
  {
    lidarAttemptCount++;
    if (lidarAttemptCount > 15){
      // Lidar is not working, use vlx
      usingLidar = false;
      return;
    }
    // Call updateDistances again
    rosBridge->updateDistLidar();
    return;
  }
  // 19*18 
  
  // Update values adding constant error
  wallDistances[0] = front - 19 / 2;
  wallDistances[1] = back - 19 / 2;
  wallDistances[2] = left - 18 / 2;
  wallDistances[3] = right - 18 / 2;
  lidarAttemptCount = 0;
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

bool Sensors::readMotorInit(){
  
  int val = digitalRead(22);
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
  if (usingLidar && rosBridge != nullptr && direction != 2 && direction != 3)
  {
    float front, back, left, right;
    rosBridge->updateDistLidar();

    if (direction >= 0 && direction <= 3 && usingLidar)
      return wallDistances[direction];
  }

  switch(direction){
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

void Sensors::getLimitSwitches(int &right, int &left)
{
  right = rightLimitSwitch();
  left = leftLimitSwitch();
}

int Sensors::leftLimitSwitch()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[0]);

  return val == HIGH;
}

int Sensors::rightLimitSwitch()
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
    // Serial.println("Switch 0 is open");
  }
  else
  {
    // Serial.println("Switch 0 is closed");
  }
  int val2 = digitalRead(kDigitalPinsLimitSwitch[1]);
  if (val2 == HIGH)
  {
    // Serial.println("Switch 1 is open");
  }
  else
  {
    // Serial.println("Switch 1 is closed");
  }
}

void Sensors::initSwitches()
{
  pinMode(kDigitalPinsLimitSwitch[0], INPUT);
  pinMode(kDigitalPinsLimitSwitch[1], INPUT);
}

void Sensors::setRosBridge(RosBridge *rosBridge)
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