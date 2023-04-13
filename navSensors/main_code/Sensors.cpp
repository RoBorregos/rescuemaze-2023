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
  tcs.init(colors, colorAmount, colorList);

  initSwitches();

  // BNO init (Not needed, because initialized before sending pointer)
}

// Sensor Methods

void Sensors::printInfo(bool bno, bool vlx, bool tcs, bool limitSwitches)
{
  if (bno && usingBNO)
    this->bno->anglesInfo();

  if (vlx && usingVLX && !CK::kusingROS)
  {
    for (int i = 0; i < kMuxVLX; i++)
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
    this->tcs.printColor();
  }

  if (limitSwitches)
    debugLimitSwitches();
}

float Sensors::getVLXInfo(int posVLX)
{
  if (!usingVLX && !CK::kusingROS)
  {
    Serial.println("WARNING: invalid method call, VLX marked as not used.");
    return -1;
  }

  return vlx[posVLX].getDistance();
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
  // return tcs.getColorWithThresholds();
  return tcs.getColorWithPrecision();
}

void Sensors::bnoAngles(float &x, float &y, float &z)
{
  if (!usingBNO && !CK::kusingROS)
  {
    Serial.println("WARNING: invalid method call, BNO marked as not used.");
    return;
  }
  bno->getAll(x, y, z);
}

void Sensors::bnoPrint()
{
  if (!usingBNO && !CK::kusingROS)
  {
    Serial.println("WARNING: invalid method call, BNO marked as not used.");
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
    Serial.println("Switch 0 is open");
  }
  else
  {
    Serial.println("Switch 0 is closed");
  }
  int val2 = digitalRead(kDigitalPinsLimitSwitch[1]);
  if (val2 == HIGH)
  {
    Serial.println("Switch 1 is open");
  }
  else
  {
    Serial.println("Switch 1 is closed");
  }
}

void Sensors::initSwitches()
{
  pinMode(kDigitalPinsLimitSwitch[0], INPUT);
  pinMode(kDigitalPinsLimitSwitch[1], INPUT);
}
