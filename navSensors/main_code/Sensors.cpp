#include "Sensors.h"

// Definitions of non-int static constexpr variables. (Declarations and initialization in .h)

constexpr char Sensors::colorList[];
constexpr uint8_t Sensors::colors[Sensors::colorAmount][3];

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
    vlx.setMux(kMuxVLX);
    vlx.init(); // VLX init
  }

  // TCS init
  tcs.setMux(kMuxTCS);
  tcs.setPrecision(kTCSPrecision);
  Wire.begin();
  tcs.init(colors, colorAmount, colorList);

  // BNO init (Not needed, because initialized before sending pointer)
}

// Sensor Methods

void Sensors::printInfo(bool bno, bool vlx, bool tcs)
{
  if (bno && usingBNO)
    this->bno->anglesInfo();

  if (vlx && usingVLX)
  {
    Serial.print("VLX sensor: ");
    Serial.println(float(getVLXInfo(0)), 4);
  }

  if (tcs)
  {
    Serial.print("TCS sensor  ");
    this->tcs.printRGB();
    this->tcs.printColor();
  }
}

float Sensors::getVLXInfo(int posVLX)
{
  if (!usingVLX)
  {
    Serial.println("WARNING: invalid method call, VLX marked as not used.");
    return -1;
  }

  return vlx.getDistance();
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
  return tcs.getColorWithPrecision();
}

void Sensors::bnoAngles(float &x, float &y, float &z)
{
  if (!usingBNO)
  {
    Serial.println("WARNING: invalid method call, BNO marked as not used.");
    return;
  }
  bno->getAll(x, y, z);
}

void Sensors::bnoPrint()
{
  if (!usingBNO)
  {
    Serial.println("WARNING: invalid method call, BNO marked as not used.");
    return;
  }
  bno->anglesInfo();
}

void Sensors::rgbTCS(){
  tcs.printRGB();
}

void Sensors::checkTCS()
{
  tcs.printColorMatrix();
  tcs.printColorList();
}
