#include "Sensors.h"

// Constructor

Sensors::Sensors(BNO *bno) : bno(bno)
{
  initSensors();
}

// Initialization
void Sensors::initSensors()
{

  for (int i = 0; i < kVLXCount; i++)
  {
    vlx[i].setMux(kMuxVLX[i]);
  }

  for (int i = 0; i < kMLXCount; i++)
  {
    mlx[i].setMux(kMuxMLX[i]);
  }

  tcs.setMux(kMuxTCS);
  tcs.setPrecision(kTCSPrecision);

  Wire.begin();

  // BNO init (Not needed, because initialized before sending pointer)

  // TCS init
  tcs.init(colors, colorAmount, colorList);

  // VLX init
  for (int i = 0; i < kVLXCount; i++)
  {
    vlx[i].init();
  }

  // MLX init
  for (int i = 0; i < kMLXCount; i++)
  {
    mlx[i].init();
  }
}

// Sensor Methods

void Sensors::printInfo()
{

  bno->anglesInfo();

  for (int i = 0; i < kVLXCount; i++)
  {
    Serial.print("VLX sensor ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(float(getVLXInfo(i)), 4);
  }

  for (int i = 0; i < kMLXCount; i++)
  {
    Serial.print("MLX sensor ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(getMLXInfo(i));
  }

  Serial.print("TCS sensor  ");
  tcs.printRGB();
  tcs.printColor();
}

float Sensors::getVLXInfo(int posVLX)
{
  if (posVLX > 0 && posVLX < kVLXCount)
    return vlx[posVLX].getDistance();

  Serial.println("Invalid position for VLX sensor at getVLXInfo().");
  return -1;
}

float Sensors::getMLXInfo(int posMLX)
{
  if (posMLX > 0 && posMLX < kMLXCount)
    return mlx[posMLX].getTemp();

  Serial.println("Invalid position for MLX sensor at getMLXInfo().");
  return -1;
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
