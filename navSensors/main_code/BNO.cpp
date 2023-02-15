#include "BNO.h"

BNO::BNO()
{
}

void BNO::init()
{
  if (!bno.begin())
  {
    
    Serial.println("ERROR BNO");
  }

  bno.setExtCrystalUse(true);
}

float BNO::getAngleX()
{
  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.x;
}

float BNO::getAngleY()
{
  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.y;
}

float BNO::getAngleZ()
{
  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.z;
}

void BNO::anglesInfo()
{
  float x{}, y{}, z{};
  getAll(x, y, z);

  Serial.print(F("X = "));
  Serial.print(x, 4);
  Serial.print(F(", Y = "));
  Serial.print(y, 4);
  Serial.print(F(", Z = "));
  Serial.println(z, 4);
}

void BNO::getAll(float &x, float &y, float &z)
{
  sensors_event_t event;
  bno.getEvent(&event);

  x = event.orientation.x;
  y = event.orientation.y;
  z = event.orientation.z;
}

void BNO::updateEvents()
{
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat = bno.getQuat();
}

float BNO::getQuat_x()
{
  updateEvents();
  return quat.x();
}

float BNO::getQuat_y()
{
  updateEvents();
  return quat.y();
}

float BNO::getQuat_z()
{
  updateEvents();
  return quat.z();
}

float BNO::getQuat_w()
{
  updateEvents();
  return quat.w();
}

float BNO::getAngVel_x()
{
  updateEvents();
  return angVelocityData.gyro.x;
}

float BNO::getAngVel_y()
{
  updateEvents();
  return angVelocityData.gyro.y;
}

float BNO::getAngVel_z()
{
  updateEvents();
  return angVelocityData.gyro.z;
}

float BNO::getLinAcc_x()
{
  updateEvents();
  return linearAccelData.acceleration.x;
}

float BNO::getLinAcc_y()
{
  updateEvents();
  return linearAccelData.acceleration.y;
}

float BNO::getLinAcc_z()
{
  updateEvents();
  return linearAccelData.acceleration.z;
}

// Funciones sacadas del ejemplo de uso de Adafruit.

void BNO::displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}

void BNO::displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
}
