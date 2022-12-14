#include "MLX.h"
#include "MUX2C.h"
#include "VLX.h"
#include "TCS.h"
#include "BNO.h"


#define CANAL_TEMP 0
#define POS_COLOR 2
#define POS_TEMP 1

#define DELAY_MS 1000

MLX sensorTemp;
TCS sensorColor;
VLX sensorDistance(CANAL_TEMP);

void setup(){
    Serial.begin(9600);
    Serial.println("Ejecutando setup");
    sensorTemp = MLX(CANAL_TEMP);
    //sensorColor = setColorSensor();
    setDistanceSensor();
}

void loop (){
    testTemperatura();
    testDistance();
}

void testTemperatura(){
    sensorTemp.printTemp();
    delay(DELAY_MS);
}

void testDistance(){
  sensorDistance.printDistance();
  delay(DELAY_MS);
}

TCS setColorSensor(){
  uint8_t precision = 10;
  char colorList[] = {"wgbgbnr"};
  uint8_t colorAmount = (sizeof(colorList) / sizeof(colorList[0]));
  
  uint8_t colors[colorAmount][COLUMNAS] = {
                          {19, 31, 58},
                          {28, 120, 82},
                          {16, 64, 31},
                          {73, 132, 73},
                          {110, 70, 108},
                          {155, 111, 57},
                          {220, 156, 150}
                          };
  
  //TCS::TCS(uint8_t posMux, uint8_t precision, uint8_t colors[][3], uint8_t colorAmount, char colorList[])
  TCS sensor(POS_COLOR, precision, colors, colorAmount, colorList);
  sensor.init();
  return sensor;
}

void setDistanceSensor(){
  sensorDistance.init();
}
