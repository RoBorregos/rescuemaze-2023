#include "MLX.h"
#include "MUX2C.h"
#include "VLX.h"
#include "TCS.h"
#include "BNO.h"
#include "Movement.h"


#define CANAL_TEMP 0
#define POS_COLOR 2
#define POS_TEMP 1
#define POS_BNO 3

#define DELAY_MS 100

MLX sensorTemp;
TCS sensorColor;
VLX sensorDistance(CANAL_TEMP);
Movement *robot = nullptr;
BNO bno;

void setup() {
  Serial.begin(115200);
  Serial.println("Ejecutando setup");

  setSensors();
}

void loop () {
  // testTemperatura();
  // testDistance();
  testBNO();
  delay(DELAY_MS);
}

void setSensors() {
  sensorTemp = MLX(CANAL_TEMP);
  //sensorColor = setColorSensor();
  //setDistanceSensor();
  setBNO();
}

void setBNO() {
  bno.setMux(POS_BNO);
  bno.init();
}

void testTemperatura() {
  sensorTemp.printTemp();
  delay(DELAY_MS);
}

void testDistance() {
  sensorDistance.printDistance();
  delay(DELAY_MS);
}

void testBNO() {
  bno.anglesInfo();
}

TCS setColorSensor() {
  uint8_t precision = 10;
  char colorList[] = {"wgbgbnr"};
  uint8_t colorAmount = (sizeof(colorList) / sizeof(colorList[0]));

  uint8_t colors[colorAmount][3] = {
    {19, 31, 58},
    {28, 120, 82},
    {16, 64, 31},
    {73, 132, 73},
    {110, 70, 108},
    {155, 111, 57},
    {220, 156, 150}
  };

  sensorColor.setPrecision(precision);
  sensorColor.setMux(POS_COLOR);
  sensorColor.init(colors, colorAmount, colorList);
}

void setDistanceSensor() {
  sensorDistance.init();
}
