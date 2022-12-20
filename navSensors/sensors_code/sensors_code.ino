#include "MLX.h"
#include "MUX2C.h"
#include "VLX.h"
#include "TCS.h"
#include "BNO.h"
#include "Movement.h"


#define CANAL_TEMP 0
#define POS_COLOR 2
#define POS_MLX 4
#define POS_BNO 2
#define POS_VLX 1

#define DELAY_MS 100
#define ITERATIONS 5

MLX sensorTemp;
TCS sensorColor;
VLX sensorDistance;
Movement *robot = nullptr;
BNO bno;
MUX2C mux;

int reps;

void setup() {
  Serial.begin(9600);
  Serial.println("Ejecutando setup");

  //setSensors();
  mux.findI2C();
}

void loop () {
  return;
  if (reps == ITERATIONS)
    return;
  // testTemperatura();
  // testDistance();
  // testBNO();   
  // testVLX();
  testMLX();
  delay(DELAY_MS);
  reps++;
}

void setSensors() {
  setMLX();
  //sensorColor = setColorSensor();
  //setBNO();
  //setVLX();
}

void setMLX(){
  sensorTemp.setMux(POS_MLX);
}

void setVLX(){
  sensorDistance.setMux(POS_VLX);
  sensorDistance.init();
}

void setBNO() {
  bno.setMux(POS_BNO);
  bno.init();
}

void testTemperatura() {
  sensorTemp.printTemp();
  delay(DELAY_MS);
}

void testVLX() {
  sensorDistance.printDistance();
}

void testBNO() {
  bno.anglesInfo();
}

void testMLX(){
  sensorTemp.printTemp();
}

TCS setColorSensor() {
  uint8_t precision = 10;

  // Colorlist, and colors[] must be static to prevent dangling pointers.
  // Otherwise, an implementation of TCS class with dynamic memory allocation is needed.
  
  static char colorList[] = {"wgbgbnr"};
  static constexpr uint8_t colorAmount = sizeof(colorList)/sizeof(colorList[0]) - 1;
  
  static uint8_t colors[colorAmount][3] = {
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
