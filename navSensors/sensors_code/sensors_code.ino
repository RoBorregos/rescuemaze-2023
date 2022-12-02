#include <MLX.h>
#include "../MUX2C.h"
#include "../VLX.h"
#include "../TCS.h"
#include "../BNO.h"


#define CANAL_TEMP 0
#define POS_COLOR 2;
#define POS_TEMP 1;

#define DELAY_MS 1000

MLX sensorTemp;
TCS sensorColor;

void setup(){
    Serial.begin(57600);
    Serial.println("Ejecutando setup");
    sensorTemp = MLX(CANAL_TEMP);
    sensorColor = setColorSensor();
}

void loop (){
    testTemperatura();
}

void testTemperatura(){
    sensorTemp.printTemp();
    
    delay(DELAY_MS);
}

TCS setColorSensor(){
  uint8_t precision = 10;
  char colorList[COLORES] = {"wgb"};
  uint8_t colors[COLORES][COLUMNAS] = {
                          {19, 31, 58},
                          {28, 120, 82},
                          {16, 64, 31},
                          {73, 132, 73},
                          {110, 70, 108},
                          {155, 111, 57},
                          {220, 156, 150}
                          };

  return TCS(POS_COLOR, precision, colors[COLORES][COLUMNAS], colorList);
}
