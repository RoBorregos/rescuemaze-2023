#ifndef TCS_H
#define TCS_H

#include "Adafruit_TCS34725.h"
#include "MUX2C.h"

#define TCSADDR 0x29
#define COLORES 3
#define COLUMNAS 3

// Sensor de color

class TCS
{
private:
  Adafruit_TCS34725 tcs = Adafruit_TCS34725();
  float red;
  float green;
  float blue;
  MUX2C mux;
  uint8_t tcaPos;
  uint8_t precision;
  char colorList[COLORES] = {'w', 'g', 'b'}; // arreglo con iniciales de colores. 

  // Matriz con colores detectados con sensor
  // rows: filas numero de colores
  // columnas: cantidades r g b detectadas
  uint8_t colors[COLORES][COLUMNAS];
  
  /*
  para colorList[i], 
    colors[i][0] = cantidad de rojo;
    colors[i][1] = cantidad de verde;
    colors[i][2] = cantidad de azul;
  detectado al registrar ese color manualmente
  */

  bool inRange(uint8_t color, uint8_t colorRegistered);
  void setDefValues();

public:
  TCS();
  TCS(uint8_t posMux);
  TCS(uint8_t posMux, uint8_t precision);
  TCS(uint8_t posMux, uint8_t precision);
  TCS(uint8_t posMux, uint8_t precision, uint8_t colors[COLORES][COLUMNAS]);
  TCS(uint8_t posMux, uint8_t precision, uint8_t colors[COLORES][COLUMNAS], char colorList[]);

  void init();
  void setMux(uint8_t posMux);
  void setPrecision(uint8_t precision);

  void printRGB();
  void updateRGB();
  char getColor();
  char getColorWithPrecision();
};

#endif
