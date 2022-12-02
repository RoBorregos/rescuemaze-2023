#include "MLX.h"
#include "MUX2C.h"

#define CANAL_TEMP 1
#define DELAY_MS 1000

MLX sensorTemp;

void setup(){
    Serial.begin(57600);
    Serial.println("Ejecutando setup");
    sensorTemp = MLX(CANAL_TEMP);
}

void loop (){
    testTemperatura();
}

void testTemperatura(){
    sensorTemp.printTemp();
    delay(DELAY_MS);
}