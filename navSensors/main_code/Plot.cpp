#include "Plot.h"

// Constructor

Plot::Plot(Movement *moveAll)
{
    this->moveAll = moveAll;
    timeMsg = millis();
}

// Plot Functions

void Plot::plotMotorSpeed()
{
    plotData(
        moveAll->motor[BACK_LEFT].getCurrentSpeed(),
        moveAll->motor[FRONT_LEFT].getCurrentSpeed(),
        moveAll->motor[BACK_RIGHT].getCurrentSpeed(),
        moveAll->motor[FRONT_RIGHT].getCurrentSpeed(),
        moveAll->motor[FRONT_RIGHT].getTargetRps(moveAll->motor[FRONT_RIGHT].getTargetSpeed())
    );
}

void Plot::plotTargetandCurrent()
{
    plotData(
        moveAll->motor[BACK_LEFT].getCurrentSpeed(),
        moveAll->motor[FRONT_LEFT].getCurrentSpeed(),
        moveAll->motor[BACK_RIGHT].getCurrentSpeed(),
        moveAll->motor[FRONT_RIGHT].getCurrentSpeed(),
        moveAll->motor[FRONT_LEFT].getTargetSpeed()
    );
}



void Plot::plotData(const double data1, const double data2, const double data3, const double data4, const double data5)
{
    if (millis() - timeMsg < 35)
    {
        return;
    }
    const byte *byteData1 = (byte *)(&data1);
    const byte *byteData2 = (byte *)(&data2);
    const byte *byteData3 = (byte *)(&data3);
    const byte *byteData4 = (byte *)(&data4);
    const byte *byteData5 = (byte *)(&data5);

    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};

    Serial.write(buf, 20);

    timeMsg = millis();
}

void Plot::startSequence(){
    Serial.write("<target>"); // Sequence to recognize at "multiplePlots.py"
}