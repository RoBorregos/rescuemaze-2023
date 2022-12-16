#ifndef Sensors_h
#define Sensors_h

#include <Wire.h>
#include "MUX2C.h"
#include "BNO.h"
#include "MLX.h"
#include "VLX.h"
#include "TCS.h"


// SENSOR DATA
#define vlx_right 0
#define vlx_left 1
#define vlx_front 2
#define vlx_back 3
#define mlx_right 0
#define mlx_left 1

class Sensors{
 // Sensors Count
    static constexpr int kBNOCount = 10; // BNO sensor data count
    static constexpr int kVLXCount = 4; // VLX sensors data count
    static constexpr int kMLXCount = 2; // MLX sensors data count
    static constexpr int kTCSCount = 1; // TCS sensors data count
    static constexpr int kSensorCount = kBNOCount + kVLXCount + kMLXCount + kTCSCount; // Total sensors data count
    
    // Sensors.
    BNO *bno_;
    MLX mlx_[kMLXCount];
    VLX vlx_[kVLXCount];
    TCS tcs_;

    // Sensor Pins.
    int kMuxVLX[kVLXCount] = {7,1,5,3}; // VLX multiplexor pins
    int kMuxMLX[kMLXCount] = {0,4}; // MLX multiplexor pins
    int kMuxTCS = 6;  // TCS multiplexor pin

  public:
    ////////////////////////////////////////Constructor//////////////////////////////////////////////////////
    Sensors(BNO *bno);

    /////////////////////////////////////////Initialization/////////////////////////////////////////////////////////
    void initSensors();

    ////////////////////////////////////////Sensor Methods/////////////////////////////////////////////////////////

    ///////////////////////////////////////////BNO data///////////////////////////////////////////////////////////
    // Returns the BNO x axis data for quatertion.
    float getQuatX();

    // Returns the BNO y axis data for quatertion.
    float getQuatY();

    // Returns the BNO z axis data for quatertion.
    float getQuatZ();

    // Returns the BNO w axis data for quatertion.
    float getQuatW();

    // Returns the BNO x axis data for angular velocity.
    float getAngVelX();

    // Returns the BNO y axis data for angular velocity.
    float getAngVelY();

    // Returns the BNO z axis data for angular velocity.
    float getAngVelZ();

    // Returns the BNO x axis data for Linear acceleration.
    float getLinAccX();

    // Returns the BNO y axis data for Linear acceleration.
    float getLinAccY();

    // Returns the BNO z axis data for Linear acceleration.
    float getLinAccZ();

    // Returns the BNO x axis data for angle.
    float getAngleX();

    // Returns the BNO y axis data for angle.
    float getAngleY();

    // Returns the BNO z axis data for angle.
    float getAngleZ();

    // Returns the VLX distance depending on the position of the VLX sensor
    float getVLXInfo(int posVLX);

    // Returns the MLX temperature depending on the position of the MLX sensor
    float getMLXInfo(int posMLX);
    
    // Returns the TCS color detected
    char getTCSInfo();

    // Prints all sensors information in the serial monitor.
    void printInfo();
};



#endif
