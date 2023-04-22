// This class has all the functions related to the ROS connection. It recieves and
// sends data through serial.
#ifndef RosBridge2_h
#define RosBridge2_h

#include <stdint.h>
#include <math.h>
#include "BNO.h"
#include <Arduino.h>
#include "Movement.h"

inline int sign(int a) { return min(1, max(-1, a)); };

class Sensors;  // Forward declaration of Sensors
class Movement; // Forward declaration of Movement

class RosBridge2
{
public:
    //////////////////////////////////Constructor//////////////////////////////////////
    RosBridge2(Movement *robot, Sensors *sensors, BNO *bno);

    //////////////////////////////////Run//////////////////////////////////////
    // Calls publish and verify it is still receiving commands.
    void run();

    // Reads until Serial.available() is 0.
    void readOnce();

private:
    //////////////////////////////////Velocity Suscriber//////////////////////////////////////
    // Receives movement commands.
    void cmdMovementCallback(int move);

    // Updates new distances to walls.
    void updateDistLidar(float front, float back, float left, float right);

    // Callback to disense kits.
    void callDispenser(int victims);

    void readSerial();

    bool readLidar();

    Movement *robot_;
    BNO *bno_;
    Sensors *sensors_;
    int state_ = -1;
    int cmdCounter[7] = {0, 0, 0, 0, 0, 0, 0};
    int goal = 0;
    long int cmdCounterT = 0;

    // Suscriber.
    static constexpr uint16_t kWatchdogPeriod = 500;

    // Timers.
    unsigned long odom_timer_ = 0;
    unsigned long watchdog_timer_ = 0;

    void executeCommand(uint8_t packet_size, uint8_t command, uint8_t *buffer);
    void writeSerial(bool success, uint8_t *payload, int elements);
    void readUntilLidar();
};

#endif
