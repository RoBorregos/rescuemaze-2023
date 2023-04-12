#ifndef GeneralChecks_h
#define GeneralChecks_h

#include "Movement.h"
#include "Sensors.h"
#include "Motor.h"
#include "MotorID.h"
#include "MUX2C.h"
#include "BNO.h"
#include "Plot.h"
#include "CommonK.h"


// Class used to perform quick tests on sensors and basic unitary movements of robot.
class GeneralChecks
{
private:
    Movement *robot;
    bool publishRos;

public:
    // Constructor
    // @param *robot Pointer to the Movement instance that will be used to control the robot.
    // @param publishRos True if the user wants logs in ROS.
    GeneralChecks(Movement *robot, bool publishRos = false);

    // Method used to calibrate tcs sensor ad find
    void calibrateSensors();

    // Check that VLX, TCS, and BNO are working correctly.
    void checkSensorData(int iterations=50);

    // Test that all motors are registered correctly (motor[0] is actually front left, etc),
    // as well as their directions.
    void checkWheelDirections();

    // Run basic unitary movements using cmd Movement.
    void checkUnitaryMovements();

    // Run the basic tests.
    void checkAll();

    // Helper function to log messages.
    void log(const char *s, bool newLine = true);

    // Do specific tests. Use to avoid polluting main_code.ino and seize access as friend class.
    void test();
    
    // End script execution. Used for testing.
    void end();
};

#endif