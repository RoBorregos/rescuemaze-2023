#ifndef GeneralChecks_h
#define GeneralChecks_h

#include "Movement.h"
#include "MUX2C.h"

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
    void checkSensorData();

    // Test that all motors are registered correctly (motor[0] is actually front left, etc),
    // as well as their directions.
    void checkWheelDirections();

    // Run basic unitary movements using cmd Movement.
    void checkUnitaryMovements();

    // Run the basic tests.
    void checkAll();

    // Helper function to log messages.
    void log(const char *s, bool newLine = true);
};

#endif