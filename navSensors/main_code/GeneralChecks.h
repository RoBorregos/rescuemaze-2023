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
//#include "SingleEMAFilterLib.h"


// Class used to perform quick tests on sensors and basic unitary movements of robot.
class GeneralChecks
{
private:
    Movement *robot;
    bool publishRos;

    double dtToS(long int dt);

public:
    // Constructor
    // @param *robot Pointer to the Movement instance that will be used to control the robot.
    // @param publishRos True if the user wants logs in ROS.
    GeneralChecks(Movement *robot, bool publishRos = false);

    // Method used to calibrate tcs sensor ad find
    void calibrateSensors();

    void printRevolutions();

    // Check that VLX, TCS, and BNO are working correctly.
    void checkSensorData(int iterations=50);

    // Test that all motors are registered correctly (motor[0] is actually front left, etc),
    // as well as their directions.
    void checkWheelDirections();

    // Run basic unitary movements using cmd Movement.
    void checkUnitaryMovements();
    
    // Tests in oled.
    void checkOled();

    void checkWheelSpeed();

    // Print the time it takes to execute commands.
    void checkDT();

    // Run the basic tests.
    void checkAll();

    // Helper function to log messages.
    void log(String s, bool newLine = true);

    // Check PID constants and compare them with target velocity.
    void checkPID();

    // Do specific tests. Use to avoid polluting main_code.ino and seize access as friend class.
    void test();
    
    // End script execution. Used for testing.
    void end();
};

#endif