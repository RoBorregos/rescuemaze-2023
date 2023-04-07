#include "GeneralChecks.h"

GeneralChecks::GeneralChecks(Movement *robot, bool publishRos) : robot(robot), publishRos(publishRos)
{
}

void GeneralChecks::checkAll()
{
    if (publishRos)
    {
        while (robot->nh->connected())
        {
            robot->nh->spinOnce();
        }

        robot->nh->loginfo("Running General Checks in Arduino.");
    }
    else
    {
        Serial.println("Running general checks in Arduino.");
    }
    checkSensorData();
    checkWheelDirections();
    checkUnitaryMovements();
}

void GeneralChecks::calibrateSensors()
{
    // Only calibrate directly on arduino, because the code with new ranges
    // would still need to be uploaded.
    if (publishRos)
    {
        if (robot->nh == nullptr)
            return;
        robot->nh->loginfo("Calibrate sensors only available directly on Arduino.");
        robot->nh->loginfo("Change publishROS flag and connect to serial monitor or arduino IDE.");
        return;
    }

    MUX2C mux;

    // Print available i2c directions
    mux.findI2C();

    // Re-initialize sensors because mux.findI2C cuts communication.
    Sensors s;
    s.initSensors();
    while (1)
    {
        // Only print TCS info for calibration.
        s.printInfo(false, false, true);
        delay(200);
    }
}

void GeneralChecks::checkSensorData()
{
    // If information is being published to ROS, simply echo individual topics.
    if (!publishRos)
    {
        Serial.println("BNO information: ");
        for (int i = 0; i < 20; i++)
        {
            robot->sensors->printInfo(true, false, false);
            delay(100);
        }

        Serial.println("VLX information: \n");

        for (int i = 0; i < robot->sensors->kMuxVLX; i++)
        {
            Serial.print(robot->sensors->vlxNames[i]);
            Serial.println(" vlx:");
            for (int j = 0; j < 10; j++)
            {
                robot->sensors->getVLXInfo(i);
                delay(100);
            }
        }

        Serial.println("TCS information: \n");
        for (int i = 0; i < 10; i++)
        {
            Serial.print("Character: ");
            Serial.println(robot->sensors->getTCSInfo());
            delay(100);
        }

        Serial.println("Limit switches:\n");

        int right = 0, left = 0;
        for (int i = 0; i < 20; i++)
        {
            robot->sensors->getLimitSwitches(right, left);
            Serial.print("Right: ");
            Serial.print(right);
            Serial.print(", Left: ");
            Serial.println(left);
            delay(100);
        }
    }
}

void GeneralChecks::checkWheelDirections()
{
    log("Testing all motors");
    delay(1000);

    log("FRONT RIGHT - Forward");
    robot->motor[FRONT_RIGHT].motorSpeedPID(90);
    delay(2000);
    robot->motor[FRONT_RIGHT].motorSpeedPID(0);
    delay(1000);

    log("FRONT RIGHT - Backwards");
    robot->motor[FRONT_RIGHT].motorSpeedPID(-90);
    delay(2000);
    robot->motor[FRONT_RIGHT].motorSpeedPID(0);
    delay(1000);

    log("FRONT LEFT - Forward");
    robot->motor[FRONT_LEFT].motorSpeedPID(90);
    delay(2000);
    robot->motor[FRONT_LEFT].motorSpeedPID(0);
    delay(1000);

    log("FRONT LEFT - Backwards");
    robot->motor[FRONT_LEFT].motorSpeedPID(-90);
    delay(2000);
    robot->motor[FRONT_LEFT].motorSpeedPID(0);
    delay(1000);

    log("BACK LEFT - Forward");
    robot->motor[BACK_LEFT].motorSpeedPID(90);
    delay(2000);
    robot->motor[BACK_LEFT].motorSpeedPID(0);
    delay(1000);

    log("BACK LEFT - Backwards");
    robot->motor[BACK_LEFT].motorSpeedPID(-90);
    delay(2000);
    robot->motor[BACK_LEFT].motorSpeedPID(0);
    delay(1000);

    log("BACK RIGHT - Forward");
    robot->motor[BACK_RIGHT].motorSpeedPID(90);
    delay(2000);
    robot->motor[BACK_RIGHT].motorSpeedPID(0);
    delay(1000);

    log("BACK RIGHT - Backwards");
    robot->motor[BACK_RIGHT].motorSpeedPID(-90);
    delay(2000);
    robot->motor[BACK_RIGHT].motorSpeedPID(0);
    delay(1000);
}

// TODO: implement function using cmdMovement.
void GeneralChecks::checkUnitaryMovements()
{
}

void GeneralChecks::log(const char *s, bool newLine)
{
    if (publishRos)
    {
        robot->nh->loginfo(s);
    }
    else
    {
        if (newLine)
            Serial.println(s);
        else
            Serial.print(s);
    }
}