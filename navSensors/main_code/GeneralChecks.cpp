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
        s.printInfo(false, false, true, false);
        delay(200);
    }
}

void GeneralChecks::checkSensorData(int iterations)
{
    // If information is being published to ROS, simply echo individual topics.
    if (!publishRos && !CK::kusingROS)
    {
        Serial.println("BNO information: ");
        for (int i = 0; i < iterations; i++)
        {
            robot->sensors->printInfo(true, false, false, false);
            delay(100);
        }

        Serial.println("VLX information: \n");

        for (int i = 0; i < robot->sensors->kMuxVLX; i++)
        {

            for (int j = 0; j < iterations; j++)
            {
                Serial.print(robot->sensors->vlxNames[i]);
                Serial.print(" vlx: ");
                Serial.println(robot->sensors->getVLXInfo(i));
                delay(100);
            }
        }

        Serial.println("\n\nTCS information: \n");
        for (int i = 0; i < iterations; i++)
        {
            Serial.print("Color: ");
            Serial.println(robot->sensors->getTCSInfo());
            delay(100);
        }

        Serial.println("Limit switches:\n");

        int right = 0, left = 0;
        for (int i = 0; i < iterations; i++)
        {
            robot->sensors->getLimitSwitches(right, left);
            Serial.print("Right Limit switch: ");
            Serial.print(right);
            Serial.print(", Left Limit switch: ");
            Serial.println(left);
            delay(100);
        }

        Serial.println("Finished checking sensors.");
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
    if (publishRos && CK::kusingROS)
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

void GeneralChecks::test()
{
    while (true)
    {
        if (!CK::kusingROS)
            Serial.println(robot->sensors->getTCSInfo());
        // s->rgbTCSClear();
        // s->rgbTCS();
        // delay(500);
        // s->printInfo(false, false, true, false);
    }

    for (int i = 0; i < 1000; i++)
    {
        robot->cmdMovement(1, 1);
        delay(100);
        robot->cmdMovement(5);
        delay(100);
        robot->cmdMovement(4, 1);
        delay(100);
        robot->cmdMovement(5);
        delay(1000);
    }

    end();

    // Go in circles
    robot->cmdMovement(1, 1);
    delay(1000);
    robot->cmdMovement(2, 1);
    delay(1000);
    robot->cmdMovement(1, 1);
    delay(1000);
    robot->cmdMovement(2, 1);
    delay(1000);
    robot->cmdMovement(1, 1);
    delay(1000);
    robot->cmdMovement(2, 1);
    delay(1000);
    robot->cmdMovement(1, 1);

    robot->cmdMovement(2, 1);
    delay(1000);
    robot->cmdMovement(2, 1);
    delay(1000);
    robot->cmdMovement(1, 1);
    delay(1000);
    robot->cmdMovement(3, 1);
    delay(1000);
    robot->cmdMovement(1, 1);
    delay(1000);
    robot->cmdMovement(3, 1);
    delay(1000);
    robot->cmdMovement(1, 1);
    delay(1000);
    robot->cmdMovement(3, 1);

    end();
}

void GeneralChecks::end()
{
    while (true)
        delay(1000);
}
/*

Test number of revolutions given by the encoders.
void GeneralChecks::test()

{
    Plot graph(robot);
    graph.startSequence();

    robot->motor[1].motorBackward();
    while (true)
    {
        robot->motor[1].setPWM(0);
        // robot->updateStraightPID(40);
        // robot
        delay(50);
        Serial.print("Front left: ");
        Serial.print(robot->motor[0].getRevolutions());
        Serial.print(", Back left: ");
        Serial.print(robot->motor[1].getRevolutions());
        Serial.print(", Front right: ");
        Serial.print(robot->motor[2].getRevolutions());

        Serial.print(", Back right: ");
        Serial.println(robot->motor[3].getRevolutions());

        // graph.plotTargetandCurrent();
        // graph.plotPWM();
    }
}
*/