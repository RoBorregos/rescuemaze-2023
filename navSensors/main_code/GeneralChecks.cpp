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
        // Serial.println("Running general checks in Arduino.");
    }
    checkSensorData();
    checkWheelDirections();
    checkUnitaryMovements();
}

void GeneralChecks::calibrateSensors()
{
    while (true)
    {
        robot->sensors->printInfo(false, false, true, false);
    }

    // Only calibrate directly on arduino, because the code with new ranges
    // would still need to be uploaded.
    // if (publishRos)
    // {
    //     if (robot->nh == nullptr)
    //         return;
    //     robot->nh->loginfo("Calibrate sensors only available directly on Arduino.");
    //     robot->nh->loginfo("Change publishROS flag and connect to serial monitor or arduino IDE.");
    //     return;
    // }

    // MUX2C mux;

    // // Print available i2c directions
    // mux.findI2C();

    // // Re-initialize sensors because mux.findI2C cuts communication.
    // Sensors s;
    // s.initSensors();
    // while (1)
    // {
    //     // Only print TCS info for calibration.
    //     s.printInfo(false, false, true, false);
    //     delay(200);
    // }
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

        for (int j = 0; j < iterations; j++)
        {
            robot->sensors->printInfo(false, true, false, false);
            delay(100);
        }

        Serial.println("\n\nTCS information: \n");
        for (int i = 0; i < iterations; i++)
        {
            robot->sensors->printInfo(false, false, true, false);
            delay(100);
        }

        Serial.println("Limit switches:\n");

        bool right = 0, left = 0;
        for (int i = 0; i < iterations; i++)
        {
            robot->sensors->getLimitSwitches(right, left);
            Serial.print("Right Limit switch: ");
            Serial.print(right);
            Serial.print(", Left Limit switch: ");
            Serial.println(left);
            delay(200);
        }

        Serial.println("Finished checking sensors.");

        for (int i = 0; i < iterations; i++)
        {
            if (robot->sensors->readMotorInit())
            {
                Serial.println("Motors are on.");
            }
            else
            {
                Serial.println("Motors are off.");
            }
            delay(100);
        }
    }
    end();
}

void GeneralChecks::checkDT()
{
    if (CK::kusingROS)
        return;
    log("Checking DT");

    log("Cmd movement(1,1): ", false);
    long int initialTime = millis();
    robot->cmdMovement(1, 1);

    Serial.println(dtToS(millis() - initialTime));

    log("Cmd movement(3,1): ", false);
    initialTime = millis();
    robot->cmdMovement(3, 1);
    Serial.println(dtToS(millis() - initialTime));

    log("Cmd movement(2,1): ", false);
    initialTime = millis();
    robot->cmdMovement(2, 1);
    Serial.println(dtToS(millis() - initialTime));

    log("robot->sensors->getTCSInfo(): ", false);
    initialTime = millis();
    robot->sensors->getTCSInfo();
    Serial.println(dtToS(millis() - initialTime));

    log("sensors->getVLXInfo(1): ", false);
    initialTime = millis();
    robot->sensors->getVLXInfo(1);
    Serial.println(dtToS(millis() - initialTime));

    log("getAngleX(): ", false);
    initialTime = millis();
    robot->sensors->getAngleX();
    Serial.println(dtToS(millis() - initialTime));
}

double GeneralChecks::dtToS(long int dt)
{
    return dt / 1000.0;
}

void GeneralChecks::checkWheelDirections()
{
    log("Testing all motors");
    delay(1000);
    robot->motor[FRONT_RIGHT].setPWM(255);
    robot->motor[FRONT_LEFT].setPWM(255);
    robot->motor[BACK_RIGHT].setPWM(255);
    robot->motor[BACK_LEFT].setPWM(255);

    log("FRONT RIGHT - Forward");
    robot->motor[FRONT_RIGHT].motorForward();
    delay(2000);
    robot->motor[FRONT_RIGHT].motorStop();
    delay(1000);

    log("FRONT RIGHT - Backwards");
    robot->motor[FRONT_RIGHT].motorBackward();
    delay(2000);
    robot->motor[FRONT_RIGHT].motorStop();
    delay(1000);

    log("FRONT LEFT - Forward");
    robot->motor[FRONT_LEFT].motorForward();
    delay(2000);
    robot->motor[FRONT_LEFT].motorStop();
    delay(1000);

    log("FRONT LEFT - Backwards");
    robot->motor[FRONT_LEFT].motorBackward();
    delay(2000);
    robot->motor[FRONT_LEFT].motorStop();
    delay(1000);

    log("BACK LEFT - Forward");
    robot->motor[BACK_LEFT].motorForward();
    delay(2000);
    robot->motor[BACK_LEFT].motorStop();
    delay(1000);

    log("BACK LEFT - Backwards");
    robot->motor[BACK_LEFT].motorBackward();
    delay(2000);
    robot->motor[BACK_LEFT].motorStop();
    delay(1000);

    log("BACK RIGHT - Forward");
    robot->motor[BACK_RIGHT].motorForward();
    delay(2000);
    robot->motor[BACK_RIGHT].motorStop();
    delay(1000);

    log("BACK RIGHT - Backwards");
    robot->motor[BACK_RIGHT].motorBackward();
    delay(2000);
    robot->motor[BACK_RIGHT].motorStop();

    delay(1000);
}

// TODO: implement function using cmdMovement.
void GeneralChecks::checkUnitaryMovements()
{
}

void GeneralChecks::log(String s, bool newLine)
{
    robot->sensors->logActive(s);
}

void GeneralChecks::test()
{
    /* Meaning of #actions, options and return values of cmdMovement:

  Action  Description                     Options                           Returns
  0       Move forward 1 unit (30 cms)    1 (use deg for error) 0 use vlx   1 -> successful, 0 -> move aborted, other -> Ramp
  3       Left turn (-90 deg)             1 to reaccomodate with back wall  1 -> successful, 0 -> move aborted
  1       Right turn (90 deg)             1 to reaccomodate with back wall  1 -> successful, 0 -> move aborted
  2       Move backward 1 unit (30 cms)   1 (use deg for error) 0 use vlx   1 -> successful, 0 -> move aborted
  5       Rearrange in current tile       None / ignored                    1 -> successful, 0 -> move aborted
  4       Traverse ramp                   None / ignored                    Estimated length of ramp.
  7       Drop n Kits.                    # of kits. Use sign for direction 1 -> successful, 0 -> move aborted
  8       Update angle reference          TODO, would help to reduce error given by physical field.
  */
    // robot->nh->loginfo("Running Test");

    // double angular = 0.7;

    // for (int i = 0; i < 5; i++)
    // {
    //     Kinematics::output test = robot->kinematics.getRPM(0, 0, angular);
    //     Serial.println("RPM with angular" + String(angular) + ": ");
    //     Serial.print("Front left: ");
    //     Serial.println(test.motor1);
    //     Serial.print("Front right: ");
    //     Serial.println(test.motor2);
    //     Serial.print("Back left: ");
    //     Serial.println(test.motor3);
    //     Serial.print("Back right: ");
    //     Serial.println(test.motor4);
    //     angular += 0.1;
    // }    
    Serial.println("Running test");

    while (true)
    {
        robot->sensors->printInfo(false, false, false, true);
        //robot->sensors->printInfo(false, true, false, false);
        // if (robot->sensors->readMotorInit())
        // {
        //     Serial.println("Motors are on");
        // }
        // else
        // {
        //     Serial.println("Motors are off");
        // }
    }

    // while (true)
    // Serial.println(robot->sensors->getDistInfo(dist_front));

    /*while (true)
    {
        robot->handleSwitches();
        Serial.println("Handle for limit switch finished");
        delay(2000);
    }*/

    // robot->motor[FRONT_LEFT].motorRotateDerPID(10, 50);
    // robot->updateAngleReference();
    // robot->basePitch = robot->sensors->getAngleY();
    // double pwm = 10;

    Serial.println("Running test");

    while (true)
    {
        if (robot->sensors->readMotorInit())
        {
            Serial.println("Motors are on");
        }
        else
        {
            Serial.println("Motors are off v2");
        }
    }
    while (true)
    {
        if (robot->sensors->readMotorInit())
        {
            Serial.println("Motors are on");
        }
        else
        {
            Serial.println("Motors are off");
        }
    }

    Serial.println("Specific test");
    robot->cmdMovement(7, 1);
    end();
    robot->cmdMovement(0, 1);
    end();
    while (true)
    {
        /*
        if (robot->outOfPitch()){
            Serial.println("Out of pitch");
        } else {
            Serial.println("In pitch");
        }*/
        robot->sensors->printInfo(false, false, false, false);
    }
    while (true)
    {
        robot->advanceXMetersVLX(0.7, 0, false);
        end();
        robot->cmdMovement(0, 1);
        delay(1000);
        robot->cmdMovement(0, 1);
        delay(1000);
        robot->cmdMovement(1, 1);
        delay(1000);
        robot->cmdMovement(0, 1);
        delay(1000);
        robot->cmdMovement(1, 1);
        delay(1000);
        robot->cmdMovement(1, 1);
        end();
        delay(2000);
    }

    delay(1000);
    end();
    robot->translationX(-0.3);
    end();

    robot->cmdMovement(3, 1);
    // robot->updateAngleReference();
    delay(1000);
    robot->advanceXMeters(0.3, 1);
    end();
    robot->updateAngleReference();
    delay(1000);
    robot->advanceXMeters(0.3, 1);
    end();
    robot->updateAngleReference();

    while (true)
    {
        robot->handleSwitches();
    }
    Serial.println("Moveng advanceX abs");
    robot->updateAngleReference();
    while (true)
    {
        robot->advanceXMeters(0.3, 1);
        end();
    }

    end();

    while (true)
    {
        // robot->handleSwitches();
        robot->sensors->printInfo(false, true, false, false);
    }

    while (true)
    {
        robot->cmdMovement(0, 1);
        delay(1000);
        end();
    }

    robot->cmdMovement(0, 1);
    delay(1000);
    robot->cmdMovement(1, 1);
    end();
}

void GeneralChecks::end()
{
    while (true)
        delay(1000);
}

/* Test ema filter
 SingleEMAFilter<float> singleEMAFilter(0.6);

 while (true){
     double rawMeasure = robot->sensors->getVLXInfo(0);

     // Calcular filtro
     singleEMAFilter.AddValue(rawMeasure);

     // Mostrar resultados
     // Emplear Serial Plotter para visualización gráfica
     Serial.print(rawMeasure);
     Serial.print(",\t");
     Serial.print(singleEMAFilter.GetLowPass());
     Serial.print(",\t");
     Serial.println(singleEMAFilter.GetHighPass());
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

void GeneralChecks::checkPID()
{
    Plot plot(robot);
    plot.startSequence();
    long int time = millis();
    long int interval = 8000;
    double revolutions = 40;

    while (true)
    {
        plot.plotPWM();
        robot->updateStraightPID(revolutions);
        if (millis() - time > interval)
        {
            time = millis();
            revolutions += 10;
        }
    }
}

void GeneralChecks::checkWheelSpeed()
{
    Serial.println("Checking wheel speed");
    robot->motor[FRONT_RIGHT].motorForward();
    robot->motor[BACK_RIGHT].motorForward();
    robot->motor[FRONT_LEFT].motorForward();
    robot->motor[BACK_LEFT].motorForward();

    // robot->motor[FRONT_RIGHT].setPWM(CK::basePwmFrontRight);
    // robot->motor[FRONT_LEFT].setPWM(CK::basePwmFrontLeft);
    // robot->motor[BACK_RIGHT].setPWM(CK::basePwmBackRight);
    // robot->motor[BACK_LEFT].setPWM(CK::basePwmBackLeft);

    robot->motor[FRONT_RIGHT].setPWM(255);
    robot->motor[FRONT_LEFT].setPWM(255);
    robot->motor[BACK_RIGHT].setPWM(255);
    robot->motor[BACK_LEFT].setPWM(255);
    while (true)
    {
        delay(500);
    }
}

void GeneralChecks::checkOled()
{
    /**
    Serial.println("Next test");
    robot->sensors->screen.resetScreen();
    robot->sensors->screen.testdraw();

    end();
    */
    robot->sensors->screen.resetScreen();
    robot->sensors->logActive("Hola 1", true, 0, 1);
    robot->sensors->logActive("Hola 2", true, 0, 2);
    robot->sensors->logActive("Hola 3", true, 0, 3);
    robot->sensors->logActive("Hola 4", true, 0, 4);
    robot->sensors->logActive("Hola 5", true, 0, 5);
    robot->sensors->logActive("Hola 6", true, 0, 6);
    robot->sensors->logActive("Hola 7", true, 0, 7);
    robot->sensors->logActive("Hola 8", true, 0, 8);
    robot->sensors->logActive("Remplazo hola 2", true, 0, 2);
    // robot->sensors->screen.resetLine(2);
    end();

    end();
    robot->sensors->screen.resetScreen();
    robot->sensors->logActive("Hola abajo", true, 0, 1);
    robot->sensors->logActive("Hola", true, 0, 0);
    robot->sensors->logActive("remplazo", true, 0, 0);

    long int dt = millis();
    robot->sensors->logActive("remplaso2", true, 0, 1);
    Serial.println("Dt: " + String((millis() - dt) / 1000.0));
    Serial.println("Remplazado");
    robot->sensors->screen.resetLine(2);
    end();
}

void GeneralChecks::printRevolutions()
{
    Plot graph(robot);
    graph.startSequence();
    // robot->motor[FRONT_RIGHT].motorBackward();
    // robot->motor[BACK_RIGHT].motorBackward();
    // robot->motor[FRONT_LEFT].motorForward();
    // robot->motor[BACK_LEFT].motorForward();

    // robot->motor[FRONT_RIGHT].setPWM(CK::basePwmFrontRight);
    // robot->motor[FRONT_LEFT].setPWM(CK::basePwmFrontLeft);
    // robot->motor[BACK_RIGHT].setPWM(CK::basePwmBackRight);
    // robot->motor[BACK_LEFT].setPWM(CK::basePwmBackLeft);

    // robot->motor[FRONT_RIGHT].setPWM(255);
    // robot->motor[FRONT_LEFT].setPWM(255);
    // robot->motor[BACK_RIGHT].setPWM(255);
    // robot->motor[BACK_LEFT].setPWM(255);

    while (true)
    {
        // robot->motor[FRONT_RIGHT].motorSpeedPID(40, false, 1);
        // robot->motor[BACK_RIGHT].motorSpeedPID(40, false, 1);
        // robot->motor[FRONT_LEFT].motorSpeedPID(40, false, 1);
        // robot->motor[BACK_LEFT].motorSpeedPID(40, false, 1);
        robot->motor[FRONT_RIGHT].motorRotationPID(40, true);
        robot->motor[BACK_RIGHT].motorRotationPID(40);
        robot->motor[FRONT_LEFT].motorRotationPID(-40);
        robot->motor[BACK_LEFT].motorRotationPID(-40);

        graph.plotPWM();
        // graph.plotTargetandCurrent();
        // Serial.print("Front left: ");
        // Serial.print(robot->motor[FRONT_LEFT].getRevolutions());
        // Serial.print(" speed: ");
        // Serial.print(robot->motor[FRONT_LEFT].currentSpeed);
        // Serial.print(", Back left: ");
        // Serial.print(robot->motor[BACK_LEFT].getRevolutions());
        // Serial.print(" speed: ");
        // Serial.print(robot->motor[FRONT_LEFT].currentSpeed);
        // Serial.print(", Front right: ");
        // Serial.print(robot->motor[FRONT_RIGHT].getRevolutions());
        // Serial.print(" speed: ");
        // Serial.print(robot->motor[FRONT_LEFT].currentSpeed);
        // Serial.print(", Back right: ");
        // Serial.println(robot->motor[BACK_RIGHT].getRevolutions());
        // Serial.print(" speed: ");
        // Serial.print(robot->motor[FRONT_LEFT].currentSpeed);
    }
}