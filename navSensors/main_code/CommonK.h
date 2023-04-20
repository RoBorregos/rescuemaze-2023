#ifndef COMMONK_H
#define COMMONK_H

// File used to define constants that are used in multiple files.

namespace CK
{
    // Determines if Serial.print should be used along the code. Serial.print may disrupt serial communication with ROS.
    const bool kusingROS = false;
    const bool debugAdvanceX = true;
    const bool debugRamp = true;
    const bool debugGoToAngle = true;
    const bool debugRotation = true;
    const bool vlxPID = true;
    const bool onlyCmdMovement = true;
    const bool debugBNOCalibration = true;
    const bool calibrateBNO = true;
    const bool usePid = true;
    const bool useBNO = true;

    // Tune constants such that motors all motors move at desired speed.
    const int basePwmBackLeft = 100;
    const int basePwmBackRight = 100;
    const int basePwmFrontLeft = 100;
    const int basePwmFrontRight = 100;

    const double kRampDt = 3;
}

#endif