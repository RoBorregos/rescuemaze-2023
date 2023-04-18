#ifndef COMMONK_H
#define COMMONK_H

// File used to define constants that are used in multiple files.

namespace CK
{
    // Determines if Serial.print should be used along the code. Serial.print may disrupt serial communication with ROS.
    const bool kusingROS = true;
    const bool debugAdvanceX = true;
    const bool debugRamp = true;
    const bool debugGoToAngle = true;
    const bool debugRotation = true;
    const bool vlxPID = true;
    const bool onlyCmdMovement = true;
    const bool debugBNOCalibration = true;
    const bool calibrateBNO = false;
}

#endif