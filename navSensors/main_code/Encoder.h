#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include "Motor.h"

// Motor Ids
#define FRONT_LEFT 0
#define BACK_LEFT 1
#define FRONT_RIGHT 2
#define BACK_RIGHT 3

// Movement comands
#define ROBOT_FORWARD 1
#define ROBOT_TURN_RIGHT 2
#define ROBOT_TURN_LEFT 3
#define ROBOT_RAMP 4

// Class used to modify motor's tics and regulate speed.
// Attach callbacks to interrupts to estimate rev/s and control speed using PID.

namespace Encoder{
  // Verifes the motor state and adds or substract tics to the motor depending on it
  // @param *motor Pointer to motor whose tics will be updated
  void updateTics(Motor *motor);
  
  // Decides to use back left encoder after motor ID
  void backLeftEncoder();

  // Decides to use front left encoder after motor ID
  void frontLeftEncoder();

  // Decides to use back right encoder after motor ID
  void backRightEncoder();

  // Decides to use front right encoder after motor ID
  void frontRightEncoder();
};

#endif 
