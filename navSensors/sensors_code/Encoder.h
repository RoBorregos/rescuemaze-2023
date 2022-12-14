#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include "Motor.h"
#include "Movement.h"

// Class used to modify motor's tics and regulate speed.

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
