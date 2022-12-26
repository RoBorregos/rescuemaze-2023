#include "Encoder.h"

// Verifes the motor state and adds or substract tics to the motor depending on it
void Encoder::updateTics(Motor *motor){
  motor->deltaPidTics(1);
  if(motor->getCurrentState() == MotorState::Forward){
    motor->deltaEncoderTics(1);
  }
  else if (motor->getCurrentState() == MotorState::Backward){
    motor->deltaEncoderTics(-1);
  }
  else {
    return;
  }
}

// Methods to select a specific motor to change tics.

void Encoder::backLeftEncoder() {
  updateTics(&robot->motor[BACK_LEFT]);
}
  
void Encoder::frontLeftEncoder() {
  updateTics(&robot->motor[FRONT_LEFT]);
}
  
void Encoder::backRightEncoder() {
  updateTics(&robot->motor[BACK_RIGHT]);
}
  
void Encoder::frontRightEncoder() {
  updateTics(&robot->motor[FRONT_RIGHT]);
}
