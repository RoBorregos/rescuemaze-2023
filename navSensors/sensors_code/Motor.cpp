#include "Motor.h"
#include "Encoder.h"

Motor::Motor(){
  this->digitalOne = 0;
  this->digitalTwo = 0;
  this->pwmPin = 0;
  this->encoderA = 0;
  this->encoderB = 0;
}

Motor::Motor(uint8_t digitalOne, uint8_t digitalTwo, int pwmPin, uint8_t encoderA, uint8_t encoderB, MotorID motorID) :
pidStraight(kPStraight, kIStraight, kDStraight, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample),
pidRotate(kPRotate, kIRotate, kDRotate, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample) {
  this->digitalOne = digitalOne;
  this->digitalTwo = digitalTwo;
  this->pwmPin = pwmPin;
  this->motorID = motorID;
  this->encoderA = encoderA;
  this->encoderB = encoderB;
}

// Atribute Getters

uint8_t Motor::getEncoderA(){
  return encoderA;
}
  
uint8_t Motor::getEncoderB(){
  return encoderB;
}

MotorState Motor::getCurrentState(){
  return currentState;
}
  
int Motor::getEncoderTics(){
  return ticsCounter;
}
  
double Motor::getCurrentSpeed(){
  return currentSpeed;
}
  
double Motor::getTargetSpeed(){
  return RPM2RPS(targetSpeed);
}

int Motor::getPidTics(){
  return pidTics;
}
  

// Initialization
void Motor::motorSetup(){
  pinMode(digitalOne, OUTPUT);
  pinMode(digitalTwo, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
}

void Motor::initEncoders(){ 
  switch(motorID){
    case MotorID::backLeft:
      attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::backLeftEncoder, RISING);
    break;
    case MotorID::frontLeft:
      attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::frontLeftEncoder, RISING);
    break;
    case MotorID::backRight:
      attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::backRightEncoder, RISING);
    break;
    case MotorID::frontRight:
      attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::frontRightEncoder, RISING);
    break;
  }
}

// Encoder and PID tics
void Motor::deltaEncoderTics(int delta){
  ticsCounter += delta;
}

void Motor::deltaPidTics(int delta){
  pidTics += delta; 
}

// Control Functions
void Motor::motorForward(){
  analogWrite(pwmPin, pwm);
  if (currentState == MotorState::Forward){
    return;
  }

  digitalWrite(digitalOne, HIGH);
  digitalWrite(digitalTwo, LOW);
  
  pidStraight.reset();
  pidRotate.reset();

  currentState = MotorState::Forward;
}

void Motor::motorBackward(){
  analogWrite(pwmPin, pwm);

  if (currentState == MotorState::Backward){
    return;
  }
  
  digitalWrite(digitalOne, LOW);
  digitalWrite(digitalTwo, HIGH);

  pidStraight.reset();
  pidRotate.reset();

  currentState = MotorState::Backward;
}
    
void Motor::motorStop(){
  analogWrite(pwmPin, LOW);

  if (currentState == MotorState::Stop){
    return;
  }
  
  digitalWrite(digitalOne, LOW);
  digitalWrite(digitalTwo, LOW);

  pidStraight.reset();
  pidRotate.reset();

  currentState = MotorState::Stop;
}

// PWM Velocity. 
void Motor::setPWM(double PWM){
  this->pwm = PWM;
  switch(currentState) {
    case MotorState::Forward:
      motorForward();
    break;
    case MotorState::Backward:
      motorBackward();
    break;
    case MotorState::Stop:
      motorStop();
    break;
  }
}

double Motor::getTargetRps(double velocity){
  return Ms2Rps(velocity);
}

double Motor::RPM2RPS(double velocity){
  return velocity/kSecondsInMinute;
}

double Motor::Ms2Rps(double MS){
  return (MS / (kDistancePerRev));
}

double Motor::Pwm2Rpm(double pwm){
  return ((pwm * kRPM) / kMaxPWM);
}

void Motor::motorSpeedPID(double target_speed){
  int speed_sign = min(1, max(-1, target_speed * 1000));
  this->targetSpeed = fabs(target_speed);
  double tmp_pwm = pwm;
  switch (speed_sign)
  {
  case 0:
    motorStop();
  break;
  case 1:
    motorForward();
  break;
  case -1:
    motorBackward();
  break;
  }
  
  pidStraight.computeSpeed(RPM2RPS(targetSpeed), currentSpeed, tmp_pwm, pidTics, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);
  
  setPWM(tmp_pwm);
}

void Motor::motorRotateIzqPID(double target_angle, double current_angle){
  double tmp_pwm = pwm;
  pidRotate.computeRotateIzq(target_angle, current_angle, tmp_pwm);
  tmp_pwm = fabs(tmp_pwm);
  if (tmp_pwm < 70){
    tmp_pwm = 70;
  } else if (tmp_pwm > 255){
    tmp_pwm = 255;
  }
  setPWM(tmp_pwm);
}

void Motor::motorRotateDerPID(double target_angle, double current_angle){
  double tmp_pwm = pwm;
  pidRotate.computeRotateDer(target_angle, current_angle, tmp_pwm);
  tmp_pwm = fabs(tmp_pwm);
  if (tmp_pwm < 70){
    tmp_pwm = 70;
  } else if (tmp_pwm > 255){
    tmp_pwm = 255;
  }
  setPWM(tmp_pwm);
}

double Motor::getPWM(){
  return pwm;
}

void Motor::motorSpeedPWM(double target_speed){
  int speed_sign = min(1, max(-1, target_speed * 1000));
  this->targetSpeed = fabs(target_speed);
  pwm = targetSpeed;

  switch (speed_sign)
  {
  case 0:
    motorStop();
  break;
  case 1:
    motorForward();
  break;
  case -1:
    motorBackward();
  break;
  }
}

double Motor::getDistanceTraveled(){
  return (getEncoderTics()/kPulsesPerRevolution) * kDistancePerRev; 
}

void Motor::setEncoderTics(int tics){
  ticsCounter = tics;
}

void Motor::PIDStraightTunnigs(double kp, double ki, double kd){
  pidStraight.setTunnings(kp,ki,kd);
}
  
void Motor::PIDRotateTunnigs(double kp, double ki, double kd){
  pidRotate.setTunnings(kp,ki,kd);
}
