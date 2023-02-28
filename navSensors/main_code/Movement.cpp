#include "Movement.h"

// Constructors

Movement::Movement(ros::NodeHandle *nh, BNO *bno, Sensors *sensors): nh_(nh), bno(bno),  sensors(sensors) {
  kinematics_ = Kinematics(motor_max_rpm_, wheel_diameter_, fr_wheels_dist_, lr_wheels_dist_, pwm_bits_);
  setMotors();
  dispenser = Dispenser(kServoPin);
  initRobot();
}

Movement::Movement(BNO *bno, Sensors *sensors) : bno(bno), sensors(sensors)
{
  kinematics_ = Kinematics(motor_max_rpm_, wheel_diameter_, fr_wheels_dist_, lr_wheels_dist_, pwm_bits_);
  setMotors();
  this->dispenser = Dispenser(kServoPin);
  initRobot();
}

Movement::Movement(BNO *bno, Sensors *sensors, bool individualConstants) : bno(bno), sensors(sensors)
{
  kinematics_ = Kinematics(motor_max_rpm_, wheel_diameter_, fr_wheels_dist_, lr_wheels_dist_, pwm_bits_);
  setMotors();
  this->dispenser = Dispenser(kServoPin);
  setIndividualPID();
  initRobot();
}

void Movement::setIndividualPID()
{
  motor[BACK_LEFT].PIDStraightTunnigs(12, 6, 2);
  motor[FRONT_RIGHT].PIDStraightTunnigs(18, 6, 2);
  motor[FRONT_LEFT].PIDStraightTunnigs(18, 6, 2);
  motor[BACK_RIGHT].PIDStraightTunnigs(16, 6, 2);
}

void Movement::setMotors()
{
  motor[FRONT_LEFT] = Motor(kDigitalPinsFrontLeftMotor[1], kDigitalPinsFrontLeftMotor[0],
                            kAnalogPinFrontLeftMotor, kEncoderPinsFrontLeftMotor[0],
                            kEncoderPinsFrontLeftMotor[1], MotorID::frontLeft);
  motor[BACK_LEFT] = Motor(kDigitalPinsBackLeftMotor[1], kDigitalPinsBackLeftMotor[0],
                           kAnalogPinBackLeftMotor, kEncoderPinsBackLeftMotor[0],
                           kEncoderPinsBackLeftMotor[1], MotorID::backLeft);
  motor[FRONT_RIGHT] = Motor(kDigitalPinsFrontRightMotor[1], kDigitalPinsFrontRightMotor[0],
                             kAnalogPinFrontRightMotor, kEncoderPinsFrontRightMotor[0],
                             kEncoderPinsBackRightMotor[1], MotorID::frontRight);
  motor[BACK_RIGHT] = Motor(kDigitalPinsBackRightMotor[1], kDigitalPinsBackRightMotor[0],
                            kAnalogPinBackRightMotor, kEncoderPinsBackRightMotor[0],
                            kEncoderPinsBackRightMotor[1], MotorID::backRight);
}

// Constrains

// Constrains the given angle to the given range for each cardinal point.
void constrainAngle(double &angle, double angle_min, double angle_max)
{
  if (angle > angle_max)
  {
    angle -= angle_max;
  }
  else if (angle < angle_min)
  {
    angle += angle_max;
  }

  if (angle < 45 || angle > 315)
  {
    angle = 0;
  }
  else if (angle < 315 && angle > 225)
  {
    angle = 270;
  }
  else if (angle < 225 && angle > 135)
  {
    angle = 180;
  }
  else if (angle < 135 && angle > 45)
  {
    angle = 90;
  }
}

// returns the constrained velocity of the robot.
double constrainDa(double x, double min_, double max_)
{
  return max(min_, min(x, max_));
}

// Verifes if the current_angle is within a range of the desired angle
bool fueraRango(int current_angle, int desired_angle)
{
  if (desired_angle == 0)
  {
    double nA = current_angle >= 180 ? current_angle : (360 + current_angle);

    if (nA > (360 - 5) && nA < (360 + 5))
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {

    if (current_angle > (desired_angle - 5) && current_angle < (desired_angle + 5))
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

// Initialization

void Movement::initRobot()
{
  for (int i = 0; i < kMotorCount; i++)
  {
    motor[i].initMotor();
    motor[i].setPWM(150);
    last_encoder_counts_[i] = 0;
  }

  dispenser.initServo();

  initLeds();
}

void Movement::initLeds()
{
  pinMode(kDigitalPinsLEDS[0], OUTPUT);
  pinMode(kDigitalPinsLEDS[1], OUTPUT);
}

void Movement::initSwitches()
{
  pinMode(kDigitalPinsLimitSwitch[0], INPUT);
  pinMode(kDigitalPinsLimitSwitch[1], INPUT);
}

// Encoder Functions

double Movement::meanDistanceTraveled()
{
  double sum_encoder_distance = 0;
  for (int i = 0; i < kMotorCount; i++)
  {
    sum_encoder_distance += this->motor[i].getDistanceTraveled();
  }
  return sum_encoder_distance / kMotorCount;
}

void Movement::getEncoderCounts(int *delta_encoder_counts)
{
  for (int i = 0; i < kMotorCount; i++)
  {
    delta_encoder_counts[i] = motor[i].getEncoderTics();
  }
  resetEncoders();
}

void Movement::resetEncoders()
{
  for (int i = 0; i < kMotorCount; i++)
  {
    motor[i].setEncoderTics(0);
  }
}

void Movement::encoderTics()
{
  for (int i = 0; i < kMotorCount; i++)
  {
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(motor[i].getEncoderTics());
    delay(10);
  }
}

// Motor Functions
void Movement::stop()
{
  for (int i = 0; i < kMotorCount; i++)
  {
    motor[i].motorStop();
  }
}

void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z){
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);

  char log_msg[20];
  char result[8];
  double rpms[kMotorCount];
  double pwms[kMotorCount];


  if (kUsingPID){
    //Serial.println("Using PID");
    Kinematics::output rpm = kinematics_.getRPM(x, y ,z);

    motor[FRONT_LEFT].motorSpeedPID(rpm.motor1);
    motor[FRONT_RIGHT].motorSpeedPID(rpm.motor2);
    motor[BACK_LEFT].motorSpeedPID(rpm.motor3);
    motor[BACK_RIGHT].motorSpeedPID(rpm.motor4);

    rpms[FRONT_LEFT] = rpm.motor1;
    rpms[FRONT_RIGHT] = rpm.motor2;
    rpms[BACK_LEFT] = rpm.motor3;
    rpms[BACK_RIGHT] = rpm.motor4;

    dtostrf(rpms[FRONT_LEFT], 6, 2, result);
    sprintf(log_msg,"M1 RPM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
    dtostrf(rpms[BACK_LEFT], 6, 2, result);
    sprintf(log_msg,"M2 RPM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
    dtostrf(rpms[FRONT_RIGHT] , 6, 2, result);
    sprintf(log_msg,"M3 RPM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
    dtostrf(rpms[BACK_RIGHT], 6, 2, result);
    sprintf(log_msg,"M4 RPM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
  } else {
    //Serial.println("NOT using PID");
    Kinematics::output pwm = kinematics_.getPWM(x, y ,z);

    motor[FRONT_LEFT].motorSpeedPWM(pwm.motor1);
    motor[FRONT_RIGHT].motorSpeedPWM(pwm.motor2);
    motor[BACK_LEFT].motorSpeedPWM(pwm.motor3);
    motor[BACK_RIGHT].motorSpeedPWM(pwm.motor4);

    pwms[FRONT_LEFT] = pwm.motor1;
    pwms[FRONT_RIGHT] = pwm.motor2;
    pwms[BACK_LEFT] = pwm.motor3;
    pwms[BACK_RIGHT] = pwm.motor4;


    dtostrf(pwms[FRONT_LEFT], 6, 2, result);
    sprintf(log_msg,"M1 PWM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
    dtostrf(pwms[BACK_LEFT], 6, 2, result);
    sprintf(log_msg,"M2 PWM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
    dtostrf(pwms[FRONT_RIGHT] , 6, 2, result);
    sprintf(log_msg,"M3 PWM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
    dtostrf(pwms[BACK_RIGHT], 6, 2, result);
    sprintf(log_msg,"M4 PWM :%s", result);
    nh_->loginfo(log_msg);
    //Serial.println(log_msg);
  }
}

void Movement::girarIzquierda()
{
  motor[FRONT_LEFT].motorForward();
  motor[BACK_LEFT].motorForward();
  motor[FRONT_RIGHT].motorBackward();
  motor[BACK_RIGHT].motorBackward();
}

void Movement::girarDerecha()
{
  motor[FRONT_LEFT].motorBackward();
  motor[BACK_LEFT].motorBackward();
  motor[FRONT_RIGHT].motorForward();
  motor[BACK_RIGHT].motorForward();
}

void Movement::updateStraightPID(int RPMs)
{
  motor[FRONT_LEFT].motorSpeedPID(RPMs);
  motor[BACK_LEFT].motorSpeedPID(RPMs);
  motor[FRONT_RIGHT].motorSpeedPID(RPMs);
  motor[BACK_RIGHT].motorSpeedPID(RPMs);
}

void Movement::advanceXMeters(double x)
{
  double dist = meanDistanceTraveled();
  double target = dist + x;

  int counter = 0;

  while (dist < target)
  {
    counter++;
    updateStraightPID(kMovementRPMs);
    if (counter == 20)
    {
      dist = meanDistanceTraveled();
      counter = 0;
    }
  }
  stop();
  resetEncoders();
}

void Movement::turnDecider(double current_angle, double desired_angle)
{
  double angle_difference = 0;
  if (desired_angle > current_angle)
  {
    angle_difference = desired_angle - current_angle;

    if (angle_difference < 180)
      girarDerecha();
    else
      girarIzquierda();
  }
  else
  {
    angle_difference = current_angle - desired_angle;

    if (angle_difference < 180)
      girarIzquierda();
    else
      girarDerecha();
  }
}

void Movement::girarDeltaAngulo(double delta_theta)
{
  double current_angle = 0;
  double desired_angle = 0;

  if (delta_theta > 0)
  {
    current_angle = bno->getAngleX();
    desired_angle = current_angle + delta_theta;
    constrainAngle(desired_angle, kMinAngle, kMaxAngle);
    while (fueraRango(current_angle, desired_angle))
    {
      turnDecider(current_angle, desired_angle);

      motor[FRONT_LEFT].motorRotateDerPID(desired_angle, current_angle);
      motor[BACK_LEFT].motorRotateDerPID(desired_angle, current_angle);
      motor[FRONT_RIGHT].motorRotateDerPID(desired_angle, current_angle);
      motor[BACK_RIGHT].motorRotateDerPID(desired_angle, current_angle);
      current_angle = bno->getAngleX();
    }
    stop();
    resetEncoders();
  }
  else if (delta_theta < 0)
  {
    desired_angle = current_angle + delta_theta;
    constrainAngle(desired_angle, kMinAngle, kMaxAngle);
    while (fueraRango(current_angle, desired_angle))
    {
      turnDecider(current_angle, desired_angle);

      motor[FRONT_LEFT].motorRotateIzqPID(desired_angle, current_angle);
      motor[BACK_LEFT].motorRotateIzqPID(desired_angle, current_angle);
      motor[FRONT_RIGHT].motorRotateIzqPID(desired_angle, current_angle);
      motor[BACK_RIGHT].motorRotateIzqPID(desired_angle, current_angle);
      current_angle = bno->getAngleX();
    }
    stop();
    resetEncoders();
  }
}

void Movement::cmdMovement(int movement_type)
{
  double current_angle = 0;
  double desired_angle = 0;
  double current_distance = 0;
  double goal_distance = 0;

  digitalWrite(kDigitalPinsLEDS[0], LOW);
  switch (movement_type)
  {
  case ROBOT_FORWARD:
  {
    current_distance = sensors->getVLXInfo(vlx_front);
    if (current_distance < 0.35)
    {
      while (current_distance > 0.08)
      {
        updateStraightPID(kMovementRPMs);
        current_distance = sensors->getVLXInfo(vlx_front);

        /*if (sensors_->getTCSInfo() == 'b'){
          stop();
          resetEncoders();
          return;
        }*/
      }
    }
    else
    {
      goal_distance = current_distance - 0.3;
      while (current_distance > goal_distance)
      {
        updateStraightPID(kMovementRPMs);
        current_distance = sensors->getVLXInfo(vlx_front);
        /*if (sensors_->getTCSInfo() == 'b'){
          stop();
          resetEncoders();
          return;
        }*/
      }
    }
    stop();
    resetEncoders();
    current_angle = bno->getAngleX();
    desired_angle = current_angle;
    constrainAngle(desired_angle, kMinAngle, kMaxAngle);
    double angle_diff = current_angle - desired_angle;
    girarDeltaAngulo(angle_diff);
    break;
  }
  case ROBOT_TURN_RIGHT:
    current_angle = bno->getAngleX();
    desired_angle = current_angle + 90;
    constrainAngle(desired_angle, kMinAngle, kMaxAngle);
    while (fueraRango(current_angle, desired_angle))
    {
      turnDecider(current_angle, desired_angle);

      motor[FRONT_LEFT].motorRotateDerPID(desired_angle, current_angle);
      motor[BACK_LEFT].motorRotateDerPID(desired_angle, current_angle);
      motor[FRONT_RIGHT].motorRotateDerPID(desired_angle, current_angle);
      motor[BACK_RIGHT].motorRotateDerPID(desired_angle, current_angle);
      current_angle = bno->getAngleX();
    }
    stop();
    resetEncoders();
    break;
  case ROBOT_TURN_LEFT:
    current_angle = bno->getAngleX();
    desired_angle = current_angle - 90;
    constrainAngle(desired_angle, kMinAngle, kMaxAngle);
    while (fueraRango(current_angle, desired_angle))
    {
      turnDecider(current_angle, desired_angle);

      motor[FRONT_LEFT].motorRotateIzqPID(desired_angle, current_angle);
      motor[BACK_LEFT].motorRotateIzqPID(desired_angle, current_angle);
      motor[FRONT_RIGHT].motorRotateIzqPID(desired_angle, current_angle);
      motor[BACK_RIGHT].motorRotateIzqPID(desired_angle, current_angle);
      current_angle = bno->getAngleX();
    }
    stop();
    resetEncoders();
    break;
  case ROBOT_RAMP:
    current_angle = bno->getAngleZ();
    while (current_angle < -10 || current_angle > 10)
    {
      updateStraightPID(kMovementRPMs - 20);
      current_angle = bno->getAngleZ();
    }
    stop();
    resetEncoders();
    break;
  }
  digitalWrite(kDigitalPinsLEDS[0], HIGH);
  checkLimitSwitches();
}

// Drop kit decider

// Gets sign which refers to where should a kit be dropped
void Movement::dropDecider(int ros_sign_callback)
{
  digitalWrite(kDigitalPinsLEDS[1], LOW);
  switch (ros_sign_callback)
  {
  case right:
    dispenser.rightDrop();
    break;
  case left:
    dispenser.leftDrop();
    break;
  }
  digitalWrite(kDigitalPinsLEDS[1], HIGH);
}

// Limit switches checker

void Movement::checkLimitSwitches()
{
  bool limitSwitchRight = true;
  bool limitSwitchLeft = true;

  while (limitSwitchLeft == true || limitSwitchRight == true)
  {
    limitSwitchRight = digitalRead(kDigitalPinsLimitSwitch[0]);
    limitSwitchLeft = digitalRead(kDigitalPinsLimitSwitch[1]);
    if (limitSwitchLeft == true && limitSwitchRight == true)
    {
      while (meanDistanceTraveled() < 0.1)
      {
        updateStraightPID(-kMovementRPMs);
      }
      stop();
      resetEncoders();
      if (limitSwitchLeft == true)
      {
        while (meanDistanceTraveled() * 2 < 0.1)
        {
          motor[FRONT_LEFT].motorSpeedPID(kMovementRPMs);
          motor[BACK_LEFT].motorSpeedPID(kMovementRPMs);
        }
        stop();
        resetEncoders();
      }
      else if (limitSwitchRight == true)
      {
        while (meanDistanceTraveled() * 2 < 0.1)
        {
          motor[FRONT_RIGHT].motorSpeedPID(kMovementRPMs);
          motor[BACK_RIGHT].motorSpeedPID(kMovementRPMs);
        }
        stop();
        resetEncoders();
      }
    }
  }
}

void Movement::testMotor()
{

  Motor *m = &motor[BACK_LEFT];
  //Motor *m = &motor[BACK_LEFT];
  //Motor *m = &motor[BACK_RIGHT];
  //Motor *m = &motor[FRONT_LEFT];
  
  m->motorBackward();
  while (true)
  { 
    Serial.println(m->getPWM());
    m->motorSpeedPID(90);
    Serial.print("Tics: ");
    Serial.println(m->getEncoderTics());
  }
}
