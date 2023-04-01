#include "Movement.h"

// Constructors

Movement::Movement(ros::NodeHandle *nh, BNO *bno, Sensors *sensors, bool individualConstants) : nh(nh), bno(bno), sensors(sensors)
{
  initMovement(individualConstants);
}

Movement::Movement(BNO *bno, Sensors *sensors, bool individualConstants) : bno(bno), sensors(sensors)
{
  nh = nullptr;
  initMovement(individualConstants);
}

Movement::Movement(ros::NodeHandle *nh, Sensors *sensors, bool individualConstants) : nh(nh), sensors(sensors)
{
  bno = nullptr;
  initMovement(individualConstants);
}

Movement::Movement(Sensors *sensors, bool individualConstants) : sensors(sensors)
{
  nh = nullptr;
  bno = nullptr;
  initMovement(individualConstants);
}

Movement::initMovement(bool individualConstants)
{
  kinematics = Kinematics(kRPM, kWheelDiameter, kFrWheelsDist, kLrWheelsDist, kPwmBits);
  setMotors();
  this->dispenser = Dispenser(kServoPin);
  if (individualConstants)
    setIndividualPID();
  initRobot();
}

void Movement::setIndividualPID()
{
  motor[BACK_LEFT].PIDStraightTunings(470, 0, 15);
  motor[FRONT_RIGHT].PIDStraightTunings(120, 80, 10);
  motor[FRONT_LEFT].PIDStraightTunings(120, 80, 10);
  motor[BACK_RIGHT].PIDStraightTunings(120, 80, 10);

  motor[BACK_LEFT].PIDConservativeTunings(120, 80, 10);
  motor[FRONT_RIGHT].PIDConservativeTunings(120, 80, 10);
  motor[FRONT_LEFT].PIDConservativeTunings(120, 80, 10);
  motor[BACK_RIGHT].PIDConservativeTunings(120, 80, 10);

  motor[BACK_LEFT].PIDAggressiveTunings(470, 0, 15);
  motor[FRONT_RIGHT].PIDAggressiveTunings(470, 0, 15);
  motor[FRONT_LEFT].PIDAggressiveTunings(470, 0, 15);
  motor[BACK_RIGHT].PIDAggressiveTunings(470, 0, 15);

  // Oscilates but arrives fast: motor[FRONT_RIGHT].PIDStraightTunings(470, 0, 15);
  // Oscilates less but arrives less fast: motor[FRONT_RIGHT].PIDStraightTunings(200, 120, 10);
  // Control constants: motor[FRONT_RIGHT].PIDStraightTunings(120, 80, 10);
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
    motor[i].setPWM(0);
  }

  dispenser.initServo();

  initLeds();
  initSwitches();
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

int Movement::leftLimitSwitch()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[0]);
  if (val == HIGH)
  {
    return 1;
  }

  return 0;
}

int Movement::rightLimitSwitch()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[1]);
  if (val == HIGH)
  {
    return 1;
  }

  return 0;
}

void Movement::debugLimitSwitches()
{
  int val = digitalRead(kDigitalPinsLimitSwitch[0]);
  if (val == HIGH)
  {
    Serial.println("Switch 0 is open");
  }
  else
  {
    Serial.println("Switch 0 is closed");
  }
  int val2 = digitalRead(kDigitalPinsLimitSwitch[1]);
  if (val2 == HIGH)
  {
    Serial.println("Switch 1 is open");
  }
  else
  {
    Serial.println("Switch 1 is closed");
  }
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

void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z, const bool debug)
{
  if (nh == nullptr)
  {
    Serial.println("ERROR, trying to use NodeHandle without initializing it. Change constructor or method call.");
    return;
  }

  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);

  if (debug)
  {
    if (linear_x != x)
    {
      nh->loginfo("Linear velocity in X constrained!");
    }
    if (linear_y != y)
    {
      nh->loginfo("Linear velocity in Y constrained!");
    }
    if (angular_z != z)
    {
      nh->loginfo("Angular velocity in Z constrained!");
    }
  }

  if (kUsingPID)
  {
    Kinematics::output rpm = kinematics.getRPM(x, y, z);
    Kinematics::output rpm1 = kinematics.getRPM(linear_x, linear_y, angular_z);

    updatePIDKinematics(rpm);

    if (debug)
    {
      char log_msg[20];
      char result[8];
      double rpms[kMotorCount];
      rpms[FRONT_LEFT] = (float)rpm.motor1;
      rpms[FRONT_RIGHT] = (float)rpm.motor2;
      rpms[BACK_LEFT] = (float)rpm.motor3;
      rpms[BACK_RIGHT] = (float)rpm.motor4;
      double rpms1[kMotorCount];
      rpms1[FRONT_LEFT] = (float)rpm1.motor1;
      rpms1[FRONT_RIGHT] = (float)rpm1.motor2;
      rpms1[BACK_LEFT] = (float)rpm1.motor3;
      rpms1[BACK_RIGHT] = (float)rpm1.motor4;

      dtostrf(angular_z, 6, 2, result);
      sprintf(log_msg, "Angular Z :%s", result);
      nh->loginfo(log_msg);
      dtostrf(z, 6, 2, result);
      sprintf(log_msg, "Z :%s", result);
      nh->loginfo(log_msg);
      dtostrf(rpms[FRONT_RIGHT], 6, 2, result);
      sprintf(log_msg, "M3 RPM :%s", result);
      nh->loginfo(log_msg);
      dtostrf(rpms[BACK_RIGHT], 6, 2, result);
      sprintf(log_msg, "M4 RPM :%s", result);
      nh->loginfo(log_msg);

      dtostrf(rpms1[FRONT_LEFT], 6, 2, result);
      sprintf(log_msg, "M1 RPM :%s", result);
      nh->loginfo(log_msg);
      dtostrf(rpms1[BACK_LEFT], 6, 2, result);
      sprintf(log_msg, "M2 RPM :%s", result);
      nh->loginfo(log_msg);
      dtostrf(rpms1[FRONT_RIGHT], 6, 2, result);
      sprintf(log_msg, "M3 RPM :%s", result);
      nh->loginfo(log_msg);
      dtostrf(rpms1[BACK_RIGHT], 6, 2, result);
      sprintf(log_msg, "M4 RPM :%s", result);
      nh->loginfo(log_msg);
    }
  }
  else
  {
    Kinematics::output pwm = kinematics.getPWM(x, y, z);

    motor[FRONT_LEFT].motorSpeedPWM(pwm.motor1);
    motor[FRONT_RIGHT].motorSpeedPWM(pwm.motor2);
    motor[BACK_LEFT].motorSpeedPWM(pwm.motor3);
    motor[BACK_RIGHT].motorSpeedPWM(pwm.motor4);

    if (debug)
    {
      char log_msg[20];
      char result[8];

      double pwms[kMotorCount];
      pwms[FRONT_LEFT] = pwm.motor1;
      pwms[FRONT_RIGHT] = pwm.motor2;
      pwms[BACK_LEFT] = pwm.motor3;
      pwms[BACK_RIGHT] = pwm.motor4;

      dtostrf(pwms[FRONT_LEFT], 6, 2, result);
      sprintf(log_msg, "M1 PWM :%s", result);
      nh->loginfo(log_msg);

      dtostrf(pwms[BACK_LEFT], 6, 2, result);
      sprintf(log_msg, "M2 PWM :%s", result);
      nh->loginfo(log_msg);

      dtostrf(pwms[FRONT_RIGHT], 6, 2, result);
      sprintf(log_msg, "M3 PWM :%s", result);
      nh->loginfo(log_msg);

      dtostrf(pwms[BACK_RIGHT], 6, 2, result);
      sprintf(log_msg, "M4 PWM :%s", result);
      nh->loginfo(log_msg);
    }
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

void Movement::updatePIDKinematics(Kinematics::output rpm)
{
  // Matching of motor # with rpm.motor# is important. Check kinematics getRPM()
  motor[FRONT_LEFT].motorSpeedPID(rpm.motor1);
  motor[FRONT_RIGHT].motorSpeedPID(rpm.motor2);
  motor[BACK_LEFT].motorSpeedPID(rpm.motor3);
  motor[BACK_RIGHT].motorSpeedPID(rpm.motor4);
}

void Movement::updateStraightPID(int RPMs)
{
  motor[FRONT_LEFT].motorSpeedPID(RPMs, false);
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
  if (bno == nullptr)
  {
    Serial.println("ERROR, trying to use BNO without initializing it. Change constructor or method call.");
    return;
  }

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
  if (bno == nullptr)
  {
    Serial.println("ERROR, trying to use BNO without initializing it. Change constructor or method call.");
    return;
  }

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
}

// Drop kit decider

// Gets sign which refers to where should a kit be dropped
void Movement::dropDecider(int ros_sign_callback)
{
  digitalWrite(kDigitalPinsLEDS[1], HIGH);

  double time = millis();

  while (ros_sign_callback > 0){
    dispenser.rightDrop();
    ros_sign_callback--;
  }

  while (ros_sign_callback < 0){
    dispenser.leftDrop();
    ros_sign_callback++;
  }
  
  // Wait for 5 seconds to turn off led.
  while (((millis() - time) / 1000.0) < 5)
    delay(0.1);
  
  digitalWrite(kDigitalPinsLEDS[1], LOW);
}

// Limit switches checker

void Movement::testMotor()
{

  // Motor *m = &motor[FRONT_RIGHT];
  Motor *m = &motor[BACK_LEFT];
  // Motor *m = &motor[BACK_RIGHT];
  //  Motor *m = &motor[FRONT_LEFT];

  while (true)
  {
    Serial.print("Curr speed: ");
    Serial.print(m->getCurrentSpeed());
    Serial.print(", ");
    Serial.print("Curr target: ");
    Serial.println(m->getTargetSpeed());
  }
}

void Movement::testAllMotors()
{
  Serial.println("Testing all motors");
  delay(1000);

  Serial.println("FRONT RIGHT - Forward");
  motor[FRONT_RIGHT].motorSpeedPID(90);
  delay(2000);
  motor[FRONT_RIGHT].motorSpeedPID(0);
  delay(1000);

  Serial.println("FRONT RIGHT - Backwards");
  motor[FRONT_RIGHT].motorSpeedPID(-90);
  delay(2000);
  motor[FRONT_RIGHT].motorSpeedPID(0);
  delay(1000);

  Serial.println("FRONT LEFT - Forward");
  motor[FRONT_LEFT].motorSpeedPID(90);
  delay(2000);
  motor[FRONT_LEFT].motorSpeedPID(0);
  delay(1000);

  Serial.println("FRONT LEFT - Backwards");
  motor[FRONT_LEFT].motorSpeedPID(-90);
  delay(2000);
  motor[FRONT_LEFT].motorSpeedPID(0);
  delay(1000);

  Serial.println("BACK LEFT - Forward");
  motor[BACK_LEFT].motorSpeedPID(90);
  delay(2000);
  motor[BACK_LEFT].motorSpeedPID(0);
  delay(1000);

  Serial.println("BACK LEFT - Backwards");
  motor[BACK_LEFT].motorSpeedPID(-90);
  delay(2000);
  motor[BACK_LEFT].motorSpeedPID(0);
  delay(1000);

  Serial.println("BACK RIGHT - Forward");
  motor[BACK_RIGHT].motorSpeedPID(90);
  delay(2000);
  motor[BACK_RIGHT].motorSpeedPID(0);
  delay(1000);

  Serial.println("BACK RIGHT - Backwards");
  motor[BACK_RIGHT].motorSpeedPID(-90);
  delay(2000);
  motor[BACK_RIGHT].motorSpeedPID(0);
  delay(1000);

  while (true)
    delay(1000);
}