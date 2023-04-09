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
  motor[BACK_LEFT].PIDStraightTunings(470, 80, 15);
  motor[FRONT_RIGHT].PIDStraightTunings(120, 80, 10);
  motor[FRONT_LEFT].PIDStraightTunings(120, 80, 10);
  motor[BACK_RIGHT].PIDStraightTunings(120, 80, 10);

  motor[BACK_LEFT].PIDConservativeTunings(40, 80, 10);
  motor[FRONT_RIGHT].PIDConservativeTunings(40, 80, 10);
  motor[FRONT_LEFT].PIDConservativeTunings(40, 80, 10);
  motor[BACK_RIGHT].PIDConservativeTunings(40, 80, 10);

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
    // Esta dentro del rango
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
}

void Movement::initLeds()
{
  pinMode(kDigitalPinsLEDS[0], OUTPUT);
  pinMode(kDigitalPinsLEDS[1], OUTPUT);
}

// Encoder Functions

double Movement::meanDistanceTraveled()
{
  double sum_encoder_distance = 0.0;
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

int Movement::cmdMovement(const int action, const int option)
{
  switch (action)
  {
  case 1:
    // Move forward 1 unit. TODO: add option of navigating with vlx distance.
    if (option == 1)
    {
      advanceXMeters(0.3, true);
    }
    else
    {
      advanceXMeters(0.3, false);
    }
    return 1;
    break;

  case 2:
    // Left turn
    rDirection = getTurnDirection(0);
    goToAngle(dirToAngle(rDirection));
    return 1;
    break;

  case 3:
    // Right turn
    rDirection = getTurnDirection(1);
    goToAngle(dirToAngle(rDirection));

    return 1;
    break;

  case 4:
    // Move back 1 unit
    if (option == 1)
    {
      advanceXMeters(-0.3, true);
    }
    else
    {
      advanceXMeters(-0.3, false);
    }
    return 1;
    break;
  case 5:
    // Rearrange in tile. Use VLX and BNO.
    goToAngle(getTurnDirection(rDirection)); // Rearrange orientation
    advanceXMeters(getDistanceToCenter(), true); // Get error in X, and move that distance.
    return 1;
    break;

  case 6:
    // Traverse ramp
    traverseRamp();
    return 1;
    break;

  case 7:
    // Drop n kits
    dropDecider(option);
    return 1;
    break;

  case 8:
    // Update angle reference.
    return 1;
    break;

  default:
    break;
  }
}

void Movement::girarIzquierda()
{
  motor[FRONT_LEFT].motorBackward();
  motor[BACK_LEFT].motorBackward();
  motor[FRONT_RIGHT].motorForward();
  motor[BACK_RIGHT].motorForward();
}

void Movement::girarDerecha()
{
  motor[FRONT_LEFT].motorForward();
  motor[BACK_LEFT].motorForward();
  motor[FRONT_RIGHT].motorBackward();
  motor[BACK_RIGHT].motorBackward();
}

void Movement::updatePIDKinematics(Kinematics::output rpm)
{
  // Matching of motor # with rpm.motor# is important. Check kinematics getRPM()
  motor[FRONT_LEFT].motorSpeedPID(rpm.motor1);
  motor[FRONT_RIGHT].motorSpeedPID(rpm.motor2);
  motor[BACK_LEFT].motorSpeedPID(rpm.motor3);
  motor[BACK_RIGHT].motorSpeedPID(rpm.motor4);
}

void Movement::updateStraightPID(int RPMs, double errorD)
{
  double factor = 10; // Increasing the factor increases the speed correction.

  motor[FRONT_LEFT].motorSpeedPID(RPMs + (errorD * factor));
  motor[BACK_LEFT].motorSpeedPID(RPMs + (errorD * factor));
  motor[FRONT_RIGHT].motorSpeedPID(RPMs + (errorD * factor * -1));
  motor[BACK_RIGHT].motorSpeedPID(RPMs + (errorD * factor * -1));
}

// A possitive errorD means that the robot must increase the speed of the right wheels.
void Movement::updateStraightPID(int RPMs)
{

  // Use angle error to update target speeds.
  motor[FRONT_LEFT].motorSpeedPID(RPMs, false);
  motor[BACK_LEFT].motorSpeedPID(RPMs, false);
  motor[FRONT_RIGHT].motorSpeedPID(RPMs);
  motor[BACK_RIGHT].motorSpeedPID(RPMs);
}

void Movement::advanceXMeters(double x, bool useAngleError)
{
  double dist = meanDistanceTraveled(); // Use encoders;

  double initial = dist;
  double target = dist + x;

  if (x > 0)
  {
    while (dist < target)
    {
      if (useAngleError)
      {
        double angleError = getAngleError(dirToAngle(rDirection));
        updateStraightPID(kMovementRPMs, angleError);
      }
      else
      {
        updateStraightPID(kMovementRPMs);
      }
      // Serial.println("Distancia recorrida: " + String(dist - initial));
      dist = meanDistanceTraveled();
    }
  }
  else
  {
    while (target < dist)
    {
      if (useAngleError)
      {
        double angleError = getAngleError(dirToAngle(rDirection));
        updateStraightPID(-kMovementRPMs, angleError);
      }
      else
      {
        updateStraightPID(-kMovementRPMs);
      }
      dist = meanDistanceTraveled();
    }
  }

  stop();
  resetEncoders();
}
/*
Find best option to rotate given current and target angle.

bool turnRight = false;

      double diff = bno->getAngleX() - rAngle;
      if (diff > 0 && diff < 180)
      {
        turnRight = false;
      }
      else
      {
        turnRight = true;
      }

      goToAngle(rAngle, turnRight);
*/

double Movement::getAngleError(double expectedAngle)
{
  double angle = sensors->getAngleX();

  if (expectedAngle == 0)
  {
    if (abs(expectedAngle - angle) > 90)
      return 360 - angle;
  }
  return expectedAngle - angle;
}

double Movement::getDistanceToCenter()
{
  double dist = 0;
  dist = sensors->getVLXInfo(vlx_front);
  dist *= 100; // m to cm.
  double center = ((int)dist % 15) - 5; // in cm. "-5" is the distance from vlx to wall.
  
  return center / 100.0;                  // Return distance in m.
}

void Movement::goToAngle(int targetAngle)
{

  double current_angle = bno->getAngleX();

  double diff = bno->getAngleX() - targetAngle;

  while (abs(diff) > 1)
  {

    bool turnRight = false;
    if (diff > 0 && diff <= 180)
    {
      turnRight = false;
    }
    else
    {
      turnRight = true;
    }

    updateRotatePID(targetAngle, turnRight);

    diff = bno->getAngleX() - targetAngle;
  }

  stop();
}

void Movement::updateRotatePID(int targetAngle, bool right)
{
  double current_angle = bno->getAngleX();
  if (right)
  {
    girarDerecha();
    motor[FRONT_LEFT].motorRotateDerPID(targetAngle, current_angle);
    motor[BACK_LEFT].motorRotateDerPID(targetAngle, current_angle);
    motor[FRONT_RIGHT].motorRotateDerPID(targetAngle, current_angle);
    motor[BACK_RIGHT].motorRotateDerPID(targetAngle, current_angle);
  }
  else
  {
    girarIzquierda();
    motor[FRONT_LEFT].motorRotateIzqPID(targetAngle, current_angle);
    motor[BACK_LEFT].motorRotateIzqPID(targetAngle, current_angle);
    motor[FRONT_RIGHT].motorRotateIzqPID(targetAngle, current_angle);
    motor[BACK_RIGHT].motorRotateIzqPID(targetAngle, current_angle);
  }
}

// Drop kit decider

// Gets sign which refers to where should a kit be dropped
void Movement::dropDecider(int ros_sign_callback)
{
  digitalWrite(kDigitalPinsLEDS[1], HIGH);

  double time = millis();

  dispenser.dropNKits(ros_sign_callback);

  // Wait for 5 seconds to turn off led.
  while (((millis() - time) / 1000.0) < 5)
    delay(0.1);

  digitalWrite(kDigitalPinsLEDS[1], LOW);
}

void Movement::traverseRamp(int option){
  // Advance first if option is set to 1

  if (option == 1){
    updateStraightPID(kMovementRPMs);
    delay(1000);
  }
  double yAngle = sensors->getAngleY();

  while (yAngle > 10 || yAngle < -10){
    double rightVlx = 
    double leftVlx = sensors->();
    updateStraightPID(kMovementRPMs, vlx_right, vlx_left)
  }


  stop();
}

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

// Robot state

int Movement::dirToAngle(int rdirection)
{
  switch (rdirection)
  {
  case 0:
    return 0; // North yaw
    break;

  case 1:
    return 90; // East yaw
    break;

  case 2:
    return 180; // South yaw
    break;

  case 3:
    return 270; // West yaw
    break;

  default:
    break;
  }
}

int Movement::getTurnDirection(int turn)
{
  if (turn) // right
  {
    if (rDirection == 3)
    {
      return 0;
    }
    else
    {
      return rDirection + 1;
    }
  }
  else // left
  {
    if (rDirection == 0)
    {
      return 3;
    }
    else
    {
      return rDirection - 1;
    }
  }

  return -1;
}