#include "Movement.h"

// Constructors

Movement::Movement(ros::NodeHandle *nh, BNO *bno, Sensors *sensors, bool individualConstants) : nh(nh), bno(bno), sensors(sensors)
{
  initMovement(individualConstants);
}

Movement::Movement(BNO *bno, Sensors *sensors, bool individualConstants, Dispenser *d) : bno(bno), sensors(sensors), dispenser(d)
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

Movement::Movement(bool individualConstants)
{
  sensors = nullptr;
  nh = nullptr;
  bno = nullptr;
  initMovement(individualConstants);
}

void Movement::initMovement(bool individualConstants)
{
  kinematics = Kinematics(kRPM, kWheelDiameter, kFrWheelsDist, kLrWheelsDist, kPwmBits);
  setMotors();
  // this->dispenser = Dispenser(kServoPin);
  // dispenser.stop();
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
  // Motors were swapped. Check which motor corresponss to pins
  motor[FRONT_RIGHT] = Motor(kDigitalPinsFrontLeftMotor[1], kDigitalPinsFrontLeftMotor[0],
                             kAnalogPinFrontLeftMotor, kEncoderPinsFrontLeftMotor[0],
                             kEncoderPinsFrontLeftMotor[1], MotorID::frontLeft);
  motor[BACK_LEFT] = Motor(kDigitalPinsBackLeftMotor[1], kDigitalPinsBackLeftMotor[0],
                           kAnalogPinBackLeftMotor, kEncoderPinsBackLeftMotor[0],
                           kEncoderPinsBackLeftMotor[1], MotorID::backLeft);
  motor[FRONT_LEFT] = Motor(kDigitalPinsFrontRightMotor[1], kDigitalPinsFrontRightMotor[0],
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

  // dispenser.initServo();
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
  if (CK::kusingROS)
    return;

  for (int i = 0; i < kMotorCount; i++)
  {
    /*
    // Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(motor[i].getEncoderTics());
    delay(10);
    */
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
  if (nh == nullptr && !CK::kusingROS)
  {
    // Serial.println("ERROR, trying to use NodeHandle without initializing it. Change constructor or method call.");
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

double Movement::cmdMovement(const int action, const int option)
{
  if (!sensors->readMotorInit())
  {
    firstMove = false;
    return -2;
  }

  // nh->loginfo("cmdMovement called");
  // nh->spinOnce();

  if (firstMove)
  {
    sensors->resetScreen();
    firstMove = false;
    basePitch = sensors->getAngleY();

    updateAngleReference();
    sensors->logActive("Updated angle reference", true, 0, 1);
    sensors->bothLedOn();
    delay(50);
    sensors->bothLedOff();
    delay(50);

    sensors->bothLedOn();
    delay(50);
    sensors->bothLedOff();
    delay(50);

    sensors->bothLedOn();
    delay(500);
    sensors->bothLedOff();
  }

  switch (action)
  {
  case 0:
    // nh->loginfo("Moving forward");
    sensors->logActive("Moving forward", true, 0, 0);
    // Move forward 1 unit.

    motor[FRONT_LEFT].pidStraight.useConservative = true;
    motor[FRONT_RIGHT].pidStraight.useConservative = true;
    motor[BACK_LEFT].pidStraight.useConservative = true;
    motor[BACK_RIGHT].pidStraight.useConservative = true;
    return advanceXMeters(0.3, option);
    break;

  case 1:
    // nh->loginfo("Turning right");
    // Right turn
    sensors->logActive("Turning right", true, 0, 0);
    rotateRobot(option, 1);
    return 1;
    break;

  case 2:
    // nh->loginfo("Moving backward");
    // Move back 1 unit
    sensors->logActive("Moving backward", true, 0, 0);

    motor[FRONT_LEFT].pidStraight.useConservative = true;
    motor[FRONT_RIGHT].pidStraight.useConservative = true;
    motor[BACK_LEFT].pidStraight.useConservative = true;
    motor[BACK_RIGHT].pidStraight.useConservative = true;
    return advanceXMeters(-0.3, option);
    break;

  case 3:
    // nh->loginfo("Turning left");
    // Left turn
    sensors->logActive("Turning left", true, 0, 0);
    rotateRobot(option, 0);
    return 1;
    break;

  case 4:
    // nh->loginfo("Traversing ramp");
    // Traverse ramp
    sensors->logActive("Traversing ramp", true, 0, 0);
    traverseRamp(option);
    return 5;
    break;

  case 5:
    // nh->loginfo("Traversing ramp");
    // Rearrange in tile. Use VLX and BNO.
    goToAngle(getTurnDirection(rDirection)); // Rearrange orientation
    advanceUntilCentered();
    return 1;
    break;

  case 7:
    // nh->loginfo("Dropping n kits");
    // Drop n kits
    sensors->logActive("Drop kits", true, 0, 0);
    dropDecider(option);
    return 1;
    break;

  case 8:
    // nh->loginfo("Update Angle reference");
    // Update angle reference.
    updateAngleReference();
    return 1;
    break;

  case 10:
    // nh->loginfo("Moving forward (fast)");
    // Move forward 1 unit.
    sensors->logActive("Moving forward (fast)", true, 0, 0);

    motor[FRONT_LEFT].pidStraight.useConservative = false;
    motor[FRONT_RIGHT].pidStraight.useConservative = false;
    motor[BACK_LEFT].pidStraight.useConservative = false;
    motor[BACK_RIGHT].pidStraight.useConservative = false;

    return advanceXMeters(0.3, option);

    break;

  case 12:
    sensors->logActive("Moving backward (fast)", true, 0, 0);
    // Move back 1 unit
    motor[FRONT_LEFT].pidStraight.useConservative = false;
    motor[FRONT_RIGHT].pidStraight.useConservative = false;
    motor[BACK_LEFT].pidStraight.useConservative = false;
    motor[BACK_RIGHT].pidStraight.useConservative = false;

    return advanceXMeters(-0.3, option);

    break;

  default:
    break;
  }
}

void Movement::rotateRobot(int option, int dir)
{
  sensors->logActive("rotateRobot", true, 0, 1);
  // nh->loginfo("rotateRobot called");
  bool reacomodate = false;

  // Move robot backward if there is a wall back.
  if (option == 1)
  {
    double dist = 100;
    dist = (dir == 0) ? sensors->getDistInfo(dist_right) : sensors->getDistInfo(dist_left);
    if (dist < kDistanceWall)
      reacomodate = true;
  }

  // Dir 1 means right turn. O is for left turn.
  rDirection = getTurnDirection(dir); // Obtain target direction based on input.
  goToAngle(dirToAngle(rDirection));

  if (reacomodate && option == 1)
  {
    if (!CK::kusingROS && CK::debugRotation)
    {
      Serial.println("Reacomodating");
    }

    updateBasePWM(-1);

    delay(2000);

    updateBasePWM(1);

    delay(200);

    stop();
    resetEncoders();
    updateAngleReference();
  }

  // delay(100);
  // if (dir == 1)
  //   translationX(0.3);
  // else
  //   translationX(-0.3);

  stop();
  resetEncoders();
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

// Update PID with either VLX or BNO error.
void Movement::updateStraightPID(int RPMs, bool useBNO, bool ramp)
{
  sensors->logActive("upd s PID", true, 0, 2);
  double errorD = 0;
  double bnoFactor = 10;
  // Debug motors status
  if (!CK::kusingROS)
  {
    getMotorStatus(FRONT_LEFT);
    getMotorStatus(BACK_LEFT);
  }

  // motor[FRONT_LEFT].motorForward();
  // motor[BACK_LEFT].motorForward();
  // motor[FRONT_RIGHT].motorForward();
  // motor[BACK_RIGHT].motorForward();

  // nh->loginfo("updateStraightPID called");
  if (useBNO || CK::pidBoth)
  {
    errorD = getAngleError(dirToAngle(rDirection));

    // Acotate error if it is too big.
    errorD = min(45, max(-45, errorD));

    if (!CK::kusingROS)
    {
      double correction = errorD * bnoFactor;
      Serial.print("ErrorD: ");
      Serial.print(correction);
      Serial.print(", Target rpm: left: ");
      Serial.print(RPMs + correction);
      Serial.print(", Target rpm: right: ");
      Serial.print(RPMs + correction * -1);
    }
  }
  if (!useBNO || CK::pidBoth)
  {
    // Use VLX distance error to update target speeds.
    if (!CK::kusingROS)
      Serial.println("UpdateStraightPID");
    if (millis() - lastUpdateVLX > kVlxErrorTimer)
    {
      lastUpdateVLX = millis();
      double rightDistance = sensors->getDistInfo(dist_right);
      double leftDistance = sensors->getDistInfo(dist_left);

      while (rightDistance > 0.3)
        rightDistance -= 0.3;
      while (leftDistance > 0.3)
        leftDistance -= 0.3;

      // A positive correction means that the robot must move to the right
      this->correctionVLX = 100 * (rightDistance - leftDistance) * 2;
    }

    if (!CK::kusingROS && CK::vlxPID)
    {

      Serial.print("Correction: ");
      Serial.print(correctionVLX);
      Serial.print(", Target rpm: left: ");
      Serial.print(RPMs + correctionVLX);
      Serial.print(", Target rpm: right: ");
      Serial.print(RPMs + correctionVLX * -1);
      Serial.print("Real Error: ");
      Serial.println(correctionVLX / 100);
    }
  }

  if (CK::pidBoth)
  {
    double combinedCorrection = ((errorD * bnoFactor) * 1.5 + (correctionVLX)*.5) / 2;
    motor[FRONT_LEFT].motorSpeedPID(RPMs + combinedCorrection);
    motor[BACK_LEFT].motorSpeedPID(RPMs + combinedCorrection);
    motor[FRONT_RIGHT].motorSpeedPID(RPMs + combinedCorrection * -1);
    motor[BACK_RIGHT].motorSpeedPID(RPMs + combinedCorrection * -1);
  }
  else if (!CK::pidBoth && useBNO)
  {
    ramp = false;
    if (ramp)
    {
      bool upwards;

      if (sensors->getAngleY() < maxPitch + basePitch)
        upwards = true; // robot is moving up.

      // If robot is moving on ramp, dont reduce speed of one side.
      double leftRPMs = max(RPMs + (errorD * bnoFactor), RPMs);
      double rightRPMs = max(RPMs + (errorD * bnoFactor * -1), RPMs);

      // Add rmps to side crashing against wall
      bool l, r;
      sensors->getLimitSwitches(l, r);

      if (upwards)
      {
        leftRPMs += (l ? 10 : 0);
        rightRPMs += (r ? 10 : 0);
      }
      else
      {
        leftRPMs += (r ? -10 : 0);
        rightRPMs += (l ? -10 : 0);
      }

      motor[FRONT_LEFT].motorSpeedPID(leftRPMs);
      motor[BACK_LEFT].motorSpeedPID(leftRPMs);
      motor[FRONT_RIGHT].motorSpeedPID(rightRPMs);
      motor[BACK_RIGHT].motorSpeedPID(rightRPMs);
    }
    else
    {
      // Possitive errorD moves the robot towards the right
      motor[FRONT_LEFT].motorSpeedPID(RPMs + (errorD * bnoFactor));
      motor[BACK_LEFT].motorSpeedPID(RPMs + (errorD * bnoFactor));
      motor[FRONT_RIGHT].motorSpeedPID(RPMs + (errorD * bnoFactor * -1));
      motor[BACK_RIGHT].motorSpeedPID(RPMs + (errorD * bnoFactor * -1));
    }
  }
  else if (!CK::pidBoth && !useBNO)
  {
    motor[FRONT_LEFT].motorSpeedPID(RPMs + correctionVLX);
    motor[BACK_LEFT].motorSpeedPID(RPMs + correctionVLX);
    motor[FRONT_RIGHT].motorSpeedPID(RPMs + correctionVLX * -1);
    motor[BACK_RIGHT].motorSpeedPID(RPMs + correctionVLX * -1);
  }
}

bool Movement::checkColor()
{
  // nh->loginfo("checkColor called");
  // sensors->toggleRightLed();
  char c = sensors->getTCSInfo();
  if (!CK::kusingROS)
  {
    Serial.print("TCS info:");
    Serial.println(c);
  }

  // sensors->toggleRightLed();
  return c == 'N';
}

// pass RPMs directly to PID.
void Movement::updateStraightPID(int RPMs)
{
  // nh->loginfo("updateStraightPID called");
  motor[FRONT_LEFT].motorSpeedPID(RPMs);
  motor[BACK_LEFT].motorSpeedPID(RPMs);
  motor[FRONT_RIGHT].motorSpeedPID(RPMs);
  motor[BACK_RIGHT].motorSpeedPID(RPMs);
}

void Movement::updateVelPwm(int RPMs, bool useBNO)
{
  // nh->loginfo("updateVelPwm called");

  int speed_sign = 0;

  if (RPMs > 0)
    speed_sign = 1;
  else if (RPMs < 0)
    speed_sign = -1;

  if (useBNO)
  {
    double errorD = getAngleError(dirToAngle(rDirection));
    double factor = 10; // Increasing the factor increases the speed correction.
    leftM = errorD * factor;
    rightM = leftM * -1;
  }
  else
  {
    // Use VLX distance error to update target speeds.
    // Serial.println("UpdateStraightPID");
    if (millis() - lastUpdateVLX > kVlxErrorTimer)
    {
      lastUpdateVLX = millis();
      double rightDistance = sensors->getDistInfo(dist_right);
      double leftDistance = sensors->getDistInfo(dist_left);

      while (rightDistance > 0.3)
        rightDistance -= 0.3;
      while (leftDistance > 0.3)
        leftDistance -= 0.3;

      // A positive correction means that the robot must move to the right
      this->correctionVLX = 100 * (rightDistance - leftDistance) * 2;
    }

    leftM = correctionVLX;
    rightM = correctionVLX * -1;
  }

  // Update PWM taking into account error.
  updateBasePWM(speed_sign, leftM, rightM);
}

void Movement::updateBasePWM(int dir, double leftCorrection, double rightCorrection)
{
  sensors->logActive("upd base pwm", true, 0, 2);
  motor[FRONT_LEFT].setPWM(leftCorrection + CK::basePwmFrontLeft, dir);
  motor[BACK_LEFT].setPWM(leftCorrection + CK::basePwmBackLeft, dir);
  motor[FRONT_RIGHT].setPWM(rightCorrection + CK::basePwmFrontRight, dir);
  motor[BACK_RIGHT].setPWM(rightCorrection + CK::basePwmBackRight, dir);
}

void Movement::updateMaxPWM(int dir)
{
  sensors->logActive("upd max pwm", true, 0, 2);
  motor[FRONT_LEFT].setPWM(255, dir);
  motor[BACK_LEFT].setPWM(255, dir);
  motor[FRONT_RIGHT].setPWM(255, dir);
  motor[BACK_RIGHT].setPWM(255, dir);
}

void Movement::updateVelocityDecider(int RPMs, bool useBNO)
{
  // nh->loginfo("updateVelocityDecider called");
  if (CK::usePid)
  {
    updateStraightPID(RPMs, useBNO);
  }
  else
  {
    updateVelPwm(RPMs, useBNO);
  }
}

double Movement::advanceXMetersVLX(double x, int straightPidType, bool forceBackward)
{
  // nh->loginfo("AdvanceXMeters called");
  sensors->logActive("adv xM vlx", true, 0, 2);
  double dist = sensors->getDistInfo(dist_front);
  double distBack = sensors->getDistInfo(dist_back);
  bool useFrontalVlx = false;
  if (dist < distBack)
  {
    useFrontalVlx = true;
  }
  else
  {
    dist = distBack;
  }

  // Use encoders to move if distance is greater than 1.28m, as its the vlx max range.
  /*if (dist > 1.28){
    useEncoders = true;
    dist = meanDistanceTraveled();
  }*/

  double initial = 0;
  double target = 0;
  if (useFrontalVlx)
  {
    sensors->logActive("using front vlx", true, 0, 2);
    initial = dist;
    target = dist - x + 0.02;
  }
  else
  {
    sensors->logActive("using back vlx", true, 0, 2);
    initial = distBack;
    target = dist + x + 0.02;
  }

  double lastTCS = millis();

  if (x > 0)
  {
    if (useFrontalVlx)
    {
      while (dist > target && dist > 0.08)
      {
        sensors->logActive("En advanceXMetersVLX", true, 0);
        // rosBridge->readOnce();
        if (!sensors->readMotorInit())
          return -2;

        updateVelocityDecider(kMovementRPMs, straightPidType);

        handleSwitches();
        // Get dist reading after correcting angle.
        rearrangeAngle(8);

        if (outOfPitch())
        {
          double dt = stabilizePitch(straightPidType, false);
          if (abs(dt) > CK::kRampDt)
            return dt; // Return number to indicate robot traversed ramp. Positive dt means robot went up.
        }

        if (millis() - lastTCS > checkTCSTimer && abs(initial-dist) < abs(0.3 - 0.1))
        {
          lastTCS = millis();
          if (checkColor())
          {
            stop();
            dist = sensors->getDistInfo(dist_front);
            // return 0;
            return advanceXMeters(dist - initial, straightPidType, true);
          }
        }

        dist = sensors->getDistInfo(dist_front);
        sensors->logActive("DF:" + String(dist), true, 0, 6);
        // sensors->logActive("Distancia recorrida advanceXMeters", initial - dist)
      }
    }
    else
    {
      double df = sensors->getDistInfo(dist_front);
      while (dist < target && df > 0.08)
      {
        sensors->logActive("En advanceXMetersVLX", true, 0);
        // rosBridge->readOnce();
        if (!sensors->readMotorInit())
          return -2;

        updateVelocityDecider(kMovementRPMs, straightPidType);

        handleSwitches();
        // Get dist reading after correcting angle.
        rearrangeAngle(8);

        if (outOfPitch())
        {
          double dt = stabilizePitch(straightPidType, false);
          if (abs(dt) > CK::kRampDt)
            return dt; // Return number to indicate robot traversed ramp. Positive dt means robot went up.
        }

        if (millis() - lastTCS > checkTCSTimer && abs(initial - dist) < abs(0.3 - 0.1))
        {
          lastTCS = millis();
          if (checkColor())
          {
            stop();
            dist = sensors->getDistInfo(dist_back);
            // return 0;
            return advanceXMeters(initial - dist, straightPidType, true);
          }
        }

        dist = sensors->getDistInfo(dist_back);
        // sensors->logActive("Distancia recorrida advanceXMeters", initial - dist)
        df = sensors->getDistInfo(dist_front);
      }
    }
  }
  else
  {
    // Use to check if robot is stuck.
    double changeT = millis();
    double prevDist = dist;
    double backDist = sensors->getDistInfo(dist_back);
    if (useFrontalVlx)
    {
      // Don't check color and stable pitch. Assume negative movement only for black tiles.
      while (dist < target && backDist > 0.05)
      {
        // sensors->logActive("En advanceXMeters -1", true, 0);
        //  rosBridge->readOnce();
        if (!sensors->readMotorInit())
          return -2;
        updateVelocityDecider(-kMovementRPMs, straightPidType);

        // Get dist reading after correcting angle.
        rearrangeAngle(8);

        dist = sensors->getDistInfo(dist_front);
        // sensors->logActive("Dist rec advanceXMeters:" + String(initial - dist), true, 0, 1);

        // If robot hasn't moved significantly in backStuckTimer, break.
        if (millis() - changeT > backStuckTimer)
        {
          break;
        }

        prevDist = dist;
        backDist = sensors->getDistInfo(dist_back);
      }
    }
    else
    {
      // Don't check color and stable pitch. Assume negative movement only for black tiles.
      while (dist > target && dist > 0.05)
      {
        // sensors->logActive("En advanceXMeters -1", true, 0);
        //  rosBridge->readOnce();
        if (!sensors->readMotorInit())
          return -2;

        updateVelocityDecider(-kMovementRPMs, straightPidType);

        // Get dist reading after correcting angle.
        rearrangeAngle(8);

        dist = sensors->getDistInfo(dist_back);
        // sensors->logActive("Dist rec advanceXMeters:" + String(initial - dist), true, 0, 1);

        // If robot hasn't moved significantly in backStuckTimer, break.
        if (millis() - changeT > backStuckTimer)
        {
          break;
        }

        prevDist = dist;
      }
    }
  }

  stop();
  resetEncoders();

  // sensors->logActive("Distancia final recorrida advanceXMetersVLX: " + String(initial - dist), true, 0, 1);

  // Indicate the robot went backwards because of black tile.
  if (forceBackward)
    return 0;

  return 1;
}

void Movement::handleSwitches()
{
  bool right = 0, left = 0;
  sensors->getLimitSwitches(right, left);
  if (right)
  {
    sensors->logActive("Handle rl switch", true, 0, 3);
    handleRightLimitSwitch();
  }
  else if (left)
  {
    sensors->logActive("Handle ll switch", true, 0, 3);
    handleLeftLimitSwitch();
  }
}

double Movement::advanceXMeters(double x, int straigthPidType, bool forceBackwards)
{
  sensors->logActive("advanceXMeters", true, 0, 1);
  double dist = sensors->getDistInfo(dist_front);
  double distBack = sensors->getDistInfo(dist_back);

  if (dist > 1.15 && distBack > 1.15)
  {
    // sensors->logActive("Use encoders advance", true, 0, 6, true);
    return advanceXMetersEncoders(x, straigthPidType, forceBackwards);
  }
  else
  {
    // sensors->logActive("Use vlx", true, 0, 6, true);
    return advanceXMetersVLX(x, straigthPidType, forceBackwards);
  }
}

double Movement::advanceXMetersEncoders(double x, int straightPidType, bool forceBackwards)
{
  sensors->logActive("adv xM enc", true, 0, 2);
  double dist = meanDistanceTraveled(); // Use encoders;

  double initial = dist;
  double target = dist + x;
  double lastTCS = millis();

  if (x > 0)
  {
    while (dist < target)
    {
      if (!sensors->readMotorInit())
        return -2;
      handleSwitches();
      updateVelocityDecider(kMovementRPMs, straightPidType);
      // Get dist reading after correcting angle.
      rearrangeAngle();
      if (outOfPitch())
      {
        double dt = stabilizePitch(straightPidType, false);
        if (abs(dt) > CK::kRampDt)
        {
          stop();
          return dt; // Return number to indicate robot traversed ramp. Positive dt means robot went up.
        }
      }
      if (millis() - lastTCS > checkTCSTimer && abs(initial - dist) < abs(0.3 - 0.1))
      {
        lastTCS = millis();
        if (checkColor())
        {
          dist = sensors->getDistInfo(dist_front);
          stop();
          // return 0;
          return advanceXMeters(initial - dist, straightPidType, true);
        }
      }

      // Get dist reading after correcting angle.
      // Serial.println("Distancia recorrida: " + String(dist - initial));
      dist = meanDistanceTraveled();
    }
  }
  else
  {
    double changeT = millis();
    while (dist > target)
    {
      // rosBridge->readOnce();
      if (!sensors->readMotorInit())
      {
        stop();
        return -2;
      }

      updateVelocityDecider(-kMovementRPMs, straightPidType);

      dist = meanDistanceTraveled();

      // If robot hasn't moved significantly in backStuckTimer, break.
      if (millis() - changeT > backStuckTimer)
      {
        break;
      }
    }
  }

  stop();
  resetEncoders();

  if (forceBackwards)
    return 0;

  return 1;
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

// Implementation with encoders

void Movement::advanceXMetersAbs(float x, int straightPidType)
{
  // nh->loginfo("AdvanceXMeters called");
  sensors->logActive("adv xM abs", true, 0, 1);
  double dist = meanDistanceTraveled(); // Use encoders;

  double initial = dist;
  double target = dist + x;
  double changeT = millis();

  if (x > 0)
  {
    while (dist < target)
    {
      if (millis() - changeT > 10000)
      {
        break;
      }

      sensors->logActive("En advanceXMetersAbs", true, 0);
      // rosBridge->readOnce();
      if (!sensors->readMotorInit())
        break;
      handleSwitches();
      updateVelocityDecider(kMovementRPMs, straightPidType);

      dist = meanDistanceTraveled();
    }
  }
  else
  {
    // Use to check if robot is stuck.
    double changeT = millis();
    double prevDist = dist;

    // Don't check color and stable pitch. Assume negative movement only for black tiles.
    while (dist > target)
    {
      // rosBridge->readOnce();
      if (!sensors->readMotorInit())
        break;
      updateVelocityDecider(-kMovementRPMs, straightPidType);
      dist = meanDistanceTraveled();

      sensors->logActive("Distancia recorrida advanceXMetersAbs: " + String(initial - dist), true, 0, 4, true);
      // If robot hasn't moved significantly in backStuckTimer, break.
      if (millis() - changeT > backStuckTimer)
      {
        break;
      }

      prevDist = dist;
    }
  }

  stop();
  sensors->logActive("Distancia final recorrida advanceXMetersAbs: " + String(initial - dist), true, 0, 1, true);
}

void Movement::advanceSlow(bool direction)
{
  if (direction)
  {
    updateVelocityDecider(10, CK::useBNO);
  }
  else
  {
    updateVelocityDecider(-10, CK::useBNO);
  }
}

void Movement::updateAngleReference()
{
  int currAngle = sensors->getAngleX();
  int currRdir = rDirection;
  angleDirs[currRdir] = currAngle;

  for (int i = 0; i < 3; i++)
  {
    currRdir++;
    currAngle += 90;

    if (currRdir > 3)
      currRdir = 0;
    if (currAngle > 360)
      currAngle -= 360;

    angleDirs[currRdir] = currAngle;
  }
  printAngleReference();
}

void Movement::updateAngleReference(int newAngle)
{
  int currAngle = newAngle;
  int currRdir = rDirection;
  angleDirs[currRdir] = currAngle;

  for (int i = 0; i < 3; i++)
  {
    currRdir++;
    currAngle += 90;

    if (currRdir > 3)
      currRdir = 0;
    if (currAngle > 360)
      currAngle -= 360;

    angleDirs[currRdir] = currAngle;
  }
}

void Movement::printAngleReference()
{
  if (CK::kusingROS)
    return;

  Serial.println("0 is north, 1 is east, 2 is south, 3 is west");

  for (int i = 0; i < 4; i++)
  {

    Serial.print("Angle: ");
    Serial.print(angleDirs[i]);
    Serial.print(" Dir: ");
    Serial.println(i);
  }
}

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

double Movement::stabilizePitch(int straightPidType, int RPMs, bool isRamp)
{
  sensors->logActive("stabilize p", true, 0, 7);
  double start = millis();
  double pitch = sensors->getAngleY();
  double dt = (millis() - start) / 1000.0;
  isRamp = true;

  bool sign = true;
  if (pitch < maxPitch + basePitch)
  {
    sign = true; // robot is moving up.
  }
  else
  {
    sign = false; // robot is moving down
  }
  while (outOfPitch())
  {
    // rosBridge->readOnce();
    // sensors->logActive("Stabilize pitch running.", true, 0, 0);
    if (!sensors->readMotorInit())
    {
      stop();
      return -2;
    }
    dt = (millis() - start) / 1000.0;
    if (dt > 9){
      isRamp = false;
    }
    // if ramp is upwards, then advance at max speed for a specified t time.
    if (sign && !isRamp)
    {
      updateMaxPWM(1);
      delay(200);
    }
    else
    {
      updateStraightPID(80, CK::useBNO, true);
    }
    if (!CK::kusingROS && CK::debugRamp)
    {
      Serial.print("Robot out of stable pitch: ");
      Serial.println(sensors->getAngleY());
    }

    // updateVelocityDecider(RPMs, straightPidType);
    // updateStraightPID(80, CK::useBNO, true);
  }

  dt = (millis() - start) / 1000.0;
  dt = dt * (sign ? 1 : -1); // apply sign
  stop();
  sensors->logActive("End stabilize pitch");

  if (sign)
  {
    sensors->logActive("Robot moved up.");
  }
  else
  {
    sensors->logActive("Robot moved down.");
  }

  return dt; // return dt in seconds.
}

bool Movement::outOfPitch()
{
  double pitch = sensors->getAngleY();
  return pitch > maxPitch + basePitch || pitch < minPitch + basePitch;
}

void Movement::rearrangeAngle(double tolerance)
{
  double angleError = getAngleError(dirToAngle(rDirection));
  if (abs(angleError) > tolerance)
    goToAngle(dirToAngle(rDirection), false);
}

double Movement::getDistanceToCenter()
{
  double dist = 0;
  dist = sensors->getDistInfo(dist_front);
  dist *= 100;                          // m to cm.
  double center = ((int)dist % 15) - 5; // in cm. "-5" is the distance from vlx to wall.

  return center / 100.0; // Return distance in m.
}

void Movement::advanceUntilCentered()
{

  double dist = sensors->getDistInfo(dist_front);
  sensors->logActive("Dist: " + String(dist));
  if (dist < 0.2)
  {
    while (dist > 0.05)
    {
      updateVelocityDecider(10, CK::useBNO);
      dist = sensors->getDistInfo(dist_front);
      sensors->logActive("Dist: " + String(dist));
      rearrangeAngle();
    }
  }

  stop();
}

// New implementationf
void Movement::goToAngle(int targetAngle, bool oneSide, bool handleSwitches, double timeout)
{
  bool lastDir = false;

  sensors->logActive("gotoangle", true, 0, 3);
  double currentAngle = bno->getAngleX();

  double diff = getAngleError(targetAngle);

  long int start = millis();

  while (abs(diff) > 2)
  {
    // rosBridge->readOnce();
    // sensors->logActive("GotoAngle");

    if (!sensors->readMotorInit() || (timeout && (millis() - start) > timeout))
    {
      stop();
      return;
    }

    if (handleSwitches)
    {
      bool l, r;
      sensors->getLimitSwitches(l, r);
      if (l || r)
      {
        updateBasePWM(-1);
        delay(100);
        stop();
      }
    }
    bool turnRight = false;
    if (diff > 0 && diff < 180 || diff < -180)
    {
      turnRight = true;
    }
    else
    {
      turnRight = false;
    }

    // In case direction has changed, reduce rotation speed.
    if (lastDir != turnRight)
    {
      variableUpperPWM -= 5;
      updateRPIDThresholds(minPWM, variableUpperPWM);
    }

    updateRotateDecider(targetAngle, turnRight, oneSide);
    lastDir = turnRight;

    diff = getAngleError(targetAngle);
    sensors->logActive("Diff: " + String(diff), true, 0, 4);
  }

  stop();
  updateRPIDThresholds(minPWM, maxPWM, true);
}
/*
void Movement::goToAngle(int targetAngle, bool oneSide)
{
  double currentAngle = bno->getAngleX();

  double diff = getAngleError(targetAngle);

  while (abs(diff) > 2)
  {
    // rosBridge->readOnce();
    sensors->logActive("GotoAngle");
    if (!sensors->readMotorInit())
      return;
    bool turnRight = false;
    if (diff > 0 && diff < 180 || diff < -180)
    {
      turnRight = true;
    }
    else
    {
      turnRight = false;
    }

    updateRotateDecider(targetAngle, turnRight, oneSide);

    diff = getAngleError(targetAngle);
    sensors->logActive("Diff: " + String(diff), true, 0, 3);
  }

  stop();
}*/

double Movement::map2(double value, double fromLow, double fromHigh, double toLow, double toHigh)
{
  // First, calculate the range of the input value
  double fromRange = fromHigh - fromLow;
  // Next, calculate the range of the output value
  double toRange = toHigh - toLow;
  // Then, calculate the scaled value
  double scaledValue = (value - fromLow) * toRange / fromRange + toLow;
  // Finally, return the scaled value
  return scaledValue;
}

void Movement::updateRotatePID(int targetAngle, bool right, bool oneSide)
{
  // Serial.println("new updateRotatePID");
  double current_angle = bno->getAngleX();
  if (right)
  {
    girarDerecha();

    // motor[FRONT_LEFT].motorRotateDerPID(targetAngle, current_angle);
    // motor[BACK_LEFT].motorRotateDerPID(targetAngle, current_angle);

    // Obtain pwm error
    double pwm = motor[FRONT_LEFT].motorRotatePID(targetAngle, current_angle, true);
    double angularV = map2(pwm, 0, 255, 0, maxAngular);
    Kinematics::output rmpK = kinematics.getRPM(0, 0, angularV * -1); // Angular velocity is negative for right turn.

    motor[FRONT_LEFT].motorRotationPID(rmpK.motor1);
    motor[BACK_LEFT].motorRotationPID(rmpK.motor3);

    if (!oneSide)
    {
      // motor[FRONT_RIGHT].motorRotateDerPID(targetAngle, current_angle);
      // motor[BACK_RIGHT].motorRotateDerPID(targetAngle, current_angle);
      motor[FRONT_RIGHT].motorRotationPID(rmpK.motor2);
      motor[BACK_RIGHT].motorRotationPID(rmpK.motor4);
    }
  }
  else
  {
    girarIzquierda();
    // motor[FRONT_RIGHT].motorRotateIzqPID(targetAngle, current_angle);
    // motor[BACK_RIGHT].motorRotateIzqPID(targetAngle, current_angle);

    double pwm = motor[FRONT_RIGHT].motorRotatePID(targetAngle, current_angle, false);
    double angularV = map2(pwm, 0, 255, 0, maxAngular);
    Kinematics::output rmpK = kinematics.getRPM(0, 0, angularV);

    motor[FRONT_RIGHT].motorRotationPID(rmpK.motor2);
    motor[BACK_RIGHT].motorRotationPID(rmpK.motor4);

    if (!oneSide)
    {
      // motor[FRONT_LEFT].motorRotateIzqPID(targetAngle, current_angle);
      // motor[BACK_LEFT].motorRotateIzqPID(targetAngle, current_angle);
      motor[FRONT_LEFT].motorRotationPID(rmpK.motor1);
      motor[BACK_LEFT].motorRotationPID(rmpK.motor3);
    }

    if (CK::debugGoToAngle && !CK::kusingROS)
    {
      Serial.print("Front left pwm: ");
      Serial.println(motor[FRONT_LEFT].getPWM());
    }
  }

  // motor[FRONT_LEFT].motorStatus();
  // motor[FRONT_RIGHT].motorStatus();
  // motor[BACK_LEFT].motorStatus();
  // motor[BACK_RIGHT].motorStatus();
}

void Movement::updateRotatePWM(int targetAngle, bool right)
{

  int diff = getAngleError(targetAngle);

  if (right)
  {
    girarDerecha();

    motor[FRONT_LEFT].setPWM(diff + CK::basePwmFrontLeft, 1);
    motor[BACK_LEFT].setPWM(diff + CK::basePwmBackLeft, 1);
    motor[FRONT_RIGHT].setPWM(diff * -1 + CK::basePwmFrontRight, -1);
    motor[BACK_RIGHT].setPWM(diff * -1 + CK::basePwmBackRight, -1);
  }
  else
  {
    girarIzquierda();

    motor[FRONT_LEFT].setPWM(diff + CK::basePwmFrontLeft, -1);
    motor[BACK_LEFT].setPWM(diff + CK::basePwmBackLeft, -1);
    motor[FRONT_RIGHT].setPWM(diff * -1 + CK::basePwmFrontRight, 1);
    motor[BACK_RIGHT].setPWM(diff * -1 + CK::basePwmBackRight, 1);
  }
}

void Movement::updateRotateDecider(int targetAngle, bool right, bool oneSide)
{
  if (CK::rotatePID)
  {
    updateRotatePID(targetAngle, right, oneSide);
  }
  else
  {
    updateRotatePWM(targetAngle, right);
  }
}

// Drop kit decider

// Gets sign which refers to where should a kit be dropped
void Movement::dropDecider(int ros_sign_callback)
{
  sensors->turnRightLedOn();

  double time = millis();

  dispenser->dropNKits(ros_sign_callback);

  // Wait for 5 seconds to turn off led.
  while (((millis() - time) / 1000.0) < 5)
    delay(100);

  sensors->turnRightLedOff();
}

// Possitive dist to move to the right.
void Movement::translationX(double dist)
{
  sensors->logActive("Translation X", true, 0, 3);
  double startAngle = bno->getAngleX();

  // TODO: vary translation depending on dist arg.
  // double startAngle = factor * dist;

  // Translate in X axis.
  if (dist > 0)
  {
    double targetAngle = startAngle - 40;
    targetAngle = validAngle(targetAngle);

    goToAngle(targetAngle, false, true, 3000);
    // goToAngle(startAngle + 40, false);
    delay(100);

    pivotWheels(dirToAngle(rDirection), true);

    // goToAngle(startAngle, true); // Move only one side of wheels to cause translation.
  }
  else
  {
    double targetAngle = startAngle + 40;
    targetAngle = validAngle(targetAngle);

    goToAngle(targetAngle, false, true, 3000);
    delay(100);
    pivotWheels(dirToAngle(rDirection), false);
    // goToAngle(startAngle, true);
  }

  // Reaccomodate in Y axis.
  advanceXMetersAbs(-0.05, 1);

  stop();
}

void Movement::pivotWheels(int targetAngle, bool pivotRight)
{
  double currentAngle = bno->getAngleX();

  double diff = getAngleError(targetAngle);
  long int dt = millis();
  while (abs(diff) > 2)
  {
    if (millis() - dt > timeoutPivot)
      return;
    // rosBridge->readOnce();
    if (!sensors->readMotorInit())
      return;
    bool turnRight = false;
    if (diff > 0 && diff < 180 || diff < -180)
      turnRight = true;
    else
      turnRight = false;

    currentAngle = bno->getAngleX();

    if (turnRight)
      girarDerecha();
    else
      girarIzquierda();

    // Right not moving, thus acting as pivot
    if (pivotRight)
    {
      motor[FRONT_LEFT].motorRotateDerPID(targetAngle, currentAngle);
      // motor[BACK_LEFT].motorRotateDerPID(targetAngle, currentAngle);
      motor[BACK_LEFT].setPWM(255);

      motor[FRONT_RIGHT].motorBackward(); // Move slightly backwards to avoid being moved

      motor[FRONT_RIGHT].setPWM(50);
      motor[BACK_RIGHT].setPWM(0);
    }
    else
    {
      motor[FRONT_RIGHT].motorRotateIzqPID(targetAngle, currentAngle);
      // motor[BACK_RIGHT].motorRotateIzqPID(targetAngle, currentAngle);
      motor[BACK_RIGHT].setPWM(255);

      motor[FRONT_LEFT].motorBackward();
      motor[FRONT_LEFT].setPWM(50);
      motor[BACK_LEFT].setPWM(0);
    }

    diff = getAngleError(targetAngle);
    sensors->logActive("Diff: " + String(diff), true, 0, 3);
  }
  stop();
}

double Movement::traverseRamp(int option)
{
  // Advance first if option is set to 1
  long int start = millis();

  if (option == 1)
  {
    while (millis() - start < kAdvanceToRampTime)
    {
      sensors->logActive("Traversing ramp");
      if (!sensors->readMotorInit())
        return -2;
      updateVelocityDecider(80, CK::useBNO); // Use vlx to center in ramp
      // updateStraightPID(80, useBNO, true);
      // motor[FRONT_LEFT].setPWM(255, 1);
      // motor[BACK_LEFT].setPWM(255, 1);
      // motor[FRONT_RIGHT].setPWM(255, 1);
      // motor[BACK_RIGHT].setPWM(255, 1);
    }
  }

  double pitch = sensors->getAngleY();

  bool sign = true;

  if (pitch < maxPitch + basePitch)
  {
    sign = true; // robot is moving up.
  }
  else
  {
    sign = false; // robot is moving down
  }
  int RPMs = 0;

  if (sign)
  {
    RPMs = 40;
  }
  else
  {
    RPMs = 10;
  }

  double dt = stabilizePitch(1, RPMs, true);

  advanceXMetersAbs(0.05, 1);
  rearrangeAngle(5);

  stop();
  resetEncoders();

  return dt;
}

void Movement::testMotor()
{

  updateBasePWM(-1);
  delay(2000);

  updateBasePWM(1);
  delay(100);

  stop();
  updateAngleReference();
}

// Robot state

int Movement::dirToAngle(int rdirection)
{
  return angleDirs[rdirection];
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

void Movement::logDebug(int data)
{
  String m = "Debug: " + String(data);
  nh->loginfo(m.c_str());
}

void Movement::logDebug(double data)
{
  String m = "Debug: " + String(data);
  nh->loginfo(m.c_str());
}

void Movement::logDebug(String data, double data2)
{
  String m = "Debug: " + data + " " + String(data2);
  nh->loginfo(m.c_str());
}

void Movement::handleRightLimitSwitch()
{

  /*
  int newAngle = bno->getAngleX();
  for (int i = 0; i < 8; i++)
  {
    newAngle += 8;
    if (newAngle > 360)
      newAngle -= 360;

    updateStraightPID(-kMovementRPMs);

    delay(80);
    goToAngle(newAngle, true);
    delay(50);
  }*/

  double dist = sensors->getDistInfo(dist_front);

  if (dist > 0.1)
  {
    updateBasePWM(-1);
    delay(80);
    translationX(0.3);
  }
  else
  {
    updateBasePWM(-1);
    delay(80);
    // advanceXMetersAbs(-0.03, 1);
  }

  // updateAngleReference(newAngle);
  // goToAngle(newAngle, true);
  rearrangeAngle();
  stop();
}

void Movement::handleLeftLimitSwitch()
{ /*
  int newAngle = bno->getAngleX();
  for (int i = 0; i < 8; i++)
  {
    newAngle -= 8;
    if (newAngle < 0)
      newAngle += 360;

    updateStraightPID(-kMovementRPMs);

    delay(80);
    goToAngle(newAngle, true);
    delay(50);
  }
*/

  double dist = sensors->getDistInfo(dist_front);

  if (dist > 0.1)
  {
    updateBasePWM(-1);
    delay(80);
    translationX(-0.3);
  }
  else
  {
    advanceXMetersAbs(-0.03, 1);
  }

  rearrangeAngle();
  stop();
}

void Movement::getMotorStatus(int pos)
{
  motor[pos].motorStatus();
}

void Movement::resetMovement()
{
  firstMove = true;
  rDirection = 0;
}

double Movement::validAngle(double angle)
{
  while (angle > 360)
    angle -= 360;
  while (angle < 0)
    angle += 360;

  return angle;
}

void Movement::updateRPIDThresholds(int min, int max, bool reset)
{
  if (reset)
  {
    min = minPWM;
    variableLowerPWM = minPWM;
    max = maxPWM;
    variableUpperPWM = maxPWM;
  }
  motor[FRONT_LEFT].setRotationPidThresholds(min, max);
  motor[FRONT_RIGHT].setRotationPidThresholds(min, max);
  motor[BACK_LEFT].setRotationPidThresholds(min, max);
  motor[BACK_RIGHT].setRotationPidThresholds(min, max);
}
/*




*/