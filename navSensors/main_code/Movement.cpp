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

  dispenser.initServo();
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
  // nh->loginfo("cmdMovement called");
  // nh->spinOnce();
  if (firstMove && !CK::kusingROS)
  {
    firstMove = false;
    updateAngleReference();
    Serial.println("Updated angle reference");
  }

  switch (action)
  {
  case 0:
    // nh->loginfo("Moving forward");
    // Move forward 1 unit.
    return advanceXMeters(0.3, option);
    break;

  case 1:
    // nh->loginfo("Turning right");
    // Right turn
    rotateRobot(option, 1);
    return 1;
    break;

  case 2:
    // nh->loginfo("Moving backward");
    // Move back 1 unit
    return advanceXMeters(-0.3, option);
    break;

  case 3:
    // nh->loginfo("Turning left");
    // Left turn
    rotateRobot(option, 0);
    return 1;
    break;

  case 4:
    // nh->loginfo("Traversing ramp");
    // Traverse ramp
    return traverseRamp(option);
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
    dropDecider(option);
    return 1;
    break;

  case 8:
    // nh->loginfo("Update Angle reference");
    // Update angle reference.
    updateAngleReference();
    return 1;
    break;

  default:
    break;
  }
}

void Movement::rotateRobot(int option, int dir)
{
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

    // Align robot with back wall.
    updateVelocityDecider(-kMovementRPMs, CK::useBNO);
    delay(kMillisBackAccomodate);
    updateAngleReference();

    // Move to center of tile
    advanceXMeters(0.03, true);
  }
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
void Movement::updateStraightPID(int RPMs, bool useBNO)
{
  // nh->loginfo("updateStraightPID called");
  if (useBNO)
  {
    double errorD = getAngleError(dirToAngle(rDirection));
    double factor = 10; // Increasing the factor increases the speed correction.

    motor[FRONT_LEFT].motorSpeedPID(RPMs + (errorD * factor));
    motor[BACK_LEFT].motorSpeedPID(RPMs + (errorD * factor));
    motor[FRONT_RIGHT].motorSpeedPID(RPMs + (errorD * factor * -1));
    motor[BACK_RIGHT].motorSpeedPID(RPMs + (errorD * factor * -1));
  }
  else
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

    motor[FRONT_LEFT].motorSpeedPID(RPMs + correctionVLX);
    motor[BACK_LEFT].motorSpeedPID(RPMs + correctionVLX);
    motor[FRONT_RIGHT].motorSpeedPID(RPMs + correctionVLX * -1);
    motor[BACK_RIGHT].motorSpeedPID(RPMs + correctionVLX * -1);

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
}

bool Movement::checkColor()
{
  // nh->loginfo("checkColor called");
  sensors->toggleRightLed();
  char c = sensors->getTCSInfo();
  if (!CK::kusingROS)
  {
    Serial.print("TCS info:");
    Serial.println(c);
  }

  sensors->toggleRightLed();
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
  motor[FRONT_LEFT].setPWM(leftM + CK::basePwmFrontLeft, speed_sign);
  motor[BACK_LEFT].setPWM(leftM + CK::basePwmBackLeft, speed_sign);
  motor[FRONT_RIGHT].setPWM(rightM + CK::basePwmFrontRight, speed_sign);
  motor[BACK_RIGHT].setPWM(rightM + CK::basePwmBackRight, speed_sign);
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

double Movement::advanceXMeters(double x, int straightPidType, bool forceBackward)
{
  // nh->loginfo("AdvanceXMeters called");
  double dist = sensors->getDistInfo(dist_front);
  bool useEncoders = false;

  // Use encoders to move if distance is greater than 1.28m, as its the vlx max range.
  /*if (dist > 1.28){
    useEncoders = true;
    dist = meanDistanceTraveled();
  }*/
  double initial = dist;
  double target = dist - x;
  double lastTCS = millis();

  if (x > 0)
  {
    while (dist > target && dist > 0.03)
    {
      handleSwitches();

      updateVelocityDecider(kMovementRPMs, straightPidType);

      // Get dist reading after correcting angle.
      rearrangeAngle();

      if (outOfPitch())
      {
        double dt = stabilizePitch(straightPidType);
        if (abs(dt) > CK::kRampDt)
          return dt; // Return number to indicate robot traversed ramp. Positive dt means robot went up.
      }

      if (millis() - lastTCS > checkTCSTimer && (initial - dist) > abs(distToCheck))
      {
        lastTCS = millis();
        if (checkColor())
        {
          dist = sensors->getDistInfo(dist_front);
          stop();
          // return 0;
          return advanceXMeters(dist - initial, straightPidType, true);
        }
      }

      dist = sensors->getDistInfo(dist_front);

      if (!CK::kusingROS && CK::debugAdvanceX)
      {
        Serial.print("Distancia recorrida advanceXMeters: ");
        Serial.println(initial - dist);
      }
    }
  }
  else
  {
    // Use to check if robot is stuck.
    double changeT = millis();
    double prevDist = dist;

    // Don't check color and stable pitch. Assume negative movement only for black tiles.
    while (dist < target)
    {
      updateVelocityDecider(-kMovementRPMs, straightPidType);

      // Get dist reading after correcting angle.
      rearrangeAngle();

      dist = sensors->getDistInfo(dist_front);

      if (!CK::kusingROS && CK::debugAdvanceX)
      {
        Serial.print("Distancia recorrida advanceXMeters: ");
        Serial.println(initial - dist);
      }

      // If robot hasn't moved significantly in backStuckTimer, break.
      if (millis() - changeT > backStuckTimer)
      {
        if (abs(prevDist - dist) < 0.02)
        {
          break;
        }
        else
        {
          changeT = millis();
        }
      }

      prevDist = dist;
    }
  }

  stop();
  resetEncoders();

  if (!CK::kusingROS && CK::debugAdvanceX)
  {
    Serial.print("Distancia final recorrida advanceXMeters: ");
    Serial.println(initial - dist);
  }

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
    handleRightLimitSwitch();
    if (!CK::kusingROS)
      Serial.print("Handled right limit switch");
  }
  return;
  if (left)
  {
    handleLeftLimitSwitch();
    if (!CK::kusingROS)
      Serial.print("Handled left limit switch");
  }
}
/*


void Movement::advanceXMetersEncoders(double x, bool useAngleError)
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
        updateVelocityDecider(kMovementRPMs, angleError);
      }
      else
      {
        updateVelocityDecider(kMovementRPMs);
      }

      if (millis() - lastUpdateVLX < 5)
      {
        goToAngle(getTurnDirection(rDirection));
      }
      // Get dist reading after correcting angle.
      // Serial.println("Distancia recorrida: " + String(dist - initial));
      dist = meanDistanceTraveled();
    }
  }
  else
  {
    while (target < dist)
    {
      if (useAngleError)
      {advanceXMeters
        double angleError = getAngleError(dirToAngle(rDirection));
        updateVelocityDecider(-kMovementRPMs, angleError);
      }advanceXMetersadvadvanceXMetersanceXMeters
      else
      {
        updateVelocityDecider(-kMovementRPMs);
      }
      dist = meanDistanceTraveled();
    }
  }

  stop();
  resetEncoders();
}
*/

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

    if (currRdir > 4)
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

double Movement::stabilizePitch(int straightPidType)
{
  double start = millis();
  double pitch = sensors->getAngleY();

  bool sign = true;
  if (pitch > maxPitch)
  {
    sign = true; // robot is moving up
  }
  else
  {
    sign = false; // robot is moving down
  }
  while (outOfPitch())
  {
    if (!CK::kusingROS && CK::debugRamp)
    {
      Serial.print("Robot out of stable pitch: ");
      Serial.println(sensors->getAngleY());
    }
    updateVelocityDecider(kMovementRPMs, straightPidType);
  }

  double dt = (millis() - start) / 1000.0;
  dt = dt * (sign ? 1 : -1); // apply sign
  return dt;                 // return dt in seconds.
}

bool Movement::outOfPitch()
{
  double pitch = sensors->getAngleY();
  return pitch > maxPitch || pitch < minPitch;
}

void Movement::rearrangeAngle()
{
  double angleError = getAngleError(dirToAngle(rDirection));
  if (abs(angleError) > kErrorVlxReading)
    goToAngle(dirToAngle(rDirection));
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

  while (dist < 0.25)
  {
    updateVelocityDecider(kMovementRPMs, CK::useBNO);
    dist = sensors->getDistInfo(dist_front);
  }
}

void Movement::goToAngle(int targetAngle)
{
  if (CK::debugGoToAngle && !CK::kusingROS)
  {
    Serial.println("Inside goToAngle");
  }

  double currentAngle = bno->getAngleX();

  double diff = getAngleError(targetAngle);

  while (abs(diff) > 0.1)
  {
    bool turnRight = false;
    if (diff > 0 && diff < 180)
    {
      turnRight = true;
    }
    else
    {
      turnRight = false;
    }

    updateRotatePID(targetAngle, turnRight);

    diff = getAngleError(targetAngle);

    if (CK::debugGoToAngle && !CK::kusingROS)
    {
      Serial.print("Target angle: ");
      Serial.print(targetAngle);
      Serial.print(", Diff: ");
      Serial.print(diff);
      if (turnRight)
      {
        Serial.print(" turning right, angle: ");
        Serial.println(bno->getAngleX());
      }
      else
      {
        Serial.print(" turning left, angle: ");
        Serial.println(bno->getAngleX());
      }
    }
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
  sensors->toggleRightLed();

  double time = millis();

  dispenser.dropNKits(ros_sign_callback);

  // Wait for 5 seconds to turn off led.
  while (((millis() - time) / 1000.0) < 5)
    delay(0.1);

  sensors->toggleRightLed();
}

double Movement::traverseRamp(int option)
{
  // Advance first if option is set to 1
  long int start = millis();

  if (option == 1)
  {
    while (millis() - start < kAdvanceToRampTime)
    {
      updateVelocityDecider(kMovementRPMs, CK::useBNO);
    }
  }

  double dt = stabilizePitch(0);

  stop();
  resetEncoders();

  return dt;
}

void Movement::testMotor()
{

  // Motor *m = &motor[FRONT_RIGHT];
  Motor *m = &motor[BACK_LEFT];
  // Motor *m = &motor[BACK_RIGHT];
  //  Motor *m = &motor[FRONT_LEFT];

  while (true)
  {
    if (!CK::kusingROS)
    {
      /* Display the results in the Serial Monitor */
      Serial.print("Curr speed: ");
      Serial.print(m->getCurrentSpeed());
      Serial.print(", ");
      Serial.print("Curr target: ");
      Serial.println(m->getTargetSpeed());
    }
  }
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
  for (int i = 0; i < 5; i++)
    updateVelocityDecider(-kMovementRPMs, CK::useBNO);

  stop();
  delay(100);
  int targetAngle = bno->getAngleX() - 20;
  if (targetAngle < 0)
    targetAngle += 360;
  goToAngle(targetAngle);
  stop();
}

void Movement::handleLeftLimitSwitch()
{
  for (int i = 0; i < 5; i++)
    updateVelocityDecider(-kMovementRPMs, CK::useBNO);

  stop();
  delay(100);
  int targetAngle = bno->getAngleX() + 20;
  if (targetAngle > 360)
    targetAngle -= 360;
  goToAngle(targetAngle);
  stop();
}

/*




*/