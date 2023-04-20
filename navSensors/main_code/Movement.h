#ifndef Movement_h
#define Movement_h

#include <Arduino.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include "Motor.h"
#include "Kinematics.h"
#include "Dispenser.h"
#include "BNO.h"
#include "Sensors.h"
#include "CommonK.h"

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

#define vlx_right 1
#define vlx_left 2
#define vlx_front 0
#define vlx_back 3

class Sensors;

class Movement
{
  // Give GeneralChecks full access.
  friend class GeneralChecks;

private:
  // ROS node.
  ros::NodeHandle *nh;
  // Servo dispenser
  Dispenser dispenser;
  // BNO sensors
  BNO *bno;
  // Sensors
  Sensors *sensors;

  bool firstMove = true; // Used to update angles after the first movement.

  // Straight PID with VLX.
  unsigned int lastUpdateVLX = millis();  
  double correctionVLX = 0.0;
  unsigned int kVlxErrorTimer = 250; // Wait x ms before next error calculation.

  int rDirection = 0; // 0 is north, 1 is east, 2 is south, and 3 is west.

  // Servo
  static constexpr uint8_t kServoPin = 0; // TODO: check pin

  // Motor.
  static constexpr int kMotorCount = 4;

  static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {35, 34};
  static constexpr uint8_t kAnalogPinFrontLeftMotor = 10;
  static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 49}; // A,B

  static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {32, 33};
  static constexpr uint8_t kAnalogPinBackLeftMotor = 12;
  static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {18, 47};

  static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {36, 37};
  static constexpr uint8_t kAnalogPinFrontRightMotor = 11;
  static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {3, 48};

  static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {31, 30};
  static constexpr uint8_t kAnalogPinBackRightMotor = 13;
  static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {19, 46};

  // Velocity maximum.
  static constexpr double kFrWheelsDist = 0.145;
  static constexpr double kLrWheelsDist = 0.18;
  static constexpr double kWheelDiameter = 0.07;
  static constexpr double kRPM = 240;
  static constexpr double kRPS = kRPM / 60;
  static constexpr double kMaxVelocity = kRPS * (M_PI * kWheelDiameter);

  static constexpr double kLinearXMaxVelocity = kMaxVelocity;
  static constexpr double kLinearYMaxVelocity = kMaxVelocity;
  static constexpr double kAngularZMaxVelocity = min(kMaxVelocity / kFrWheelsDist, kMaxVelocity / kLrWheelsDist);
  static constexpr uint8_t kPwmBits = 8;

  // PID
  static constexpr bool kUsingPID = true;

  // Control constants
  static constexpr double kErrorVlxReading = 1; // Error to consider a reading as valid, in degrees.
  static constexpr double minPitch = -10.0;
  static constexpr double maxPitch = 10.0;
  static constexpr double checkTCSTimer = 50; // Time to check TCS in ms.
  static constexpr double kDistanceWall = 0.08; // Distance to consider detection as wall.
  static constexpr int kMillisBackAccomodate = 700;
  static constexpr int kAdvanceToRampTime = 1000; // Time to advance to ramp in traverseRamp();
  static constexpr double distToCheck = 0.05;
  static constexpr double backStuckTimer = 100000;

  int leftM = 0;
  int rightM = 0;

  // Kinematics.
  Kinematics kinematics;

  // Cmd movement constants
  // TODO: Check maximum rpms of motors in field, and in function of battery available.
  static constexpr int kMovementRPMs = 40; // Value reduced from 100.
  static constexpr int kMaxAngle = 360;
  static constexpr uint16_t kInterAngle = 180;
  static constexpr int kMinAngle = 0.0;

  int angleDirs[4] = {0, 90, 180, 270};

public:
  // Motor Array.
  Motor motor[4]; // Public to allow access in encoder.ino.

  // Constructors

  // Using ROS and BNO with arduino
  Movement(ros::NodeHandle *nh, BNO *bno, Sensors *sensors, bool individualConstants = false);

  // Using ROS, with external use of BNO.
  Movement(ros::NodeHandle *nh, Sensors *sensors, bool individualConstants = false);

  // Using only Arduino.
  Movement(BNO *bno, Sensors *sensors, bool individualConstants = false);

  // Using only Arduino without bno
  Movement(Sensors *sensors, bool individualConstants = false);

  // Only motors
  Movement(bool individualConstants = false);

  // Initialize objects in common of constructors.
  void initMovement(bool individualConstants = false);

  void handleSwitches();
  void handleRightLimitSwitch();
  void handleLeftLimitSwitch();

  void advanceUntilCentered();

  // Initialization

  // Initializes motors, servo and kinematics.
  void initRobot();

  // Sets the values of the PID for each motor.
  void setIndividualPID();

  // Sets pins for all motors.
  void setMotors();

  // Encoder Methods
  // @return The motor's mean distance traveled.
  double meanDistanceTraveled();

  // Resets all encoders counts
  void resetEncoders();

  // Modifies pointer with encoder counts for all motors
  void getEncoderCounts(int *delta_encoder_counts);

  // Prints all encoder tics in serial monitor.
  void encoderTics();

  // Movement Methods
  // Comand Velocity, using kinematics.
  void cmdVelocity(const double linear_x, const double linear_y, const double angular_z, const bool debug = false);

  // Command movement function. Used as a clean and concise interface for robot movement.
  // The function uses the most of the movement methods implemented in this class. See the parameters'
  // definitions below the function's signature.
  // @param action A specific movement.
  // @param option Specify which option to use for specific actions.s
  // @return The status code of the movement.
  double cmdMovement(const int action, const int option = 0);

  /* Meaning of #actions, options and return values of cmdMovement:

  Action  Description                     Options                           Returns
  0       Move forward 1 unit (30 cms)    1 (use deg for error) 0 use vlx   1 -> successful, 0 -> move aborted, other -> Ramp
  3       Left turn (-90 deg)             1 to reaccomodate with back wall  1 -> successful, 0 -> move aborted
  1       Right turn (90 deg)             1 to reaccomodate with back wall  1 -> successful, 0 -> move aborted
  2       Move backward 1 unit (30 cms)   1 (use deg for error) 0 use vlx   1 -> successful, 0 -> move aborted
  5       Rearrange in current tile       None / ignored                    1 -> successful, 0 -> move aborted
  4       Traverse ramp                   None / ignored                    Estimated length of ramp.
  7       Drop n Kits.                    # of kits. Use sign for direction 1 -> successful, 0 -> move aborted
  8       Update angle reference          TODO, would help to reduce error given by physical field.
  */

  // Rotation methods

  // Sets the direction of motors for right rotation.
  void girarDerecha();

  // Sets the direction of motors for left rotation.
  void girarIzquierda();

  // Rotates the robot to the specified angle.
  void goToAngle(int targetAngle);

  // Updates individual motor speed using the angle error and PID.
  void updateRotatePID(int RPMs, bool right);

  // Returns the robot's expected angle given its rdirection.
  int dirToAngle(int rdirection);

  // Sets the current angle as the expected angle for given rDirection.
  // Updates all angles accordingly.
  void updateAngleReference();

  void printAngleReference();

  // A positive error means that the robot must rotate to the right. Negative error -> left.
  double getAngleError(double expectedAngle);

  // Returns new rdirection given turn sign.
  // @param turn 1 means right turn, 0 for left turn.
  int getTurnDirection(int turn);

  // Linear movement methods.

  // Calls straight PID method for all motors. Updates pwm of motors to approach target RPMs.
  // Use vlx or BNO for correction depending on flag.
  // @param RPMs The target speed in RPMs.
  // @param useBNO True to use BNO, false to use VLX.
  void updateStraightPID(int RPMs, bool useBNO);

  // Calls straight PID method for all motors. Updates pwm of motors to approach target RPMs.
  // @param RPMs The target speed in RPMs.
  void updateStraightPID(int RPMs);

  // Updates pwm given to each motor, choose between corrections using bno or vlx.
  void updateVelPwm(int RPMs, bool useBNO);

  // Update pwm given to each motor, choose between corrections to pwm or
  // get closer to RPMs via PID. Use flag from CommonK.h
  void updateVelocityDecider(int RPMs, bool useBNO=true);

  // Calls straight PID method for all motors, each with its specific target RMPs.
  // @param rpm Kinematic object with target rpms per wheel.
  void updatePIDKinematics(Kinematics::output rpm);

  // Moves the robot forward the specified distance.
  // @param x Distance in meters

  // TODO: Adjust distance precision.
  // Advance the specified distance using vlx. The second argument specified which method
  // should be used for straight PID.
  // @param x the distance to travel.
  // @param straigthPidType 1 for using BNO to move straight. 0 to use vlx.
  // @param forceBackwards Param used internally for robot to move back if it detects a black tile.
  double advanceXMeters(double x, int straightPidType, bool forceBackwards = false);

  // Rotate the robot in the specified dir. 0 for left and 1 for right.
  // Option specifies if the robot should move back to realign to wall.
  // if option is 1, the robot aligns with back wall.
  void rotateRobot(int option, int dir);

  // TODO: Test method.
  void advanceSlow(bool direction);

  // Traverses a ramp. If option is 1, the robot will move dt ms forward and then call stabilizePitch.
  double traverseRamp(int option);

  // Advance straight until the pitch is stable.
  // @param straightPidType 1 for using BNO to move straight. 0 to use vlx.
  double stabilizePitch(int straightPidType);

  // Return true if robot is not straight in pitch axis.
  bool outOfPitch();

  // Check if the color corresponds to the color black.
  bool checkColor();

  // If error in yaw is greater than kErrorVlxReading, then use BNO to correct.
  void rearrangeAngle();

  // TODO: Correct logic for this method.
  double getDistanceToCenter();

  // Stop all motors
  void stop();

  // Other Methods
  // Gets sign which refers to where should a kit be dropped
  void dropDecider(int ros_sign_callback);

  // For specific tests on specific motors.
  void testMotor();

  void logDebug(int data);

  void logDebug(double data);

  void logDebug(String data, double data2);
};

#endif
