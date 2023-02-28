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

class Movement
{
private:
  // ROS node.
  ros::NodeHandle * nh_;
  // Servo dispenser
  Dispenser dispenser;
  // BNO sensors
  BNO *bno;
  // Sensors
  Sensors *sensors;

  // Servo
  static constexpr uint8_t kServoPin = 7;

  // Leds
  static constexpr uint8_t kDigitalPinsLEDS[2] = {41, 42};

  // Limit Switches
  static constexpr uint8_t kDigitalPinsLimitSwitch[2] = {24, 25};

  // Motor.
  static constexpr int kMotorCount = 4;

  static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {35, 34};
  static constexpr uint8_t kAnalogPinFrontLeftMotor = 10;
  static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 49}; // A,B

  static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {33, 32};
  static constexpr uint8_t kAnalogPinBackLeftMotor = 12;
  static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {19, 47};

  static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {36, 37};
  static constexpr uint8_t kAnalogPinFrontRightMotor = 11;
  static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {3, 48};

  static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {30, 31};
  static constexpr uint8_t kAnalogPinBackRightMotor = 13;
  static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {18, 46};
  int last_encoder_counts_[kMotorCount];

  // Velocity maximum.
  static constexpr double kWheelDiameter = 0.05;
  static constexpr double kRPM = 150;
  static constexpr double kRPS = kRPM / 60;
  static constexpr double kMaxVelocity = kRPS * M_PI * kWheelDiameter;
  static constexpr double kLinearXMaxVelocity = kMaxVelocity;
  static constexpr double kLinearYMaxVelocity = kMaxVelocity;
  static constexpr double kAngularZMaxVelocity = kMaxVelocity;

  // PID
  static constexpr bool kUsingPID = false;
  static constexpr double kPStraightFR = 7; // 60
  static constexpr double kIStraightFR = 3; // 55
  static constexpr double kDStraightFR = 2; // 40

  static constexpr double kPRotateFR = 5;
  static constexpr double kIRotateFR = 3;
  static constexpr double kDRotateFR = 1;

  // Kinematics.
  Kinematics kinematics_;
  int motor_max_rpm_ = 150;
  float wheel_diameter_ = 0.05; // En metros

  // Distancia llanta trasera y delantera
  float fr_wheels_dist_ = 0.13;

  // Distancia llanta izquierda y derehca
  float lr_wheels_dist_ = 0.13;
  int pwm_bits_ = 8;

  // Cmd movement constants
  static constexpr int kMovementRPMs = 60;
  static constexpr double kMaxAngle = 359.0;
  static constexpr double kMinAngle = 0.0;

public:
  // Motor Array.
  Motor motor[4]; // Public to allow access in encoder.ino.

  // Constructors

  // Using ROS
  Movement(ros::NodeHandle *nh, BNO *bno, Sensors *sensors);

  // Using arduino movement
  Movement(BNO *bno, Sensors *sensors);

  // Call using a boolean value to use individual PID constants for each motor.
  Movement(BNO *bno, Sensors *sensors, bool individualConstants);

  // Initialization

  // Initializes motors, leds, servo, limit switches, and kinematics.
  void initRobot();

  // Sets the values of the PID for each motor.
  void setIndividualPID();

  // Sets pins for all motors.
  void setMotors();

  // Initializes indicator leds.
  void initLeds();

  // Initializes limit switches.
  void initSwitches();

  // Encoder Methods
  // Returns the motors mean distance traveled.
  double meanDistanceTraveled();

  // Resets all encoders counts
  void resetEncoders();

  // Modifies pointer with encoder counts for all motors
  void getEncoderCounts(int *delta_encoder_counts);

  // Prints all encoder tics in serial monitor.
  void encoderTics();

  // Movement Methods
  // Comand Velocity, using kinematics.
  void cmdVelocity(const double linear_x, const double linear_y, const double angular_z);

  // Comand Movement, using PID and static velocity
  void cmdMovement(int movement_type);

  // Sets motors state to turn right.
  void girarDerecha();

  // Sets motors state to turn left.
  void girarIzquierda();

  // Calls straight PID method for all motors. Updates pwm of motors to approach target RPMs.
  // @param RPMs The target speed in RPMs.
  void updateStraightPID(int RPMs);

  // Moves the robot forward the specified distance.
  // @param x Distance in meters.
  void advanceXMeters(double x);

  // Decides how to turn depinding on the current and desired angles
  void turnDecider(double current_angle, double desired_angle);

  // Uses turn decider but with a delta angle
  void girarDeltaAngulo(double delta_theta);

  void stop();

  // Other Methods
  // Gets sign which refers to where should a kit be dropped
  void dropDecider(int ros_sign_callback);

  // Cheks limit switches state to then decide how to move
  void checkLimitSwitches();

  // For specific tests on specific motors.
  void testMotor();
};

#endif
