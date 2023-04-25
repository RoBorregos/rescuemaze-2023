#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include "MotorID.h"
#include "PIDRb.h"
#include "CommonK.h"

// Describes the state of a specific motor.
enum class MotorState
{
  Backward = -1,
  Stop = 0,
  Forward = 1
};

// Class for controlling motor movement.
class Motor
{
  friend class GeneralChecks;

private:
  // Motor info

  uint8_t digitalOne;
  uint8_t digitalTwo;
  int pwmPin;
  uint8_t encoderA;
  uint8_t encoderB;
  MotorID motorID;
  MotorState currentState;

  // Velocity Data

  uint8_t pwm = 0;

  // TODO: preguntar, no deberían ser volatiles estas variables? Porque las accede el arduino usando un interrupt.
  long long int ticsCounter = 0;
  int pidTics = 0;

  double currentSpeed = 0;
  double targetSpeed = 0;

  // Motor characteristics.

  static constexpr double kPulsesPerRevolution = 490.0;
  static constexpr double kWheelDiameter = 0.07;                   // (m)
  static constexpr double kRPM = 150;                              // (rev/m)
  static constexpr double kRPS = kRPM / 60;                        // (rev/s)
  static constexpr double kDistancePerRev = M_PI * kWheelDiameter; // (m)
  static constexpr double kMaxVelocity = kRPS * kDistancePerRev;   // (m/s)
  static constexpr double kMaxPWM = 255;
  static constexpr double kPwmDeadZone = 0;
  static constexpr double kMinPwmForMovement = 0;
  double velocity_adjustment_ = 0;

  // PID Data.

  static constexpr uint8_t kPidMinOutputLimit = 30;
  static constexpr uint8_t kPidMaxOutputLimit = 255;
  static constexpr uint16_t kPidMaxErrorSum = 2000;
  static constexpr uint8_t kPidMotorTimeSample = 100; // Time before calculating speed again (ms)
  static constexpr double kOneSecondInMillis = 1000.0;
  static constexpr double kSecondsInMinute = 60;
  static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis / kPidMotorTimeSample;
  static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute * kPidCountTimeSamplesInOneSecond;

  // PID controllers for straight movement. Manual calibration

  static constexpr double kPStraight = 13; // 13; // 60
  static constexpr double kIStraight = 6;  // 6;  // 55
  static constexpr double kDStraight = 2;  // 2;  // 40

  // PID controllers for rotations (used only in Movement::cmdMovement).

  PIDRb pidRotate;
  static constexpr double kPRotate = 10; // 0.5
  static constexpr double kIRotate = 10;
  static constexpr double kDRotate = 5;

public:
  // Constructors

  PIDRb pidStraight;

  Motor(uint8_t digitalOne, uint8_t digitalTwo, int pwmPin, uint8_t encoderA, uint8_t entcoderB, MotorID motorID);
  Motor();

  // Motor Functions

  // Returns motor current state
  MotorState getCurrentState();

  // Returns motor current speed in RPM
  double getCurrentSpeed();

  // Returns motor target speed in RPM
  double getTargetSpeed();

  // Returns target RPS
  // @param velocity Target speed in meters
  double getTargetRps(double velocity);

  // Returns motor current PID tics
  int getPidTics();

  // Updates pid tics
  void deltaPidTics(int delta);

  // Initialization of motor

  void initMotor();

  void motorSetup();

  void motorStatus();

  // Encoder Methods

  // Returns encoder B pin
  uint8_t getEncoderB();

  // Returns encoder A pin
  uint8_t getEncoderA();

  // Returns motor encoder tics
  int getEncoderTics();

  // Sets encoder tics to argument
  void setEncoderTics(int tics);

  // Returns distance traveled in meters
  double getDistanceTraveled();

  // Return revolutions traveled.
  double getRevolutions();

  // Attach interrupt of encoders.
  void initEncoders();

  // Updates encoder tics
  void deltaEncoderTics(int delta);

  void setVelocityAdjustment(const double velocity_adjustment);

  // Unit Conversion

  // Returns RPMs in RPSs
  double RPM2RPS(double velocity);

  // Returns Meters in RPS
  // @param MS meters.
  double Ms2Rps(double MS);

  // Control Methods

  // Sets motor direction forward
  void motorForward();

  // Sets motor direction backward
  void motorBackward();

  // Stops motor
  void motorStop();

  // Sets motor PWM
  void setPWM(double PWM);

  // Sets the pwm according and changes wheel direction
  void setPWM(int PWM, int speed_sign);

  // Returns current PWM
  double getPWM();

  // Computes target speed using straight PID controller. Enter true as
  // second argument to print debug messages.
  void motorSpeedPID(double target_speed, bool debug = false);

  // Computes target speed without using PID controller, using PWM Kinematics
  void motorSpeedPWM(double target_speed);

  // Computes rotation using rotate PID controller (right rotation)
  void motorRotateDerPID(double target_angle, double current_angle);

  // computes rotation using rotate PID controller (left rotation)
  void motorRotateIzqPID(double target_angle, double current_angle);

  // PID METHODS

  // Sets PID controllers for straight movement
  void PIDStraightTunings(double kp, double ki, double kd);

  // Sets PID controllers for rotation movement
  void PIDRotateTunings(double kp, double ki, double kd);

  void PIDAggressiveTunings(double kp, double ki, double kd);

  void PIDConservativeTunings(double kp, double ki, double kd);
};
#endif
