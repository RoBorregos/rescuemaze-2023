#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include "MotorID.h"
#include "PID.h"

enum class MotorState{
  Backward = -1,
  Stop = 0,
  Forward = 1
};

class Motor{
  private:
    // Motor info
    uint8_t digitalOne;
    uint8_t digitalTwo;
    int pwmPin;
    uint8_t encoderA;
    uint8_t encoderB;
    MotorID motorID;
    MotorState currentState;

    // VelocityData
    uint8_t pwm = 0;

    // TODO: preguntar, no deberían ser volatiles estas variables? Porque las accede el arduino usando un interrupt.
    long long int ticsCounter = 0;
    int pidTics = 0;
    
    double currentSpeed = 0;
    double targetSpeed = 0;

    // Motor characteristics.
    static constexpr double kPulsesPerRevolution = 451.0;
    static constexpr double kWheelDiameter = 0.05;
    static constexpr double kRPM = 150;
    static constexpr double kRPS = kRPM / 60;
    static constexpr double kMaxVelocity = kRPS * M_PI * kWheelDiameter;
    static constexpr double kMaxPWM = 255;
    static constexpr double kPwmDeadZone = 0;
    static constexpr double kMinPwmForMovement = 0;
    static constexpr double kDistancePerRev = M_PI * kWheelDiameter;

    // PID Data.
    static constexpr uint8_t kPidMinOutputLimit = 30;
    static constexpr uint8_t kPidMaxOutputLimit = 255;
    static constexpr uint16_t kPidMaxErrorSum = 2000;
    static constexpr uint8_t kPidMotorTimeSample = 100;
    static constexpr double kOneSecondInMillis = 1000.0;
    static constexpr double kSecondsInMinute = 60;
    static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kPidMotorTimeSample;
    static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute*kPidCountTimeSamplesInOneSecond;

    // PID Controllers for straight movement. Manual calibration
    PID pidStraight;
    static constexpr double kPStraight = 13; //60
    static constexpr double kIStraight = 6; //55
    static constexpr double kDStraight = 2; //40

    // PID Controllers for rotations (used only in Movement::cmdMovement).
    PID pidRotate;
    static constexpr double kPRotate = 1.25; //0.5
    static constexpr double kIRotate = 0;
    static constexpr double kDRotate = 0;

  public:
    // Constructors
    
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
    double getTargetRps(double velocity);

    // Returns motor current PID tics
    int getPidTics();

    // Updates pid tics
    void deltaPidTics(int delta);

    // Initialization of motor
    void motorSetup();

    // Encoder Methods

    // Returns encoder B pin
    uint8_t getEncoderB();

    // Returns encoder A pin
    uint8_t getEncoderA();

    // Returns motor encoder tics
    int getEncoderTics();

    // Sets encoder tics to parameter
    void setEncoderTics(int tics);

    // Returns distance traveled in meters
    double getDistanceTraveled();

    // Attach interrupt of encoders.
    void initEncoders();
    
    // Updates encoder tics
    void deltaEncoderTics(int delta);
    
    // Unit Conversion

    // Returns RPMs in RPSs
    double RPM2RPS(double velocity);
    
    // Returns MS in RPSs
    double Ms2Rps(double MS);
    
    // Returns PWM in RPMs
    double Pwm2Rpm(double pwm);

    // Control Methods

    // Sets motor direction forward
    void motorForward();

    // Sets motor direction backward
    void motorBackward();

    // Stops motor
    void motorStop();

    // Sets motor PWM
    void setPWM(double PWM);

    // Returns current PWM
    double getPWM();
    
    // Computes target speed using straight PID controller
    void motorSpeedPID(double target_speed);

    // Computes target speed without using PID controller, using PWM Kinematics
    void motorSpeedPWM(double target_speed);

    // Computes rotation using rotate PID controller (right rotation)
    void motorRotateDerPID(double target_angle, double current_angle);

    // computes rotation using rotate PID controller (left rotation)
    void motorRotateIzqPID(double target_angle, double current_angle);

    // PID METHODS
    
    // Sets PID controllers for straight movement
    void PIDStraightTunnigs(double kp, double ki, double kd);

    // Sets PID controllers for rotation movement
    void PIDRotateTunnigs(double kp, double ki, double kd);
};
#endif
