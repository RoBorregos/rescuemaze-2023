#ifndef PIDRb_h
#define PIDRb_h

#include <Arduino.h>
#include <math.h>
#include "CommonK.h"

class PIDRb
{
  friend class GeneralChecks;
  
private:
  double kp{0};
  double ki{0};
  double kd{0};

  double kp_conservative{0};
  double ki_conservative{0};
  double kd_conservative{0};

  double cons_kp{0};
  double cons_ki{0};
  double cons_kd{0};

  double agr_kp{0};
  double agr_ki{0};
  double agr_kd{0};

  double errorSum{0};
  double errorPre{0};

  double maxError{2000};
  double minOutput{30};
  double maxOutput{255};

  unsigned long timePassed;
  unsigned long sampleTime{50};


public:
  bool useConservative{true};
  // Constructors

  // Set PID with manually determined constants.
  // @param kp The constant of proportionality.
  // @param ki The integral constant.
  // @param kd The derivative constant.
  // @param out_min The minimum PWM that the motor needs to for moving.
  // @param out_max The maximum PWM supported by the motor (255).
  // @param max_error_sum The maximum error that can be accumulated by integral part.
  // @param sample_time The minimum time needed to compute speed again.
  PIDRb(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time);

  // Creates PID specifing the three constants and using default values for other variables.
  PIDRb(const double kp, const double ki, const double kd);

  PIDRb();

  // PID Methods

  // Computes speed (in PWM) of the motor using PID tunings and Encoder tics for straight movement
  // @param setpoint The target speed in rev/s.
  // @param &input The current speed in rev/s. Calculated with tics. Value set as argument unused.
  // @param &output The PWM to set as the new motor speed.
  // @param &reset_variable The tics recorded since last speed computation. This variable is set to 0.
  // @param pulses_per_rev The pulses needed for one revolution. Depends on type of motor.
  // @param count_time_samples_in_one_second Speed computations per second.
  // @param debug True to print messages.
  void computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,
                    const double count_time_samples_in_one_second, const bool debug = false);

  void compute(double error, double &output, const byte flag);

  // Computes the speed of the motor using PID tunings and encoder tics for left rotation
  // @param desired Target angle
  // @param current Current angle
  // @param &output PWM of motor.
  void computeRotateIzq(const double desired, double current, double &output);

  // Computes the speed of the motor using PID tunings and encoder tics for right rotation
  // @param desired Target angle
  // @param current Current angle
  // @param &output PWM of motor.
  void computeRotateDer(const double desired, double current, double &output);

  // Other Methods

  // Sets the tunings of the PID controller
  void setTunings(double kp, double ki, double kd);

  // Sets the tunings of the PID controller
  void setConservative(double kp, double ki, double kd);
  void setAggressive(double kp, double ki, double kd);

  void flipMode(double error);

  // Resets pid tics
  void reset();

  // Prints all PID information to serial port
  void infoPID();
};

#endif
