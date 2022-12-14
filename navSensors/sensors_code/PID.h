#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include <math.h>

class PID{
  private:
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double errorSum = 0;
    double errorPre = 0;

    double maxError;
    double minOutput;
    double maxOutput;
    
    unsigned long timePassed;
    unsigned long sampleTime;
    
  public:
    // Constructors

    PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time);
    PID();

    // PID Methods

    // Computes speed of the motor using PID tunnigs and Encoder tics for straight movement
    void computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,const double count_time_samples_in_one_second);

    // Computes the speed of the motor using PID tunnigs and encoder tics for left rotation
    void computeRotateIzq(const double desired, double current, double &output);

    // Computes the speed of the motor using PID tunnigs and encoder tics for right rotation
    void computeRotateDer(const double desired, double current, double &output);

    // Other Methods
    
    // Sets the tunnings of the PID controller
    void setTunnings(double kp, double ki, double kd);
    
    // Resets pid tics
    void reset();

    // Prints all PID information to serial port
    void infoPID();
};

#endif
