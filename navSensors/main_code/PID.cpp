#include "PID.h"

// Constructor

PID::PID()
{
  timePassed = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time)
{
  timePassed = millis();
  setTunnings(kp, ki, kd);
  sampleTime = sample_time;

  maxError = max_error_sum;
  minOutput = out_min;
  maxOutput = out_max;
}

// PID Methods

void PID::computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev, const double count_time_samples_in_one_second)
{

  unsigned long timeDiff = millis() - timePassed;

  if (timeDiff < sampleTime)
  {
    return;
  }

  // Commented out because the time difference may be greater than sample time.
  // input = (reset_variable / pulses_per_rev) * count_time_samples_in_one_second;

  // reset_variable / pulses per rev -> revs/timeDiff
  // revs/timeDiff * 1000/timeDiff -> revs / s
  input = (reset_variable / pulses_per_rev) * (1000.0 / timeDiff);

  reset_variable = 0;

  const double error = setpoint - input;

  output = error * kp + errorSum * ki + (error - errorPre) / timeDiff * kd;

  errorPre = error;
  errorSum += error;

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));

  timePassed = millis();
}

void PID::computeRotateIzq(const double desired, double current, double &output)
{
  unsigned long timeDiff = millis() - timePassed;

  if (timeDiff < sampleTime)
  {
    return;
  }

  double error = 0;

  if (current < desired)
  {
    error = 360 - desired + current;
  }
  else if (desired == 0)
  {
    error = current;
  }
  else
  {
    error = current - desired;
  }

  output = error * kp + errorSum * ki + (error - errorPre) / timeDiff * kd;

  errorPre = error;
  errorSum += error;

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));
  timePassed = millis();
}

void PID::computeRotateDer(const double desired, double current, double &output)
{
  unsigned long timeDiff = millis() - timePassed;

  if (timeDiff < sampleTime)
  {
    return;
  }

  double error = 0;

  if (current > desired)
  {
    error = desired + (360 - current);
  }
  else if (desired == 0)
  {
    error = 360 - current;
  }
  else
  {
    error = desired - current;
  }

  output = error * kp + errorSum * ki + (error - errorPre) / timeDiff * kd;

  errorPre = error;
  errorSum += error;

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));
  timePassed = millis();
}

// Other Methods

void PID::setTunnings(double kp, double ki, double kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PID::reset()
{
  errorSum = 0;
  errorPre = 0;
}

void PID::infoPID()
{
  // Serial.println("PID INFORMATION");
  // Serial.print("kP = ");
  // Serial.print(kp_);
  // Serial.print("  kI = ");
  // Serial.print(ki_);
  // Serial.print("  kD = ");
  // Serial.print(kd_);
  // Serial.print("  Sample time = ");
  // Serial.print(sample_time_);
  // Serial.print("  MaxError = ");
  // Serial.print(max_error_);
  // Serial.print("  OutputMIN = ");
  // Serial.print(min_output_);
  // Serial.print("  OutputMAX = ");
  // Serial.println(max_output_);
  // Serial.println(" ");
}