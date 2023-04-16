#include "PIDRb.h"

// Constructor

PIDRb::PIDRb()
{
  timePassed = millis();
}

PIDRb::PIDRb(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time)
{
  timePassed = millis();
  setTunings(kp, ki, kd);
  sampleTime = sample_time;

  maxError = max_error_sum;
  minOutput = out_min;
  maxOutput = out_max;
}

PIDRb::PIDRb(const double kp, const double ki, const double kd)
{
  timePassed = millis();
  setTunings(kp, ki, kd);
}

// PID Methods

void PIDRb::computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,
                         const double count_time_samples_in_one_second, const bool debug)
{
  unsigned long timeDiff = millis() - timePassed;

  if (timeDiff < sampleTime)
  {
    return;
  }

  double timeDiffSeconds = timeDiff / 1000.0;

  // Commented out because the time difference may be greater than sample time.

  // reset_variable / pulses per rev -> revs / timeDiff
  // revs / timeDiff * 1000/timeDiff -> revs / s
  input = (reset_variable / pulses_per_rev) * (1000.0 / timeDiff);

  reset_variable = 0; // Reset encoder tics

  const double error = setpoint - input; // Get error in terms of rev / s

  flipMode(error);

  errorSum += error * timeDiffSeconds;

  const double derivative = (error - errorPre) / timeDiffSeconds;

  output = error * kp + errorSum * ki + derivative * kd;

  errorPre = error;

  // Reduce variables to appropiate magnitudes.
  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));

  timePassed = millis();

  if (debug && !CK::kusingROS)
  {
    Serial.println("Time diff: " + String(timeDiff));
    Serial.println("Input: " + String(input));
    Serial.println("Error: " + String(error));
    Serial.println("ErrorPre: " + String(errorPre));
    Serial.println("Derivative: " + String(derivative));
    Serial.println("ErrorSum: " + String(errorSum));
    Serial.println("Output: " + String(output));
  }
}

void PIDRb::computeRotateIzq(const double desired, double current, double &output)
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

void PIDRb::computeRotateDer(const double desired, double current, double &output)
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

void PIDRb::setTunings(double kp, double ki, double kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PIDRb::setConservative(double kp, double ki, double kd)
{
  this->cons_kp = kp;
  this->cons_ki = ki;
  this->cons_kd = kd;
}

void PIDRb::compute(const double error, double &output, const byte flag)
{
  if (millis() - timePassed < sampleTime)
  {
    return;
  }

  if (errorPre * error <= 0)
  {
    errorPre = 0;
    errorSum = 0;
  }
  if (flag == 0)
  {
    if (abs(error) <= 2)
    {
      errorPre = 0;
      errorSum = 0;
    }
  }

  output = error * kp + errorSum * ki + (error - errorPre) * kd;
  errorPre = error;
  errorSum += error;

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));

  timePassed = millis();
}

void PIDRb::setAggressive(double kp, double ki, double kd)
{
  this->agr_kp = kp;
  this->agr_ki = ki;
  this->agr_kd = kd;
}

void PIDRb::flipMode(double error)
{
  if (error < 0.2)
  {
    setTunings(cons_kp, cons_ki, cons_kd);
  }
  else
  {
    setTunings(agr_kp, agr_ki, agr_kd);
  }
}

void PIDRb::reset()
{
  errorSum = 0;
  errorPre = 0;
}

void PIDRb::infoPID()
{
  // if (!CK::kusingROS){
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
  // }
}