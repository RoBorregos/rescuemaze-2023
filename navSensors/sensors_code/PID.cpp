#include "PID.h"

// Constructor

PID::PID(){
  timePassed = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time){
  timePassed = millis();
  setTunnings(kp, ki, kd);
  sampleTime = sample_time;

  maxError = max_error_sum;
  minOutput = out_min;
  maxOutput = out_max;
}

// PID Methods

void PID::computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev, const double count_time_samples_in_one_second) {
  
  if(millis()-timePassed < sampleTime) {
      return;
  }
  
  // TODO: preguntar, como se calcula velocidad aqui?.
  /*
  V = m/s, puls/puls-por-rev = rev, rev * dist-por-rev = metros
  m / dt = velocidad.
  */ 
  
  input = (reset_variable / pulses_per_rev) * count_time_samples_in_one_second;
  reset_variable = 0;

  const double error = setpoint - input;

  // TODO: preguntar, no se necesita tiempo para la parte derivada?
  /*  
  derivada = dx/dt, dx = error - errorPre.
  dt = tiempo transcurrido -> dt = millis - timePassed?
  En codigo, dx/dt = dx (error - errorPre).
  dt = 1? -> Se hace cada segundo? O el tiempo es constante y considerado en kd?
  */

  output = error * kp + errorSum * ki + (error - errorPre) * kd;
  
  errorPre = error;
  errorSum += error;

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));

  timePassed = millis();
}

void PID::computeRotateIzq(const double desired, double current, double &output) {
  if(millis()-timePassed < sampleTime) {
    return;
  }

  double error = 0;

  if (current < desired){
    error = 360 - desired + current;
  } else if (desired == 0){
    error = current;
  } else {
    error = current - desired;
  }

  output = error * kp + errorSum * ki + (error - errorPre) * kd;
  
  errorPre = error;
  errorSum += error;
  

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));
  timePassed = millis();
}

void PID::computeRotateDer(const double desired, double current, double &output) {
  if(millis()-timePassed < sampleTime) {
      return;
  }

  double error = 0;

  if (current > desired){
    error = desired + (360 - current);
  } else if (desired == 0){
    error = 360 - current;
  } else {
    error = desired - current;
  }

  output = error * kp + errorSum * ki + (error - errorPre) * kd;
  
  errorPre = error;
  errorSum += error;
  

  errorSum = max(maxError * -1, min(maxError, errorSum));
  output = max(minOutput, min(maxOutput, output));
  timePassed = millis();
}

// Other Methods

void PID::setTunnings(double kp, double ki, double kd){
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PID::reset(){
  errorSum = 0;
  errorPre = 0;
}

void PID::infoPID(){
  //Serial.println("PID INFORMATION");
  //Serial.print("kP = ");
  //Serial.print(kp_);
  //Serial.print("  kI = ");
  //Serial.print(ki_);
  //Serial.print("  kD = ");
  //Serial.print(kd_);
  //Serial.print("  Sample time = ");
  //Serial.print(sample_time_);
  //Serial.print("  MaxError = ");
  //Serial.print(max_error_);
  //Serial.print("  OutputMIN = ");
  //Serial.print(min_output_);
  //Serial.print("  OutputMAX = ");
  //Serial.println(max_output_);
  //Serial.println(" ");
}