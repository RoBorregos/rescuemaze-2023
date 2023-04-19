/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Count the number of encoder pulses per revolution.  
 */
 

// Modified to print Encoder pulses of 4 motors instead of 1.
// Use script to calculate tics per rotation of individual motors.

// If advanceXMeters is not working properly, use this script to check if a specific motor
// has more or less tics per revolution than the default / general.

// Encoder output to Arduino Interrupt pin. Tracks the pulse count.

#define ENC_IN_RIGHT_F 2
#define ENC_IN_LEFT_F 3
#define ENC_IN_LEFT_B 18
#define ENC_IN_RIGHT_B 19
 
// Keep track of the number of right wheel pulses
long front_right_pulse_count = 0;
long front_left_pulse_count = 0;
long back_right_pulse_count = 0;
long back_left_pulse_count = 0;

void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder

  pinMode(ENC_IN_RIGHT_F , INPUT);
  pinMode(ENC_IN_LEFT_F , INPUT);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_F), right_front_wheel_pulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_F), left_front_wheel_pulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_B), left_back_wheel_pulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_B), right_back_wheel_pulseA, RISING);
   
}
 
void loop() {
  
    // Serial.print(" Pulses: ");
    // Serial.print("Fr: ");
    // Serial.print(front_right_pulse_count);
    // Serial.print(", Fl: ");
    // Serial.print(front_left_pulse_count);
    // Serial.print(", Br: ");
    // Serial.print(back_right_pulse_count);
    // Serial.print(", Bl: ");
    // Serial.println(back_left_pulse_count);  
}
 
// Increment the number of pulses by 1

void right_front_wheel_pulseA() {
  front_right_pulse_count++;
}
void left_front_wheel_pulseA() {
  front_left_pulse_count++;
}

void left_back_wheel_pulseA() {
  back_left_pulse_count++;
}

void right_back_wheel_pulseA() {
  back_right_pulse_count++;
}
