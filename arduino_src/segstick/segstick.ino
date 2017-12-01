#define LPWM 9          // left motor PWM
#define RPWM 10         // right motor PWM
#define LDIR 11         // left motor direction
#define RDIR 12         // right motor direction

#define A_PIN 0         // accelerometer analog input
#define G_PIN 4         // gyro analog input
#define S_PIN 6         // steering analog input

#define A_ZERO 341      // approx. 1.5[V] * 1024[LSB/V]
#define G_ZERO 253      // approx. 1.23[V] * 1024[LSB/V]
#define S_ZERO 766      // approx. 2.5[V] * 1024[LSB/V]

#define A_GAIN 0.932    // [deg/LSB]
#define G_GAIN 1.466    // [deg/s/LSB]
#define S_GAIN 0.25     // [LSB/LSB] (AAAHHHHHHH WHAT?)

#define DT 0.02         // [s/loop] loop period
#define A 0.962         // complementary filter constant

#define KP 0.5          // proportional controller gain [LSB/deg/loop]
#define KD 0.5          // derivative controller gain [LSB/deg/loop]

float angle = 0.0;      // [deg]
float rate = 0.0;       // [deg/s]
float output = 0.0;     // [LSB] (100% voltage to motor is 255LSB)

void setup()
{
  // Make sure all motor controller pins start low.
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, LOW);
  digitalWrite(LDIR, LOW);
  digitalWrite(RDIR, LOW);
  
  // Set all motor control pins to outputs.
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(13, OUTPUT);
  
  // switch to 15.625kHz PWM
  TCCR1B &= ~0x07;
  TCCR1B |= 0x01;   // no prescale
  TCCR1A &= ~0x03;  // 9-bit PWM
  TCCR1A |= 0x02;
  
  Serial.begin(9600);
}

void loop()
{
  signed int accel_raw = 0;
  signed int gyro_raw = 0;
  signed int output_left = 0;
  signed int output_right = 0;
  signed int steer_raw = 0;
  
  // Loop speed test.
  digitalWrite(13, HIGH);
  
  // Read in the raw accelerometer, gyro, and steering signals.
  // Offset for zero angle/rate.
  accel_raw = (signed int) analogRead(A_PIN) - A_ZERO;
  gyro_raw = G_ZERO - (signed int) analogRead(G_PIN);
  steer_raw = (signed int) analogRead(S_PIN) - S_ZERO;
  
  // Scale the gyro to [deg/s].
  rate = (float) gyro_raw * G_GAIN;
  
  // Complementarty filter.
  angle = A * (angle + rate * DT) + (1 - A) * (float) accel_raw * A_GAIN;
  
  // PD controller.
  output += angle * KP + rate * KD;
  
  // Clip as float (to prevent wind-up).
  if(output < -255.0) { output = -255.0; } 
  if(output > 255.0) { output = 255.0; }
  
  // Add/subtract steering and integerize.
  output_left = (signed int) (output + (float) steer_raw * S_GAIN );
  output_right = (signed int) (output - (float) steer_raw * S_GAIN);
  
  // Clip as integer.
  if(output_left < -255) { output_left = -255; }
  if(output_left > 255) { output_left = 255; }
  if(output_right < -255) { output_right = -255; }
  if(output_right > 255) { output_right = 255; }
  
  // Choose directions and set PWM outputs.
  if(output_left >= 0)
  {
    digitalWrite(LDIR, HIGH);
    analogWrite(LPWM, output_left);
  }
  else
  {
    digitalWrite(LDIR, LOW);
    analogWrite(LPWM, -output_left);
  }
  if(output_right >= 0)
  {
    digitalWrite(RDIR, HIGH);
    analogWrite(RPWM, output_right);
  }
  else
  {
    digitalWrite(RDIR, LOW);
    analogWrite(RPWM, -output_right);
  }
  
  // Loop speed test.
  digitalWrite(13, LOW);
 
  // Debug.
  Serial.println(output_right);
 
  // Delay for consistent loop rate.
  delay(20);
} 

