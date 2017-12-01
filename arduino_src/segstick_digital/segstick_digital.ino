#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#define LPWM 9          // left motor PWM
#define RPWM 10         // right motor PWM
#define LDIR 11         // left motor direction
#define RDIR 12         // right motor direction

#define SDA_PIN 4
#define SCL_PIN 5

//#define A_PIN 0         // accelerometer analog input
//#define G_PIN 4         // gyro analog input
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

signed int output_left = 0;
signed int output_right = 0;
signed int steer_raw = 0;

const int AvgAngles = 3;
float prevTargetAngle = 0;
float targetAngle = 0;

float angles[5];
float values[6];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;

FreeSixIMU sixDOF = FreeSixIMU();

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
  pinMode(13, OUTPUT); //What is this for?!?
  
  // switch to 15.625kHz PWM
  TCCR1B &= ~0x07;
  TCCR1B |= 0x01;   // no prescale
  TCCR1A &= ~0x03;  // 9-bit PWM
  TCCR1A |= 0x02;
  
  Serial.begin(9600);

  // For the IMU
  Wire.begin();

  delay(5);
  sixDOF.init(); //Begin the IMU
  delay(5);
  
}

void loop()
{
  // Loop speed test.
  digitalWrite(13, HIGH);
  
  // Read in the raw accelerometer, gyro, and steering singals.
  // Offset for zero angle/rate.
  //accel_raw = (signed int) analogRead(A_PIN) - A_ZERO;
  //gyro_raw = G_ZERO - (signed int) analogRead(G_PIN);
  steer_raw = (signed int) analogRead(S_PIN) - S_ZERO;

  updateCurrAngle();
  sixDOF.getValues(values);
  
  // Scale the gyro to [deg/s].
  rate = values[4];
  
  // Complementarty filter.
  angle = A * (angle + rate * DT) + (1 - A) * currAngle;
  
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

void updateCurrAngle() {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}

