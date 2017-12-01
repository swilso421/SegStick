// 03/03/2014  SELF BALANCING SKATEBOARD AND SEGWAY CONTROLLER USING SPARKFUN 6DOF DIGITAL IMU
// John Dingley.
// My previous versions used analog IMU's which are becoming very hard to find.
// This has been a long time coming but finally this code works with a DIGITAL IMU.
// It is a combination of my previous self-balancer codes and aspects of the PIDDYBOT code which was for a small self-balancing robot project.
// Piddybot Program Written and Pieced together by Sean Hodgins.
// The Piddybot program itself was developed from many programs, in particular the work of Varesano.
// Piddybot site is here
// http://www.Idlehandsproject.com
// My previous pages on Instructables website on construction of self-balancing skateboards 
// are here:  http://www.instructables.com/id/Easy-build-self-balancing-skateboardrobotsegway-/
// Previous tutorial on using an analog IMU is here: http://www.instructables.com/id/Self-balancing-skateboardsegwy-project-Arduino-S/
// My website on all things self balancing:    https://sites.google.com/site/onewheeledselfbalancing/
// My latest Uni-Mig 01 one wheeled self balancer (similar software): https://sites.google.com/site/secretselfbalancingscooter/

// IMU used here:   Sparkfun  Code: SEN 10121    IMU Digital Combo Board - 6 Degrees of Freedom ITG3200/ADXL345
// See Sparkfun website:  https://www.sparkfun.com/products/10121


// LIBRARIES
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

//Download the folder "FreeSixIMU" from this website and put whole folder into your Arduino libraries folder so then the libraries listed above can be accessed by the Arduino.
//Instructions on how to download this folder and put it into your Arduino libraries folder can be found here on 
//this website (scroll down the page about half way): http://bildr.org/2012/03/stable-orientation-digital-imu-6dof-arduino/


FreeSixIMU sixDOF = FreeSixIMU();


//Set dip switches on the Sabertooth for simplified serial and 9600 Buadrate. Diagram of this on my Instructables page see above



//simplifierd serial limits for each motor
#define SABER_MOTOR1_FULL_FORWARD 127
#define SABER_MOTOR1_FULL_REVERSE 1

#define SABER_MOTOR2_FULL_FORWARD 255
#define SABER_MOTOR2_FULL_REVERSE 128

//motor level to send when issuing full stop command
#define SABER_ALL_STOP  0



                                             



//setup all variables. Terms may have strange names but this software has evolved from bits and bobs done by other segway clone builders

// code that keeps loop time at 10ms per cycle of main program loop xxxxxxxxxxxxxxx
int STD_LOOP_TIME = 9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;


//XXXXXXXXXXXXXXXXXXXXXXXXXXX USER ADJUSTABLE VARIABLES XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
float gyroscalingfactor = 1.0;  //strange constant. It is best thought of as the "balance gyro scaling factor"
//Increase the value of Start_Balance_Point to bring the initial balance point further backwards
float Start_Balance_point = 0;


float P_constant = 4.5;  //previously 4.0 
float D_constant = 0.5; //previously 0.4
float I_constant = 1.0;  //previously 2.5

float overallgaintarget = 0.6;   //previously 0.5
float overallgainstart = 0.3; //starting value before softstart
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX END OF USER ADJUSTABLE VARIABLES XXXXXXXXXXXXXX


float overallgain;

float level=0;

float Steering;
float SteerValue;
float SteerCorrect;
float steersum;
int Steer = 0;

int firstloop;


float g;
float s;


float gangleratedeg;




int cut = 100;


float gangleraterads;



int k4;
int k5;
int k6;
int k7;
int k8;



float gyroangledt;
float angle;
float anglerads;
float balance_torque;
float cur_speed;
float balancetrim;

int i;
int j;
int tipstart;

signed char Motor1percent;
signed char Motor2percent;


//digital inputs. Wire up the IMU as in my Instructable and these inputs will be valid.
int leftbuttonPin = 4;//left steer rocker switch
int rightbuttonPin = 5;//right steer rocker switch
int forwardtrimbuttonPin = 2;  //trim rocker switch
int backtrimbuttonPin = 3; //trim rocker switch other way
int deadmanbuttonPin = 9;  // deadman button , push to make switch. If you let go, motors both stop permamently


//DIGITAL OUTPUTS
//DEFINE THE LEDS we are using as alarms etc
int ledonePin = 10;
int ledtwoPin = 11;










const int AvgAngles = 3;
 float prevTargetAngle = 0;
 float targetAngle = 0;


float angles[5];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;

// time vars
int currTime = 0; 
int prevTime = 0; 





void setup() {


    //digital inputs
  pinMode(deadmanbuttonPin, INPUT);
  digitalWrite(deadmanbuttonPin, HIGH);  //enables the Arduino internal pullup. when button pressed it connects it to ground giving a value of zero when pressed
  pinMode(rightbuttonPin, INPUT);
  digitalWrite(rightbuttonPin, HIGH); 
  pinMode(leftbuttonPin, INPUT);
  digitalWrite(leftbuttonPin, HIGH); 
  pinMode(forwardtrimbuttonPin, INPUT);
  digitalWrite(forwardtrimbuttonPin, HIGH); 
  pinMode(backtrimbuttonPin, INPUT);
  digitalWrite(backtrimbuttonPin, HIGH); 
  
  
  //digital outputs
  pinMode(ledonePin, OUTPUT);
  pinMode(ledtwoPin, OUTPUT);
  
  
  
  // Serial Connection
  Serial.begin(9600);

  // IMU Connection
  Wire.begin(); 

  delay(5);
  sixDOF.init(); //Begin the IMU
  delay(5);
  
  Serial.write(SABER_ALL_STOP);   //kill motors when first switched on

}


void loop() {
  tipstart = 0;
  overallgain = 0;
  cur_speed = 0;
  level = 0;
  Steer = 0;
  balancetrim = 0;
  
  digitalWrite(ledonePin, HIGH);
  //Tilt board one end on floor. Turn it on and let go i.e. stop wobbling it about
  //as now the software will read the gyro values 200 times when there is no rotational movement to find the average zero point for each gyro.
  delay(3000);

   for (i=0; i<200; i++) {
                  sample_inputs();
                         }
                         
   
  
   digitalWrite(ledonePin, HIGH);
    digitalWrite(ledtwoPin, HIGH);

             for (i=0; i<50; i++) {
   //XXXXXXXXXXXXXXXXXXXXX TIMEKEEPER      loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
   lastLoopUsefulTime = millis()-loopStartTime;
   if (lastLoopUsefulTime < STD_LOOP_TIME) {
                                delay(STD_LOOP_TIME-lastLoopUsefulTime);
                                            }
   lastLoopTime = millis() - loopStartTime;
   loopStartTime = millis();   
   //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX 
                  sample_inputs();
                  
                             } 
      
      
      while (tipstart < 5) {  //don't know why I chose 5 but there we are
    //XXXXXXXXXXXXXXXXXXXXX TIMEKEEPER      loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
   lastLoopUsefulTime = millis()-loopStartTime;
   if (lastLoopUsefulTime < STD_LOOP_TIME) {
                                delay(STD_LOOP_TIME-lastLoopUsefulTime);
                                            }
   lastLoopTime = millis() - loopStartTime;
   loopStartTime = millis();   
   //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  
                  sample_inputs(); 
  
  
  
  
  
  
                if ((currAngle < (Start_Balance_point -1)) || (currAngle > (Start_Balance_point + 1))){
              
                                  tipstart = 0;
                                  overallgain = 0;
                                  cur_speed = 0;
                                  level = 0;
                                  Steer = 0;
                                  balancetrim = 0; 
                                                   }
              else {
                     tipstart = 5;
                     
                      digitalWrite(ledonePin, LOW);
                      digitalWrite(ledtwoPin, LOW);
                      
                     
                     
                    }     
                             
    } //end of while tipstart < 5 
  
 overallgain = overallgainstart;  //softstart value. Gain will now rise to final of 0.6 at rate of 0.005 per program loop. 
//i.e. it will go from 0.3 to 0.6 over the first 4 seconds or so after tipstart has been activated


angle = 0;
cur_speed = 0;
Steering = 512;
SteerValue = 512;
balancetrim = 0; 
  
  
  firstloop = 1;
  
  
 //end of tiltstart code. If go beyond this point then machine is active
//main balance routine, just loops forever. Machine is just trying to stay level. You "trick" it into moving by tilting one end down
//works best if keep legs stiff so you are more rigid like a broom handle is if you are balancing it vertically on end of your finger
//if you are all wobbly, the board will go crazy trying to correct your own flexibility.
//NB: This is why a segway has to have vertical handlebar otherwise ankle joint flexibility in fore-aft direction would make it oscillate wildly.
//NB: This is why the handlebar-less version of Toyota Winglet still has a vertical section you jam between your knees.
while (1) {
  
  sample_inputs();
  
  set_motor();
  
  //XXXXXXXXXXXXXXXXXXXXX loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
   lastLoopUsefulTime = millis()-loopStartTime;
   if (lastLoopUsefulTime < STD_LOOP_TIME) {
                                delay(STD_LOOP_TIME-lastLoopUsefulTime);
                                            }
   lastLoopTime = millis() - loopStartTime;
   loopStartTime = millis();   
   //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX   



  
   serialOut_timing();//updates the LED status every now and then (one means sending >50% of full power to motors.
   //Two LED's lit means sending > 75% of full power to motors and you need to slow down.
   

 
 //XXXXXXXXXXXXXXXXXXXX softstart function: board a bit squishy when you first bring it to balanced point, 
 //then ride becomes firmer over next 4 seconds as value for overallgain increases from starting value of 0.3 to 0.6(overallgaintarget) set in user adjustable variables at start.
   if (overallgain < overallgaintarget) {
       overallgain = (float)overallgain + 0.005;
                          }
   if (overallgain > overallgaintarget) {overallgain = overallgaintarget;}
 //XXXXXXXXXXXXXXX end of softstart code XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 
  
           } 
  
  
  
}

  
  
  
  
  
  
  
 void serialOut_timing(){
   static int skip=0;
   
   if (skip++==20) { //display every 2000ms (at 100Hz)
     skip = 0;


                            }
    if ((Motor1percent > 50)  || (Motor1percent < -50)){
                       digitalWrite(ledonePin, HIGH);
                            }
    if ((Motor1percent > 75) || (Motor1percent < -75)){
                       digitalWrite(ledtwoPin, HIGH);
                            }
    if ((Motor1percent <= 50) && (Motor1percent >= -50)){
                       digitalWrite(ledtwoPin, LOW);
                        digitalWrite(ledonePin, LOW);
                            
                             } 
    

     

   }   //XXXXXXXXXXXXXX end of LED status update XXXXXXXXXXXXXXXXXXXXXXXXXXX
                    
                             
  
  
  
  
  
  
  
void updateAngle() {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}



void sample_inputs()  {
  
  k4 = digitalRead(deadmanbuttonPin); //1 when not pressed and 0 when is being pressed
  k5 = digitalRead(leftbuttonPin);
  k6 = digitalRead(rightbuttonPin);
  k7 = digitalRead(forwardtrimbuttonPin);
  k8 = digitalRead(backtrimbuttonPin);
   updateAngle();
  
  
 //adjust balance trim
  if (k7 == 0){  //is 0 when it IS being pressed
    balancetrim = balancetrim - 0.005;
               }
  if (k8 == 0){
    balancetrim = balancetrim + 0.005;
               }             
  
  
  
  
  if (balancetrim < -8) balancetrim = -8; //stops you going too far with this
  if (balancetrim > 8) balancetrim = 8; //stops you going too far the other way
  


//STEERING
  if (k5 == 0) {
             
                                           SteerValue = (float)SteerValue + 0.5;
                }
                
  if (k6 == 0) {
             
                                           SteerValue = (float)SteerValue - 0.5;
                } 
                
  if ((k6 == 1) && (k5 == 1)){  SteerValue = 512;}  //if steering switch not being flicked either way then you want to go straight on
  
   
    if (SteerValue < 362) {
                    SteerValue = 362;  //limiting max rate of turning (512 is no turning)
                        } 
    if (SteerValue > 662) {
                    SteerValue = 662;   //limiting max rate of turning
                           } 
    SteerCorrect = 0;  
//XXXXXXXXXXX  End of steering code    




    if (firstloop == 1){
                   lastLoopTime = 10;
                   firstloop = 0;
                   
                   gyroangledt = 0;
                   gangleraterads = 0;
                      }
  else {
  gangleratedeg = (float) (currAngle - prevAngle) * (1/(lastLoopTime * 0.001));  //angle change rate in degrees per second
  gyroangledt = (float) (gyroscalingfactor * lastLoopTime * 0.001 * gangleratedeg);
  gangleraterads = (float) (gangleratedeg * 0.017453); // just a scaling issue from history
        }
        
  angle = (float) currAngle + balancetrim;
  
  anglerads = (float) angle * 0.017453;
  
  balance_torque = (float) (P_constant * anglerads) + (D_constant * gangleraterads);
  
  cur_speed = (float) (cur_speed + (I_constant * anglerads * 0.001 * lastLoopTime));
  
  level = (float)(balance_torque + cur_speed) * overallgain;  
  
}//end of sample inputs








void set_motor()   {
  unsigned char cSpeedVal_Motor1 = 0;
  unsigned char cSpeedVal_Motor2 = 0;
  
  level = level * 200; //changes it to a scale of about -100 to +100
  if (level < -100) {level = -100;}
  if (level > 100) {level = 100;}
  
  
  
  Steer = (float) SteerValue - SteerCorrect;  //at this point is on the 0-1023 scale 
  //SteerValue is either 512 for dead ahead or bigger/smaller if you are pressing steering switch left or right
 
  Steer = (Steer - 512) * 0.19;   //gets it down from 0-1023 (with 512 as the middle no-steer point) to -100 to +100 with 0 as the middle no-steer point on scale
  
  
  
  
//set motors using the simplified serial Sabertooth protocol (same for smaller 2 x 5 Watt Sabertooth by the way) 
                
 Motor1percent = (signed char) level + Steer;
 Motor2percent = (signed char) level - Steer;
 
 
 if (Motor1percent > 100) Motor1percent = 100;
 if (Motor1percent < -100) Motor1percent = -100;
 if (Motor2percent > 100) Motor2percent = 100;
 if (Motor2percent < -100) Motor2percent = -100;
 
 //if not pressing deadman button on hand controller - cut everything
    if (k4 == 1) { //is 1 when you ARENT pressing the deadman button
      cut = cut - 1;
      if (cut < 0){ cut = 0;}
                }
    if (k4 < 1){  //is zero when you ARE pressing deadman button
      cut = cut + 1;
      if (cut > 50){ cut = 50;}    //if cut is 100 takes 1 second off the deadman before motors actually cut
                }
    
  
   if (cut == 0) { 
    level = 0;
    Steer = 0;
    Motor1percent = 0;
    Motor2percent = 0;
  
     cSpeedVal_Motor1 = map (Motor1percent,
                         -100,
                         100,
                         SABER_MOTOR1_FULL_REVERSE,
                         SABER_MOTOR1_FULL_FORWARD);
                         
     cSpeedVal_Motor2 = map (Motor2percent,
                         -100,
                          100,
                         SABER_MOTOR2_FULL_REVERSE,
                         SABER_MOTOR2_FULL_FORWARD);
                         
 Serial.write (cSpeedVal_Motor1);     //enacts the command for zero power before we then enter the endless while loop
 Serial.write (cSpeedVal_Motor2);     //enacts the command for zero power before we then enter the endless while loop        
 

         
               while(1) {    //loops endlessly until reset
     
                              
                                delay(500);
                                 pinMode(ledonePin, HIGH);
                                 pinMode(ledtwoPin, HIGH);
                           
                                delay(500);
                                 pinMode(ledonePin, LOW);
                                 pinMode(ledtwoPin, LOW);
                                
     
     
                         } // end of while 1
                 }   //end of if cut == 0  

               
               
               
               
               
               
               
               
               
               
               
               
 //cut is not 0 so we therefore enact the specified command to the motors              
 
 cSpeedVal_Motor1 = map (Motor1percent,
                         -100,
                         100,
                         SABER_MOTOR1_FULL_REVERSE,
                         SABER_MOTOR1_FULL_FORWARD);
                         
 cSpeedVal_Motor2 = map (Motor2percent,
                         -100,
                          100,
                         SABER_MOTOR2_FULL_REVERSE,
                         SABER_MOTOR2_FULL_FORWARD);
                         
 Serial.write (cSpeedVal_Motor1);
 Serial.write (cSpeedVal_Motor2);
 
 
  
}

