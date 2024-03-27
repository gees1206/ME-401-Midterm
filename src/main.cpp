#include <Arduino.h>

#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"

//Austin Writing his stupid ass concerns:
//I've made the rotational gain twice of the directional but I still worry directional will dominate, and 
//potentially make us just go forward.


//Successfully created a git repository


//TODO
//Calibrate the Color sensor
//Finish PID tuning the IR sensor
//Calibrate the IR sensor
//Write up the servo nonsense
//Figure out the web shit processing
//Path planning, and subroutines for attacking, defending, and shooting?

//tU = .111 seconds
//ku = 45
//kp = 27
//ki = 320
//kd = 5

//Communications and positional
int myID = 10;
int x;
int y;
int theta;
int ballNum;

int d_x;
int d_y;
int error_x;
int error_y;
int error_d;
int error_theta;
int prev_error_d = 0;
int prev_error_theta = 0;

int Kp1 = 1;
int Kd1 = 0.1;
int Kp2 = 2;
int Kd2 = 0.1;
int omega_1;
int omega_2;

//IR sensor stuff
double kp =21; double ki=151; double kd=.263; 
double setpoint = 0;
int pos = 0;
int irSensor1Pin=34;

//limit switch stuff
int limit1 = 36; // left
int limit2 = 39; // right

//LED and sensor pin setup
int sensorPin = 34;      
int bluePin = 33;        
int greenPin = 27;      
int redPin = 32;
int maxblack[]={3536,2895,3313};
int minwhite[]={3264,2468,2865};
int color[]={0,0,0};


void setup() {
  
  Serial.begin(115200);

  //Leave commented out when testing
  setupCommunications();

  //Limit switches
  pinMode(limit1, INPUT);
  pinMode(limit2, INPUT);

  //Vision: IR sensor and DC motor
  setupDCMotors();
  setPIDgains1(kp,ki,kd);
  pinMode(irSensor1Pin,INPUT);

  //RGB LED and Photoresistor
  pinMode(sensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  //Servos
  setupServos();
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);   //Servo 2 is shooter
  //servo1.writeMicroseconds(1700);
  servo2.writeMicroseconds(1700);

  servo3.attach(SERVO3_PIN, 1300, 1700);  //Movement servos
  servo4.attach(SERVO4_PIN, 1300, 1700);
  servo3.writeMicroseconds(1500); //Servos 0 velocity
  servo4.writeMicroseconds(1500);
}


extern double output1;

void loop() {

  
  Serial.begin(115200);

  //Sensor Data
  printf("%d IR distance\n", analogRead(irSensor1Pin));
  printf("%d color\n", analogRead(sensorPin));
  //printf("%d Limit1\n", digitalRead(limit1));
  //printf("%d Limit2\n", digitalRead(limit2));

  //Setup RGB LED blinking
  digitalWrite(redPin,HIGH);
  delay(50);
  digitalWrite(redPin,LOW);
  digitalWrite(greenPin,HIGH);
  delay(50);
  digitalWrite(greenPin,LOW);
  digitalWrite(bluePin,HIGH);
  delay(50);
  digitalWrite(bluePin,LOW);

  // DC motors
  D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(getError1());
  D_print("    OUTPUT:"); D_println(getOutput1());
  

  //*****************************************************************************************
  //Printing information about what we have and where
  //*****************************************************************************************
  Serial.printf("/%d Numballs\n", ballNum);
  
  
  RobotPose jimmy = getRobotPose(10);
  Serial.printf("#%d\tx:%d\ty:%d\n", jimmy.ID,jimmy.x,jimmy.y);
  BallPosition balzz[20];
  int numBalzz = getBallPositions(balzz);
  for(int i =0; i < numBalzz; i++){
    Serial.printf("c:%d\tx:%d\ty:%d\n", balzz[i].hue,balzz[i].x,balzz[i].y);
  } 
  //*****************************************************************************************

  // while (Serial.available() > 0) // check serial monitor for input
  // {
  //   // read the incoming byte:
  //   int incomingByte = Serial.read();

  //   // say what you got:
  //   if (incomingByte == 'a')
  //   {      
  //     setpoint += 300; 
  //     setSetpoint1(setpoint);
  //   }
  //   else if (incomingByte == 'z')
  //   {
  //     setpoint -= 300;
  //     setSetpoint1(setpoint);
  //   }            
  //   else if (incomingByte == 'o')
  //   {
  //     // kp +=0.5;
  //     // myPID.SetTunings(kp,ki,kd);
  //   }
  //   else if (incomingByte == 'o')
  //   {
  //     // kp -=0.1;
  //     // myPID.SetTunings(kp,ki,kd);
  //   }
  //   else if (incomingByte == 't')
  //   {
  //     pos = pos+5;
  //     servo1.write(pos);  
  //     servo2.write(pos);  
  //   }
  //   else if (incomingByte == 'g')
  //   {
  //     pos = pos-5;
  //     servo1.write(pos);  
  //     servo2.write(pos);  
  //   }
  //   else if (incomingByte == 'f')
  //   {
  //     servo3.writeMicroseconds(1700);  
  //     servo4.writeMicroseconds(1700);  
  //   }
  //   else if (incomingByte == 'b')
  //   {
  //     servo3.writeMicroseconds(1300);  
  //     servo4.writeMicroseconds(1300);  
  //   }
  //   else{
  //     break;
  //   }
  // }

  //Get robot pose and ball position*
  //*Not yet implemented
  RobotPose pose = getRobotPose(myID);
  if (pose.valid == true){
    x = pose.x;
    y = pose.y;
    theta = pose.theta;
  }
  else {
    Serial.println("Pose not valid");
  }

  //Calculate the distance to the desired ball position
  //Need to find closest ball and then update to get us towards the ball.
  //Still need to figure out the logic for the first time through the loop if necessary.
  int nearestball;
  double balldistance;
  int closest;
  double closestdistance;
  for(int i = 0; i < numBalzz; i++){
    balldistance = sqrt((x-balzz[i].x)^2 + (y-balzz[i].y)^2);
    if(balldistance < closestdistance){
      nearestball = i;
    }

  }

  d_x = balzz[nearestball].x; 
  d_y = balzz[nearestball].y;
  error_x = d_x - x;
  error_y = d_y - y;
  error_d = sqrt(error_x^2+error_y^2);
  error_theta = (atan2(error_y,error_x) - theta)*(180/(PI*1000));

  if (error_theta < -180){
    error_theta = error_theta + 360;
  }
  else if (error_theta > 180){
    error_theta = error_theta - 360;
  }
  
  //Drive towards closest ball position proportional controller, 
  omega_1 = (Kp1*error_d + Kp2*error_theta); //+ Kd1*(error_d-prev_error_d) - Kd2*(error_theta - prev_error_theta));
  omega_2 = -(Kp1*error_d + Kp2*error_theta); //+ Kd1*(error_d-prev_error_d) + Kd2*(error_theta - prev_error_theta));
  prev_error_d = error_d;
  prev_error_theta = error_theta;

  //Hypothetical maximum omegas:
  //707*gain linear, 3141*gain rotational, 

  //Mapping values based on absolute maximum error, narrowing the range is a good idea.
  servo3.writeMicroseconds(map(omega_1, 707,-707, 1300, 1700));
  servo4.writeMicroseconds(map(omega_2, 707,-707, 1300, 1700));

  // int a = analogRead(36);
  // Serial.printf("Left Switch: %d\n", a);
  // int b = analogRead(39);
  // Serial.printf("Right Switch: %d\n", b);
  
  //Now grabbing the ball:
  double rooterror = sqrt(error_x * error_x + error_y * error_y);
  //Once we're close, drive forward slowly while lowering the gate
  if(rooterror <= 15){
    servo3.writeMicroseconds((1525));
    servo4.writeMicroseconds((1525));
    servo1.writeMicroseconds(70);
  }



}


