#include <Arduino.h>

#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"

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
int myID = 1;
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
int Kp2 = 1;
int Kd2 = 0.1;
int omega_1;
int omega_2;

//IR sensor stuff
double kp =21; double ki=151; double kd=.263; 
double setpoint = 0;
int pos = 0;
int irSensor1Pin=34;

//limit switch stuff
int limit1 = 36;
int limit2 = 39; // left

//LED and sensor pin setup
int sensorPin = 34;      
int bluePin = 23;        
int greenPin = 22;      
int redPin = 32;
int maxblack[]={3536,2895,3313};
int minwhite[]={3264,2468,2865};
int color[]={0,0,0};

void setup() {
  
  Serial.begin(115200);

  //Leave commented out when testing
  // setupCommunications();

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
  //servo2.writeMicroseconds(1700);

  servo3.attach(SERVO3_PIN, 1300, 1700);  //Movement servos
  servo4.attach(SERVO4_PIN, 1300, 1700);
  servo3.writeMicroseconds(1500); //Servos 0 velocity
  servo4.writeMicroseconds(1500);
}


extern double output1;

void loop() {

  //Sensor Data
  printf("%d IR distance\n", analogRead(irSensor1Pin));
  printf("%d color\n", analogRead(sensorPin));
  //printf("%d Limit1\n", digitalRead(limit1));
  //printf("%d Limit2\n", digitalRead(limit2));

  //Setup RGB LED blinking
  rgbled_setup();

  // DC motors
  D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(getError1());
  D_print("    OUTPUT:"); D_println(getOutput1());


  while (Serial.available() > 0) // check serial monitor for input
  {
    // read the incoming byte:
    int incomingByte = Serial.read();

    // say what you got:
    if (incomingByte == 'a')
    {      
      setpoint += 300; 
      setSetpoint1(setpoint);
    }
    else if (incomingByte == 'z')
    {
      setpoint -= 300;
      setSetpoint1(setpoint);
    }            
    else if (incomingByte == 'o')
    {
      // kp +=0.5;
      // myPID.SetTunings(kp,ki,kd);
    }
    else if (incomingByte == 'o')
    {
      // kp -=0.1;
      // myPID.SetTunings(kp,ki,kd);
    }
    else if (incomingByte == 't')
    {
      pos = pos+5;
      servo1.write(pos);  
      servo2.write(pos);  
    }
    else if (incomingByte == 'g')
    {
      pos = pos-5;
      servo1.write(pos);  
      servo2.write(pos);  
    }
    else if (incomingByte == 'f')
    {
      servo3.writeMicroseconds(1700);  
      servo4.writeMicroseconds(1700);  
    }
    else if (incomingByte == 'b')
    {
      servo3.writeMicroseconds(1300);  
      servo4.writeMicroseconds(1300);  
    }
    else{
      break;
    }
  }

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
  d_x = 500; 
  d_y = 500;
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
  
  //Drive towards closest ball position
  omega_1 = 0.5*(Kp1*error_d - Kp2*error_theta + Kd1*(error_d-prev_error_d) - Kd2*(error_theta - prev_error_theta));
  omega_2 = 0.5*(Kp1*error_d + Kp2*error_theta + Kd1*(error_d-prev_error_d) + Kd2*(error_theta - prev_error_theta));
  prev_error_d = error_d;
  prev_error_theta = error_theta;

  servo3.writeMicroseconds(omega_1);
  servo3.writeMicroseconds(omega_2);
  

  int a = analogRead(36);
  Serial.printf("AD36: %d\n", a);

}

void rgbled_setup() //Causes the RGB LED to switch color
{
  digitalWrite(redPin,HIGH);
  delay(100);
  digitalWrite(redPin,LOW);
  digitalWrite(greenPin,HIGH);
  delay(100);
  digitalWrite(greenPin,LOW);
  digitalWrite(bluePin,HIGH);
  delay(100);
  digitalWrite(bluePin,LOW);
}