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
//int bluePin = ;        
int greenPin = 4;      
int redPin = 2;
int maxblack[]={3536,2895,3313};
int minwhite[]={3264,2468,2865};
int color[]={0,0,0};

void setup() {
  
  Serial.begin(115200);
  setupDCMotors();
  setupServos();
  pinMode(sensorPin, INPUT);
  //pinMode(limit1, INPUT);
  //pinMode(limit2, INPUT);

  pinMode(irSensor1Pin,INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
 // pinMode(bluePin, OUTPUT);

   setupCommunications();

  setPIDgains1(kp,ki,kd);

  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);

  servo3.attach(SERVO3_PIN, 1300, 1700);
  servo4.attach(SERVO4_PIN, 1300, 1700);

  //servo3.writeMicroseconds(1500);
  //servo4.writeMicroseconds(1500);

  //servo1.writeMicroseconds(1700);
  //servo2.writeMicroseconds(1700);
}


extern double output1;

void loop() {

  printf("%d IR distance\n", analogRead(irSensor1Pin));
  
  //servo2.writeMicroseconds(1700);
  //delay(200);
  //servo2.writeMicroseconds(1300);
  printf("%d color\n", analogRead(sensorPin));

  //printf("%d Limit1\n", digitalRead(limit1));
  //printf("%d Limit2\n", digitalRead(limit2));

  // put your main code here, to run repeatedly:
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



    
  }

  int a = analogRead(36);
  

  Serial.printf("AD36: %d\n", a);
  analogWrite(redPin,100);
  delay(100);
  analogWrite(redPin,0);

  //delay(500);
}