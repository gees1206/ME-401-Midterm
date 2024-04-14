#include <Arduino.h>

// #include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"


//TODO
//Calibrate the Color sensor
//Finish PID tuning the IR sensor
//Calibrate the IR sensor
//Write up the servo nonsense
//Figure out the web shit processing
//Path planning, and subroutines for attacking, defending, and shooting?

// We want to avoid obstacles as we go along
// Interrupts for switches, Interrupt for ir-sensor


//tU = .111 seconds
//ku = 45
//kp = 27
//ki = 320
//kd = 5

//Communications and positional
int myID = 10;
int x; int y; int theta;
int ballNum;

int d_x; int d_y;
int error_x; int error_y;
int error_d; int prev_error_d = 0;
int error_theta; int prev_error_theta = 0;

// Kd1 = 0.5, Kd2 = 4.0; //Drives and points correctly
double Kp1 = 0.5;
double Kp2 = 4.0;
int omega_1; int omega_2;

//IR sensor stuff
double kp = 21; double ki = 151; double kd = 0.263;
double setpoint = 0; int pos = 0;
int irSensor1Pin = 34;
extern double output1;

//limit switch stuff
int limit1 = 36; // left
int limit2 = 39; // right

//LED and sensor pin setup
int sensorPin = 34;
int bluePin = 23;
int greenPin = 22;
int redPin = 19;
int maxblack[] = {3536,2895,3313};
int minwhite[] = {3264,2468,2865};
int color[] = {0,0,0};

/*
This is the state variable. It changes the state of our robot.
See the switch statements at the end of loop()
*/
int state = 1; //Default state is drive to ball

void driveToPoint(RobotPose pose, int x, int y, bool rotate_only){

  //Sets to only point the robot
  int rot = 1;
  if (rotate_only == true){
    rot = 0;
  }

  // Coordinates of the desired point
  d_x = x;
  d_y = y;
  
  // Pose of the robot
  x = pose.x;
  y = pose.y;
  theta = pose.theta;

  error_x = d_x - x;
  error_y = d_y - y;
  error_d = sqrt((error_x * error_x) + (error_y * error_y));
  error_theta = ((1000*atan2(error_y, error_x)) - theta) * (180 / (PI*1000));

  if (error_theta < -180){
    error_theta = error_theta + 360;
  }
  else if (error_theta > 180){
    error_theta = error_theta - 360;
  }

  // Drive towards closest ball position proportional controller, 
  omega_1 = 0.5*(-Kp1*error_d*rot - Kp2*error_theta);
  omega_2 = 0.5*(-Kp1*error_d*rot + Kp2*error_theta);
  // Serial.printf("Omega_1: %d, Omega_2: %d", omega_1, omega_2);

  servo3.writeMicroseconds(omega_1 + 1500);
  servo4.writeMicroseconds(-omega_2 + 1500);
}

int getNearestBall(RobotPose pose, BallPosition balzz[20], int numBalzz){
  //Calculate the distance to the closest ball position
  int nearestball = 0;
  double balldistance = 0;
  int closest = 0;
  double closestdistance = 999;
  for(int i = 0; i < numBalzz; i++) {
    balldistance = sqrt((x-balzz[i].x)*(x-balzz[i].x) + (y-balzz[i].y)*(y-balzz[i].y));
    if(balldistance < closestdistance){
      nearestball = i;
      closestdistance = balldistance; //I changed this line here -Gabe
    }
  }
  return nearestball;
}

void setup() {
  
  Serial.begin(115200);

  /*
    //Comment out when testing outside the 401 room
  */
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
  servo2.write(115);

  servo3.attach(SERVO3_PIN, 1300, 1700);  //rigt
  servo4.attach(SERVO4_PIN, 1300, 1700);  //ligt
  servo3.writeMicroseconds(1500); //Servos 0 velocity
  servo4.writeMicroseconds(1500);
  
}

void loop() {
  printf("\n State: %d", state);
  int prevState = state;
  //Serial.printf(" theta: %d , dist: %d , roboX: %d , roboY %d , BX: %d , BY %d\n", error_theta, error_d, x, y, d_x, d_y);

  //Get robot pose and ball position
  RobotPose pose = getRobotPose(myID);
  BallPosition balzz[20];
  int numBalzz = getBallPositions(balzz);
  // for(int i = 0; i < numBalzz; i++){
  //   Serial.printf("c:%d\tx:%d\ty:%d\n", balzz[i].hue,balzz[i].x,balzz[i].y);
  // } 

  if (pose.valid == true && state == 1){ //Check if pose is valid and there are balls, otherwise no point

    //Get Nearest Ball position
    int nearestball = getNearestBall(pose, balzz, numBalzz);

    //Coordinates of the closest ball
    d_x = balzz[nearestball].x;
    d_y = balzz[nearestball].y;
    // Serial.printf("\n Ballpos: %d, %d",d_x,d_y);
    state = 1;
  }
  else if (numBalzz < 1) { 
    Serial.println("No more balls");
    state = 3; //Go defend
  }
  else if (pose.valid == false) { 
    Serial.println("Pose not valid");
    state = 0; //Do nothing (Stop)
  }

  /*
    This is the main state machine. It determines which state the robot is in.
    We need to keep track of variable called 'state'!!! and change it when appropriate.
    i.e. Change it from 
  */
  
  switch(state) {
    // Do nothing (Stop)
    case 0: 
      servo3.writeMicroseconds(1500); //Servos 0 velocity
      servo4.writeMicroseconds(1500);
      delay(1000);
      servo2.write(150); //Open the gate
      state = prevState; //Go back to previous state
      break;

    // Capture the ball
    case 1: 
      //Once we're close and pointed correctly, drive forward and lower the gate
      if((error_d <= 200) && (abs(error_theta) < 12)){ 
        servo3.writeMicroseconds(1425);
        servo4.writeMicroseconds(1575);
        servo2.write(75); //Lower the gate
        state = 2;
      }
      else { // Drive to the closest ball
        driveToPoint(pose, d_x, d_y, false);
        servo2.write(135); //Open the gate
      }
      break;

    // Shoot the ball
    case 2: 
      //Go to a point infront of the goal 
      servo2.write(135);
      driveToPoint(pose, 1150, 30, false);
      error_x = 1150 - pose.x;
      error_y = 350 - pose.y;
      error_d = sqrt((error_x * error_x) + (error_y * error_y));

      /*
        We will need to implement the color sensor here
        if(error_d < 666 && (color is blue/red)){
          shoot the ball;
        }
      */
      if(error_d < 666){ //When close to the goal
        
        //Sanity check we're pointing towards the goal
        driveToPoint(pose, 1150, 30, true);
        error_x = 1150 - x;
        error_y = 30 - y;

        error_theta = ((1000*atan2(error_y, error_x)) - pose.theta) * (180 / (PI*1000));
        if (error_theta < -180){
          error_theta = error_theta + 360;
        }
        else if (error_theta > 180){
          error_theta = error_theta - 360;
        }
        
        if(error_theta > 12){
          servo2.write(135); //Open the gate
          servo3.writeMicroseconds(1300); //Go forward
          servo4.writeMicroseconds(1700);
          delay(1000);
          servo3.writeMicroseconds(1700); //Go backwards
          servo4.writeMicroseconds(1300);
          delay(1000);
          state = 1;
        }
      }
      break;

    // Defend
    case 3: 
      //Check if balls available, switch to shooting state
      if(ballNum > 0){ 
        state = 1;
        break;
      }
      servo2.write(75); // Close the gate 
      // Orient robot parallel with the goal if close
      error_x = 1150 - pose.x;
      error_y = 350 - pose.y;
      error_d = sqrt((error_x * error_x) + (error_y * error_y));

      if(error_d < 100){
        driveToPoint(pose, 10, 50, true);
      }
      // Drive to a point infront of the goal
      else {
        driveToPoint(pose, 1150, 50, false);
      }
      break;
  }
}

//Sensor Data
  //printf(" %d IR distance\n ", analogRead(irSensor1Pin));
  //printf("%d color\n", analogRead(sensorPin));

  // Serial.printf("Left Switch: %d\n", analogRead(36));
  // Serial.printf("Right Switch: %d\n", analogRead(39));

  // DC motors
  // D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(getError1());
  // D_print("    OUTPUT:"); D_println(getOutput1());