#include <Arduino.h>

// #include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"

/* Communications and positional */
int myID = 10;
int ballNum;
int b_x = 0; int b_y = 0;
int error_x; int error_y;
int error_d; int error_theta;

double Kp1 = 0.5; // Kd1 = 0.5
double Kp2 = 4.5; // Kd2 = 4.0
int omega_1; int omega_2;

/* IR sensor stuff */
double kp = 21; double ki = 151; double kd = 0.263;
double setpoint = 0; int pos = 0;
int irSensor1Pin = 34;
extern double output1;

/* limit switch stuff */
int limit1 = 36; // left
int limit2 = 39; // right

/* LED and sensor pin setup */
int sensorPin = 32;
int bluePin = 23;
int greenPin = 22;
int redPin = 19;
int maxblack[] = {2336,2382,2480};
int minwhite[] = {871,844,855};
int color[] = {0,0,0};

/* Servo and driving */
int servoUP = 135; int servoDW = 85;

/* Shooting y-pos, oponent goal y-pose, own defensive y-pos, Color (1 red, 2 blue) */
//int team [4] = {2100, 2350, 300, 1}; //Team blue 
int team [4] = {300, 10, 2100, 2}; //Team red
int midfield_x = 1225;

int state = 1; //Default state is drive to ball


int getErrorD(RobotPose pose, int error_x, int error_y) {
  int distance = sqrt((error_x * error_x) + (error_y * error_y));
  return distance;
}

int getErrorTheta(RobotPose pose, int error_x, int error_y) {
  int theta = ((1000*atan2(error_y, error_x)) - pose.theta) * (180 / (PI*1000));

  if (theta < -180){
    theta = theta + 360;
  }
  else if (theta > 180){
    theta = theta - 360;
  }
  return theta;
}

/**
 * Drives or rotates the robot to a point.
 *
 * @param pose Most up-to date robot position.
 * @param d_x X coordinate of the point we're trying to go/point to
 * @param d_y Y coordinate of the point we're trying to go/point to
 * @param rotate_only If true, robot will only point towards the point
 * @return Returns the index of the ball in balzz, that's closest to the robot.
 */
void driveToPoint(RobotPose pose, int d_x, int d_y, bool rotate_only) {

  int rot = 1;
  if (rotate_only == true) {
    rot = 0;
  }

  int error_x = d_x - pose.x;
  int error_y = d_y - pose.y;
  int error_d = getErrorD(pose, error_x, error_y);
  int error_theta = getErrorTheta(pose, error_x, error_y);

  omega_1 = 0.5*(-Kp1*error_d*rot - Kp2*error_theta);
  omega_2 = 0.5*(-Kp1*error_d*rot + Kp2*error_theta);

  servo3.writeMicroseconds(omega_1 + 1500);
  servo4.writeMicroseconds(-omega_2 + 1500);
}

/**
 * Get position of nearest block.
 *
 * @param pose Most up-to date robot position.
 * @param balzz Int array containing position of all balls on field.
 * @param numBalzz Number of balls on the field.
 * @return Returns the index of the ball in balzz, that's closest to the robot.
 */
int getNearestBall(RobotPose pose, BallPosition balzz[20], int numBalzz) {
  int nearestball = 0;
  double balldistance = 0;
  int closest = 0;
  double closestdistance = 999;

  for(int i = 0; i < numBalzz; i++) {
    balldistance = sqrt((pose.x-balzz[i].x)*(pose.x-balzz[i].x) + (pose.y-balzz[i].y)*(pose.y-balzz[i].y));
    if(balldistance < closestdistance) {
      nearestball = i;
      closestdistance = balldistance; 
    }
  }
  return nearestball;
}

void setup() {
  Serial.begin(115200);

  /* Comment out when testing without comms */
  setupCommunications();

  /* Limit switches */
  pinMode(limit1, INPUT);
  pinMode(limit2, INPUT);

  /* Vision: IR sensor and DC motor */
  setupDCMotors();
  setPIDgains1(kp, ki, kd);
  pinMode(irSensor1Pin, INPUT);

  /* RGB LED and Photoresistor */
  pinMode(sensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  /* Servos */
  setupServos();
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);   //Servo 2 is shooter
  servo3.attach(SERVO3_PIN, 1300, 1700);  //rigt
  servo4.attach(SERVO4_PIN, 1300, 1700);  //ligt

  servo2.write(servoUP);
  servo3.writeMicroseconds(1500); //Servos 0 velocity
  servo4.writeMicroseconds(1500);
}

void loop() {
  int prevState = state;

  RobotPose pose = getRobotPose(myID);
  BallPosition balzz[20];
  int numBalzz = getBallPositions(balzz);
  // Serial.printf("\n Number of balls: %d", numBalzz);
  
  if (pose.valid == true && state != 2 && numBalzz >= 1) { 
    int nearestball = getNearestBall(pose, balzz, numBalzz);
    b_x = balzz[nearestball].x;
    b_y = balzz[nearestball].y;
    int error_x = b_x - pose.x;
    int error_y = b_y - pose.y;
    // Serial.printf("\n Ballpos: %d, %d", b_x, b_y);

    state = 2; // Shooting
  }

  else if (pose.valid == false) { 
    Serial.println("Pose not valid");
    state = 0; // Do nothing (Stop)
  }

  else if (!(numBalzz >= 1) && state != 2) { 
    Serial.println("No more balls");
    state = 3; // Go defend
  }
  
  int left_switch =  analogRead(limit1);
  if(left_switch > 10) {
    state = 4;
  }

  /*
      CHECK OBSTACLE HERE!!
  */

  
  /**
   * This is the main state machine. It determines which state the robot is in.
   * We need to keep track of variable called 'state'!!! and change it when appropriate.
   * i.e. Change it from 
   **/
  switch(state) {

    /* Do nothing (Stop) */
    case 0: 
      servo3.writeMicroseconds(1500);
      servo4.writeMicroseconds(1500);
      state = prevState;
      break;

    /* Capture the ball */
    case 1: 
      error_x = b_x - pose.x;
      error_y = b_y - pose.y;
      error_d = getErrorD(pose, error_x, error_y);
      error_theta = getErrorTheta(pose, error_x, error_y);

      if((error_d <= 210) && (abs(error_theta) < 10)) { 
        Serial.println("Capturing the ball");
        servo3.writeMicroseconds(1425); //Go forward
        servo4.writeMicroseconds(1575);
        servo2.write(servoDW); 
        state = 2;
      }

      else { 
        Serial.printf("\nDriving to ball (%d,%d)",b_x,b_y);
        driveToPoint(pose, b_x, b_y, false);
        servo2.write(servoUP); 
      }
      break;

    /* Shoot the ball */
    case 2: 
      Serial.println("Shooting the ball");

      servo2.write(servoDW); 
      error_x = midfield_x - pose.x;
      error_y = team[0] - pose.y;
      error_d = getErrorD(pose, error_x, error_y);

      // We will need to implement the color sensor here
      // if(error_d < 666 && (color is blue/red)){
      //   shoot the ball;}
      if(error_d < 150) { 
        
        error_x = midfield_x - pose.x;
        error_y = team[1] - pose.y;
        error_theta = getErrorTheta(pose, error_x, error_y);
        Kp2 = 2;
        driveToPoint(pose, midfield_x, team[1], true); //Point towards the goal
        Kp2 = 4.5;

        if(abs(error_theta) < 5) {
          //Austin here, adding the color stuff. Wanting to try to find the color 3 times then shoot anyways if it isn't right.
          for(int i = 0; i < 3; i++) {
            //Flashing through all 3 colors and reading photoresistor values.
            digitalWrite(redPin,HIGH);
            delay(10); // wait for the photresistor value to settle
            color[0] = analogRead(sensorPin); // read the photoresistor value
            digitalWrite(redPin,LOW);

            digitalWrite(greenPin,HIGH);
            delay(10); // wait for the photresistor value to settle
            color[1] = analogRead(sensorPin); // read the photoresistor value
            digitalWrite(greenPin,LOW);
            
            digitalWrite(bluePin,HIGH);
            delay(10); // wait for the photresistor value to settle
            color[2] = analogRead(sensorPin); // read the photoresistor value
            digitalWrite(bluePin,LOW);

            //Assuming we're on blue paper, red team
            if(color[0] > 1500 && color[1] < 1300 && team[3] == 2) {
              servo2.write(servoUP); 
              servo3.writeMicroseconds(1300); //Go forward
              servo4.writeMicroseconds(1700);
              delay(1000);
              servo3.writeMicroseconds(1700); //Go backwards
              servo4.writeMicroseconds(1300);
              delay(1000);
              servo3.writeMicroseconds(1600); //Turn back 
              servo4.writeMicroseconds(1600);
              delay(1500);
              state = 1;
              break;
            }
            //and if we're on red paper, blue team
            if(color[0] < 1100 && color[1] > 1600 && team[3] == 1){
              servo2.write(servoUP); 
              servo3.writeMicroseconds(1300); //Go forward
              servo4.writeMicroseconds(1700);
              delay(1000);
              servo3.writeMicroseconds(1700); //Go backwards
              servo4.writeMicroseconds(1300);
              delay(1000);
              servo3.writeMicroseconds(1600); //Turn back 
              servo4.writeMicroseconds(1600);
              delay(1500);
              state = 1;
              break;
            }
            //Need to clear color afterwards
            color[0] = 0;
            color[1] = 0;
            color[2] = 0;
          } 
          servo2.write(servoUP); 
          servo3.writeMicroseconds(1300); //Go forward
          servo4.writeMicroseconds(1700);
          delay(1000);
          servo3.writeMicroseconds(1700); //Go backwards
          servo4.writeMicroseconds(1300);
          delay(1000);
          servo3.writeMicroseconds(1600); //Turn back 
          servo4.writeMicroseconds(1600);
          delay(1500);
          state = 1;
        }
      }
      else {
        driveToPoint(pose, midfield_x, team[0], false);
      }
      break;

    /* Defend */
    case 3: 
      //Serial.printf("\nCurrent Position: %d, %d", pose.x, pose.y);
      //delay(100);

      servo2.write(servoDW);
      error_x = midfield_x + 75 - pose.x;
      error_y = team[2] - pose.y;
      error_d = getErrorD(pose, error_x, error_y);

      if(error_d < 100) {
        // Serial.println("Turning parallel to goal");
        Serial.printf("\tDriving to: %d, %d", 10, team[2]);
        driveToPoint(pose, 10, team[2], true);
      }

      else {
        // Serial.println("Driving to defensive position");
        Serial.printf("\tTurning to: %d, %d", (midfield_x + 75), team[2]);
        driveToPoint(pose, (midfield_x + 75), team[2], false);
      }
      break;
    
    case 4:
      servo3.writeMicroseconds(1300); //Go forward
      servo4.writeMicroseconds(1700);
      delay(1000);  
      servo3.writeMicroseconds(1600); //Turn left?
      servo4.writeMicroseconds(1600);
      delay(500);
      break;
  }
}


// for(int i = 0; i < numBalzz; i++){
//   Serial.printf("c:%d\tx:%d\ty:%d\n", balzz[i].hue,balzz[i].x,balzz[i].y);
// } 

//Sensor Data
//printf(" %d IR distance\n ", analogRead(irSensor1Pin));
//printf("%d color\n", analogRead(sensorPin));

// Serial.printf("Left Switch: %d\n", analogRead(36));
// Serial.printf("Right Switch: %d\n", analogRead(39));

// DC motors
// D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(getError1());
// D_print("    OUTPUT:"); D_println(getOutput1());