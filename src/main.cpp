#include <Arduino.h>
//#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"
#include "MedianFilterLib.h"

/* Communications and positional */
int myID = 10;
int b_x = 0, b_y = 0;
int error_x, error_y;
int error_d, error_theta;

// Kp1 = 0.5, Kp2 = 4.5
double Kp1 = 0.5, Kp2 = 5.0;
int omega_1, omega_2;

/* DC motor */
double kp = 60, ki = 15, kd =1.5;
/* IR Sensor */ 
int irSensorPin = 34;
int ir_points[] = {-45,-30,-15,0,15,30,45};
int ir_map[]={0,0,0,0,0,0,0};
int IRSize=30;
int irLen=7;
MedianFilter<int> filterBoi(IRSize);
//Obstacle detected variable:
int obstacle = 0;
int irThresh=20;
int maxIndex=-1;

/* limit switch */
int limitBackPin = 36;
/* LED and sensor pin setup */
int lightSensorPin = 32;
int bluePin = 23, greenPin = 22, redPin = 19;
int maxblack[] = {2336,2382,2480};
int minwhite[] = {871,844,855};
int color[] = {0,0,0};

/* Servo and driving */
int servoUP = 120; int servoDW = 80;

/* Shooting y-pos, oponent goal y-pose, own defensive y-pos, Color (1 red, 2 blue) */
//int team [4] = {2100, 2350, 300, 1}; //Team blue 
int team [4] = {300, 10, 2100, 2}; //Team red
int midfield_x = 1225;

int state = 1; //Default state is drive to ball

TaskHandle_t Task0;
TaskHandle_t Task1;

/**
 * Gets distance to a point
 *
 * @param error_x error in x position to a point
 * @param error_y error in y position to a point
 * @return Returns the distance error to a point
 */
int getErrorD(int error_x, int error_y) {
  int distance = sqrt((error_x * error_x) + (error_y * error_y));
  return distance;
}

/**
 * Gets angle from robot x axis to a point
 *
 * @param pose Most up-to date robot position.
 * @param error_x error in x position to a point
 * @param error_y error in y position to a point
 * @return Returns the angle error to a point
 */
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

  int rot = (rotate_only == true) ? 0 : 1;

  int error_x = d_x - pose.x;
  int error_y = d_y - pose.y;
  int error_d = getErrorD(error_x, error_y);
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
  double closestdistance = 9999;

  for(int i = 0; i < numBalzz; i++) {
    balldistance = getErrorD(pose.x-balzz[i].x, pose.y-balzz[i].y);
    // balldistance = sqrt((pose.x-balzz[i].x)*(pose.x-balzz[i].x) + (pose.y-balzz[i].y)*(pose.y-balzz[i].y));
    if(balldistance < closestdistance) {
      nearestball = i;
      closestdistance = balldistance; 
    }
  }
  return nearestball;
}

// /**
//  * Get IR sensor reading.
//  * @return returns mean reading across 50 values
//  */
// int getIR_Distance() {
//   int sum = 0;
//   for (int i = 0; i < 50; i++){
//     sum = sum + analogRead(irSensorPin);
//   }
//   int distance = sum / 50;
//   Serial.printf("\nDistance: %d", distance);
  
//   return distance;
// }

/**
 * Check if there is an obstacle and avoid.
 * @return none
 */
void getIR(void* pvParameters) {
  
  for (;;) {
    for(int i=0; i<irLen;i++){
      setSetpoint1(ir_points[i]*7);
      Serial.print("Setpoint:");
      Serial.println(ir_points[i]);
      while(abs(getError1())>5.0) {
        delay(200);
      }
      for(int j=0;j<IRSize;j++){
        filterBoi.AddValue(analogRead(irSensorPin));
      }
      ir_map[i] = 50.25*exp(-9E-4*(filterBoi.GetFiltered()));
      //calcAvoi();

    }
    for(int i = irLen-1 ; i >= 0; i--){
      setSetpoint1(ir_points[i]*7);
      Serial.print("Setpoint:");
      Serial.println(ir_points[i]);
      while(abs(getError1()) > 5.0) { 
        delay(200);
      }
      for(int j=0;j<IRSize;j++) {
        filterBoi.AddValue(analogRead(irSensorPin));
      }
      ir_map[i]=50.25*exp(-9E-4*(filterBoi.GetFiltered()));
    }
    Serial.printf("\nIr_map: ");
    for (int i = irLen; i < irLen ; i++) {
      Serial.printf(",%d ", ir_map[i]);
      if(ir_map[i] < irThresh) {
        obstacle = 1;
      }
    }
    Serial.println();
    
  }
}

void PIDcontroler(void* pvParameters) {
  
  for (;;) {
    RobotPose pose = getRobotPose(myID);
    //Serial.printf("\nCurrent Position: %d, %d", pose.x, pose.y);
    BallPosition balzz[20];
    int numBalzz = getBallPositions(balzz);
    
    Kp2 = 4.5;
    int prevState = state;

    //Serial.printf("Obstacle: %d", obstacle);
    if(obstacle != 1) {
      if (pose.valid == true && state != 2 && numBalzz >= 1) { 
        state = 1; // Shooting
      }
      else if (!(numBalzz >= 1) && state != 2 ) { 
        Serial.println("No more balls");
        state = 3; // Go defend
      }
    }
    else {
      state = 5;
    }
    if (pose.valid == false) { 
      Serial.println("Pose not valid");
      state = 0; // Do nothing (Stop)
    }

    analogRead(limitBackPin) > 10 ? state = 4: state =state;

    switch(state) {

      /* Do nothing (Stop) */
      case 0: 
        servo3.writeMicroseconds(1500);
        servo4.writeMicroseconds(1500);
        state = prevState;
        break;

      /* Capture the ball */
      case 1: 
        b_x = balzz[getNearestBall(pose, balzz, numBalzz)].x;
        b_y = balzz[getNearestBall(pose, balzz, numBalzz)].y;
        error_x = b_x - pose.x;
        error_y = b_y - pose.y;

        if((getErrorD(error_x, error_y) <= 210) && (abs(getErrorTheta(pose, error_x, error_y)) < 8)) { 
          Serial.println("Capturing the ball");
          servo3.writeMicroseconds(1425); //Go forward
          servo4.writeMicroseconds(1575);
          servo2.write(servoDW); 
          state = 2;
        }
        else { 
          //Serial.printf("\nDriving to ball (%d,%d)", b_x, b_y);
          driveToPoint(pose, b_x, b_y, false);
          servo2.write(servoUP); 
        }
        break;

      /* Shoot the ball */
      case 2: 
        Serial.println("Shooting the ball");

        if(getErrorD(midfield_x - pose.x, team[0] - pose.y) < 150) { 
          Kp2 = 2;
          driveToPoint(pose, midfield_x, team[1], true); //Point towards the goal

          if (abs(getErrorTheta(pose, midfield_x - pose.x, team[1] - pose.y)) < 5) {
            
            //Check color of zone were in 3 times, if good shoot, if not shoot anyway after 3 tries
            for(int i = 0; i < 3; i++) {
              digitalWrite(redPin,HIGH);
              delay(10); 
              color[0] = analogRead(lightSensorPin);
              digitalWrite(redPin,LOW);

              digitalWrite(greenPin,HIGH);
              delay(10); 
              color[1] = analogRead(lightSensorPin);
              digitalWrite(greenPin,LOW);
              
              digitalWrite(bluePin,HIGH);
              delay(10);
              color[2] = analogRead(lightSensorPin);
              digitalWrite(bluePin,LOW);

              if((color[0] > 1500 && color[1] < 1300 && team[3] == 2) || (color[0] < 1100 && color[1] > 1600 && team[3] == 1)) {
                i = 4;
              }
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
        servo2.write(servoDW);

        if(getErrorD(midfield_x + 75 - pose.x, team[2] - pose.y) < 100) {
          Serial.println("Turning parallel to goal");
          driveToPoint(pose, 10, team[2], true);
        }
        else {
          Serial.println("Driving to defensive position");
          driveToPoint(pose, (midfield_x + 75), team[2], false);
        }
        break;
      
      case 4: //Back switch avoidance
        servo3.writeMicroseconds(1300); //Go forward
        servo4.writeMicroseconds(1700); 
        delay(1000);  
        servo3.writeMicroseconds(1600); //Turn left
        servo4.writeMicroseconds(1600); 
        delay(500);
        break;

      case 5:
        // printf("obstacle");
        for(int i=0;i<irLen;i++){
          if(ir_map[i]<irThresh){
            if(maxIndex!=-1){
              if(ir_map[i]<ir_map[maxIndex]){
                maxIndex=i;
              }
            }
                else{
                  maxIndex=i;
                }
          }
        }
        Serial.printf("%d","%d","%d","%d","%d","%d",ir_map[0],ir_map[1],ir_map[2],ir_map[3],ir_map[4],ir_map[5]);
        if(maxIndex > 2){
        Serial.println("Avoiding obstacle");
        servo3.writeMicroseconds(1700); //Go backwards
        servo4.writeMicroseconds(1300); 
        delay(1000);  
        servo3.writeMicroseconds(1400); //Turn right?
        servo4.writeMicroseconds(1400); 
        delay(500);
        servo3.writeMicroseconds(1300); //Go forward
        servo4.writeMicroseconds(1700); 
        delay(1000); 
        }

        if(maxIndex <=2){

          //Serial.println("Avoiding obstacle");
          servo3.writeMicroseconds(1700); //Go backwards
          servo4.writeMicroseconds(1300); 
          delay(1000);  
          servo3.writeMicroseconds(1600); //Turn left?
          servo4.writeMicroseconds(1600); 
          delay(500);
          servo3.writeMicroseconds(1300); //Go forward
          servo4.writeMicroseconds(1700); 
          delay(1000); 
        }
        obstacle = 0;
    }
    delay(10);
    
  }
}

void setup() {
  Serial.begin(115200);
  /* Comment out when testing without comms */
  
  
  setupCommunications();
  
  
  /* Limit switches */
  pinMode(limitBackPin, INPUT);
  /* Vision: IR sensor and DC motor */
  setupDCMotors();
  setPIDgains1(kp, ki, kd);
  pinMode(irSensorPin, INPUT);
  setSetpoint1(0);
  /* RGB LED and Photoresistor */
  pinMode(lightSensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  /* Servos */
  setupServos();
  servo2.attach(SERVO2_PIN, 500, 2400);   //Arm servo
  servo3.attach(SERVO3_PIN, 1300, 1700);  //Right driving servo
  servo4.attach(SERVO4_PIN, 1300, 1700);  //Left driving servo
  /* Servo 0 pos and velocity */
  servo2.write(servoUP);
  servo3.writeMicroseconds(1500); //Servos 0 velocity
  servo4.writeMicroseconds(1500);

    //create a task that executes the Task0code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(PIDcontroler, "Task1", 10000, NULL, 1, &Task1, 1);
    //create a task that executes the Task0code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(getIR, "Task0", 10000, NULL, 1, &Task0, 0);
}

void loop() {

  //Serial.println("Loop");
  delay(10);
  
}

// DC motors
// D_print("SETPOINT:"); D_print(getSetpoint1()); D_print("   POS:"); D_print(getPosition1()); D_print("    ERR:"); D_print(getError1());
// D_print("    OUTPUT:"); D_println(getOutput1());