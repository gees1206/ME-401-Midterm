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
int b_x = 1085, b_y = 1540;
int t_x = 1300, t_y = 1110;
int error_x, error_y;
int error_d, error_theta;
bool die = false;

// Kp1 = 0.5, Kp2 = 4.5
double Kp1 = 0.5, Kp2 = 4.5;
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
int irThresh = 15;
int maxIndex = -1;

/* limit switch */
int limitBackPin = 36;
/* LED and sensor pin setup */
int lightSensorPin = 32;
int bluePin = 23, greenPin = 22, redPin = 19;
int maxblack[] = {2336,2382,2480};
int minwhite[] = {871,844,855};
int color[] = {0,0,0};

/* Servo and driving */
int servoUP = 120; int servoDW = 85;

int state = 2; //Default state is drive to park

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
 * Clears IR map for the next pass.
 * 
 * @return none
 */
void clearmap(){
  obstacle = 0;
  for(int i=0;i<irLen;i++){
    ir_map[i]=-1;
  }
}

/**
 * Calculate closest obstacle and flag avoidance.
 *
 * @return none (rewrites @var obstacle)
 */
void calcAvoi(){
  Serial.printf("[%d][%d][%d][%d][%d][%d][%d]\n",ir_map[0],ir_map[1],ir_map[2],ir_map[3],ir_map[4],ir_map[5],ir_map[6]);

  int irThresh=1800;
  int maxIndex=-1;

  for(int i=0;i<irLen;i++){
    if(ir_map[i]>irThresh){
      if(maxIndex!=-1){
        if(ir_map[i]>ir_map[maxIndex]){
          maxIndex=i;
        }
      } 
      else{
        maxIndex=i;
      }
    }
  }
  if(maxIndex!= -1){
    obstacle = 1;
    //do things here
  }
    //Serial.printf("LMOD: %d\tRMOD:%d\n",LMod,RMod);
  else {
    obstacle = 0;
    //undo things here
  }
}

/**
 * Check if there is an obstacle and avoid.
 * @return none
 */
void getIR(void* pvParameters) {
    double err_thresh=10.0;
    int waittime=10;
    for (;;) {
      for(int i=0; i<irLen;i++){
        setSetpoint1(ir_points[i]*7);
        //Serial.print("Setpoint:");Serial.println(ir_points[i]);
        while(abs(getError1())>err_thresh){delay(waittime);}
        for(int j=0;j<IRSize;j++){
          filterBoi.AddValue(analogRead(irSensorPin));
          delayMicroseconds(10);
        }
        ir_map[i]=filterBoi.GetFiltered();
        calcAvoi();
        if(die==true){return;}

      }
      for(int i=irLen-1; i>=0;i--){
        setSetpoint1(ir_points[i]*7);
        //Serial.print("Setpoint:");Serial.println(ir_points[i]);
        while(abs(getError1())>err_thresh){delay(waittime);}
        for(int j=0;j<IRSize;j++){
        filterBoi.AddValue(analogRead(irSensorPin));
        delayMicroseconds(10);
        }
        ir_map[i]=filterBoi.GetFiltered();
        calcAvoi();
        if(die==true){return;}

      }
    }
  }


/**
 * PID controller function.
 * The robot navigates a field with obstacles, avoiding them using
 * IR sweep and parks at a predetermined location and angle withing 
 * a couple of centimeters.
 *
 * @param pvParameters 
 * @return none
 */
void PIDcontroler(void* pvParameters) {
  
  for (;;) {
    RobotPose pose = getRobotPose(myID);
    //Serial.printf("\nCurrent Position: %d, %d", pose.x, pose.y);
    
    Kp2 = 4.5;
    int prevState = state;

    //Serial.printf("Obstacle: %d", obstacle);
    if (state != 3){
      if(obstacle != 1) {
      state = 2;
      }
      else {
        state = 5;
      }
      if (pose.valid == false) { 
        Serial.println("Pose not valid");
        state = 0; // Do nothing (Stop)
      }
    }

    analogRead(limitBackPin) > 10 ? state = 4: state =state;
    clearmap();

    switch(state) {

      /* Do nothing (Stop) */
      case 0: 
        servo3.writeMicroseconds(1500);
        servo4.writeMicroseconds(1500);
        state = prevState;
        break;

      /* Drive to point */
      case 2: 
        Serial.println("Shooting the ball");

        if(getErrorD(b_x - pose.x, b_y - pose.y) < 70) { 
          Kp2 = 2;
          driveToPoint(pose, t_x, t_y, true); //Orientation

          if (abs(getErrorTheta(pose, t_x - pose.x, t_y - pose.y)) < 5) {
            state = 3;
          }
        }
        else {
          driveToPoint(pose, b_x, b_y, false);
        }
        break;

      /* Parked */
      case 3: 
        servo3.writeMicroseconds(1500); 
        servo4.writeMicroseconds(1500); 
        servo2.write(servoDW);
        //die=true;
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
        //Serial.printf("%d","%d","%d","%d","%d","%d",ir_map[0],ir_map[1],ir_map[2],ir_map[3],ir_map[4],ir_map[5]);
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
        //obstacle = 0;
        state = prevState;
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