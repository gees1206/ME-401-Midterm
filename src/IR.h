#pragma once

#include <Arduino.h>
#include "dc_motors.h"
#include "MeanFilterLib.h"

volatile int curr_ang=0;
const int ang_range=0;
const int num_steps=0;
volatile int curr_step=0;
const int num_samples=0;
const int time_inte=0;
volatile MeanFilter<int> filter(num_samples);
volatile int old_time=0;

int step_size = ang_range/num_steps;
int pn=1;

bool isObstical=false;
TaskHandle_t Task1;

void IRINIT(){
    xTaskCreatePinnedToCore(
      IRScan, /* Function to implement the task */
      "IRSCAN", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */
    };


void IRScan( void * parameter){
    while(true){

    }
};
bool isObstical(){
    return isObstical;
}

int getAvoidLeft(){
    return 0;
}

int getAvoidRight(){
    return 0 ;
}
