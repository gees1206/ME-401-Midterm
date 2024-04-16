#include "IR.h"

const int ang_range=0;
const int num_steps=0;
const int num_samples=0;
const int IR_PIN=34;//TODO: CHANGE
int old_time=0;
int curr_step=0;
int step_size = ang_range/num_steps;
int pn=1;
int ang_const=1;
bool is_Obstical=1;
int Lturn=0;
int Rturn=0;

TaskHandle_t Task1;
MeanFilter<int> filter(num_samples);
void IRINIT(){
    pinMode(IR_PIN,INPUT);

    xTaskCreatePinnedToCore(
      IRScan, /* Function to implement the task */
      "IRSCAN", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */
}

void IRScan( void * parameter){
    while(true){
        setSetpoint1(curr_step*step_size*ang_const);
        for(int i =0 ; i<num_samples;i++){
            filter.AddValue(analogRead(IR_PIN));
        }
        if(filter.GetFiltered()<150){
            is_Obstical=true;
            if(curr_step<1){
                Lturn=100;
                Rturn=-100;
            }
            else if(curr_step>=1){
                Rturn=100;
                Lturn=-100;
            }
        }
        else{
            is_Obstical=false;
            Lturn=0;
            Rturn=0;
        }
        curr_step++;
        if(curr_step>num_steps/2){
            curr_step=(num_steps/2)*-1;
        }
        
        
    }
}
bool isObstical(){
    return is_Obstical;
}

int getAvoidLeft(){
    return Lturn;
}

int getAvoidRight(){
    return Rturn ;
}
