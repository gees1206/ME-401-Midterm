#include "IR.h"

const int ang_range=30;
const int num_samples=1000;
const int IR_PIN=34;//TODO: CHANGE
int pn=1;
int ang_const=7;
bool is_Obstical=1;
int Lturn=0;
int Rturn=0;

int kp_IR=10;
int ki_IR=0;
int kd_IR=0;


TaskHandle_t Task1;
MeanFilter<int> filter(num_samples);
void IRINIT(){
    pinMode(IR_PIN,INPUT);
    setPIDgains1(kp_IR,ki_IR,kd_IR);
    Serial.println("Creating task");

    xTaskCreatePinnedToCore(
      IRScan, /* Function to implement the task */
      "IRSCAN", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &Task1,  /* Task handle. */
      tskNO_AFFINITY); /* Core where the task should run */
    Serial.println("Task Created");
}

void IRScan( void * parameter){
    while(true){
        //delay(10);
        setSetpoint1(ang_const*pn);
        delay(100);
        for(int i =0 ; i<num_samples;i++){
            filter.AddValue(analogRead(IR_PIN));
        }
        Serial.println(filter.GetFiltered());
        if(filter.GetFiltered()<150){
            is_Obstical=true;
            if(pn<0){
                Lturn=100;
                Rturn=0;
            }
            else if(pn>0){
                Rturn=100;
                Lturn=0;
            }
        }
        else{
            is_Obstical=false;
            Lturn=0;
            Rturn=0;
        }
        pn= pn*-1;
        
        
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
