#include <Arduino.h>


//LED and sensor pin setup
int sensorPin = 33;      
int bluePin = 23;        
int greenPin = 22;      
int redPin = 19;
int maxblack[]={1855,2124,2201};
int minwhite[]={1253,1386,1461};
int color[]={0,0,0};

void setup() {
  
  Serial.begin(115200);

  //RGB LED and Photoresistor
  pinMode(sensorPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  delay(10000);
  int bavg=0;
  int ravg=0;
  int gavg=0;
  for(int i =0; i<100;i++){
    digitalWrite(redPin,HIGH);
    delay(10);
    int RVA = analogRead(sensorPin);
    digitalWrite(redPin,LOW);
    delay(10);
    digitalWrite(greenPin,HIGH);
    delay(10);
    int GVA = analogRead(sensorPin);
    digitalWrite(greenPin,LOW);
    delay(10);
    digitalWrite(bluePin,HIGH);
    delay(10);
    int BVA = analogRead(sensorPin);
    digitalWrite(bluePin,LOW);
    delay(10);
    Serial.printf("R:%d\t B:%d\t G:%d\n",RVA,BVA,GVA);
    ravg+=RVA;
    bavg+=BVA;
    gavg+=GVA;
  }
  ravg= ravg/100;
  gavg= gavg/100;
  bavg= bavg/100;

  Serial.printf("AVG: R:%d B:%d G:%d",ravg,gavg,bavg);

}

void loop() {

  //Setup RGB LED blinking
  
  //delay(100);

}

