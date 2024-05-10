#define DEBUG 0

#define BAUDRATE 9600

#define COUNT_OF_ENCODER 4

#define COMMAND_FOR_RIGHT Serial.println("R")
#define COMMAND_FOR_LEFT Serial.println("L")
#define COMMAND_FOR_RESET Serial.println("Z")
#define COMMAND_FOR_PRESS Serial.println("P")

#define ENCODER_L_PIN 26
#define ENCODER_R_PIN 25


#define REMOTE_PIN_1 18
#define REMOTE_PIN_2 21


#define SENSITIVITY 8
#define DELAY_TIME_IN_ms 250


#include <Arduino.h>

uint8_t countR = 0;
uint8_t countL = 0;

uint8_t lastcountR = 0;
uint8_t lastcountL = 0;

uint8_t encoder_l_counter = 0;
uint8_t encoder_r_counter = 0;

bool currentState;
bool lastState;




void read_encoder() 
  { 
   currentState = digitalRead(ENCODER_L_PIN);
   if (currentState != lastState)
    {     
     if (digitalRead(ENCODER_R_PIN) != currentState) { encoder_l_counter++; encoder_r_counter =0; if(encoder_l_counter > SENSITIVITY){ countL++; encoder_l_counter=0; delay(DELAY_TIME_IN_ms);} } 
     else {encoder_r_counter ++; encoder_l_counter =0; if(encoder_r_counter > SENSITIVITY) { countR++; encoder_r_counter =0; delay(DELAY_TIME_IN_ms);} }
    }
  //  else if(!digitalRead(ENCODER_P_PIN)){ COMMAND_FOR_PRESS; while(!digitalRead(ENCODER_P_PIN)){yield();}}
   lastState = currentState; 
}

unsigned long lastActivityTime = 0;

void increment_counter(){
  if (countR > lastcountR){
    lastcountR = countR;
    lastcountL = 0;
    countL = 0;
    lastActivityTime = millis();
  }
  if (countL > lastcountL){
    lastcountL = countL;
    lastcountR = 0;
    countR = 0;
    lastActivityTime = millis();
  }
  if(countR > COUNT_OF_ENCODER){
    COMMAND_FOR_RIGHT;
    countR = 0;
  }
  if(countL > COUNT_OF_ENCODER){
    COMMAND_FOR_LEFT;
    countL = 0;
  }

  if (millis() - lastActivityTime > 5000) {
    countR = 0;
    countL = 0;
    lastcountR = 0;
    lastcountL = 0;
  }
}

void read_remote(){
  if(!digitalRead(REMOTE_PIN_1)){
    COMMAND_FOR_PRESS;
    countL = 0;
    countR = 0;
    lastcountL = 0;
    lastcountR = 0;
    delay(1000);
  }
  if(!digitalRead(REMOTE_PIN_2)){
    COMMAND_FOR_RESET;
    countL = 0;
    countR = 0;
    lastcountL = 0;
    lastcountR = 0;
    delay(1000);
  }
}

void setup(){
  Serial.begin(BAUDRATE);


  pinMode(ENCODER_L_PIN,INPUT_PULLUP);
  pinMode(ENCODER_R_PIN,INPUT_PULLUP);
  pinMode(REMOTE_PIN_1,INPUT_PULLUP);
  pinMode(REMOTE_PIN_2,INPUT_PULLUP);

}


void loop(){
  read_encoder();
  read_remote();
  increment_counter();
}