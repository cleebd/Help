#include "esp32-hal-gpio.h"
#include "Sensor.hpp"

/*Initialize all the IR sensor Data Pins*/
void IR::Init(){
  pinMode(IR_L,INPUT);
  pinMode(IR_M,INPUT);
  pinMode(IR_R,INPUT);
};
/*For the Logic
  0: White 
  1: Black*/
uint8_t IR::Tracking(){
  //M_IR on Track
  if(digitalRead(IR_M) && !(digitalRead(IR_L)) && !(digitalRead(IR_R)))
    return OnTrack;
  //R_IR on Track 
  if((!digitalRead(IR_M) && (!digitalRead(IR_L)) && (digitalRead(IR_R))) || (digitalRead(IR_M) && (!digitalRead(IR_L)) && (digitalRead(IR_R))))
    return IR_ROnTrack;  
  //L_IR on Track 
  if(!digitalRead(IR_M) && (digitalRead(IR_L)) && !(digitalRead(IR_R)) || (digitalRead(IR_M) && (digitalRead(IR_L)) && !(digitalRead(IR_R))))
    return IR_LOnTrack;
  //All on Track
  if(digitalRead(IR_M) && (digitalRead(IR_L)) && (digitalRead(IR_R)))
    return AllOnTrack;

  return OutOfTrack; 
};

void Ultrasonic::Init(){
  pinMode(Tirg, OUTPUT); // Sets the trigPin as an Output
  pinMode(Echo, INPUT); // Sets the echoPin as an Input
};

float Ultrasonic::GetDistance(){
  // Clears the trigPin
  digitalWrite(Tirg, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(Tirg, HIGH);
  delayMicroseconds(10);
  digitalWrite(Tirg, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long  duration = pulseIn(Echo, HIGH);
  
  // Calculate the distance (in m)
  float distance = (duration * SOUND_SPEED/100)/2;
  
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // Serial.println(distance/100);
  return distance;
};


