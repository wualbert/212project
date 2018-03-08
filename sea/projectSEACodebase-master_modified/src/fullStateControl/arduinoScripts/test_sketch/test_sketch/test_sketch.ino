#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Float32MultiArray.h>  
#include <std_msgs/Float32.h>  

#include <SPI.h>
#include "Encoder.h"
#include "TimerOne.h"
  
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initEncoders(); //Serial.println("Encoders Initialized...");  

  clearEncoderCount(); //Serial.println("Encoders Cleared...");
  pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);

  
  // do first read from encoders to zero them out
  /*motorEncData = readEncoder(1);
  motorEncPos = mod(-motorEncData * 360*enc1Scale / (64 * 26.9 * 5),360);
  motorOriginalPos = motorEncPos;*/
  
  delay(1);

  armEncData = readAS5147P(2);
  calibArmEnc = armEncData;
  armEncPos = mod(-(armEncData - calibArmEnc)* 360 / 16384,360);
  armEncThetaPos = armEncToTheta(motorEncPos, armEncPos);
//  // initialize interrupt
//  Timer1.initialize(dtInterrupt);  /* Unit:Microsecond */
//  Timer1.attachInterrupt(readEncoders);
}

void loop() {
  // put your main code here, to run repeatedly:
  readEncoders();


}

void readEncoders(){

  //motorEncData = readEncoder(1);
  armEncData = readAS5147P(2);
  Serial.print("Motor enc Data: " + String(motorEncData));
  Serial.print(" Arm enc Data: " + String(armEncData));
  
}

