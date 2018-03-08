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

// define some variables for the encoders
float motorEncData = 0;
float motorEncPos = 0;
float armEncData = 0;
float armEncPos = 0;
float calibArmEnc = 0; //int in [0,16384]
float armEncThetaPos = 0; //takes into account the 4x gear ration and calculates angle
float armEncThetaPosOld = 0;
float absArmEncThetaPos = 0;
float absArmEncThetaPosOld = 0; 
int armcntr = 0;

// previous enc values
float motorEncPosOld = 0;
float armEncPosOld = 0;

//Workaround for velocity
float motorOriginalPosOld = 0;
float motorOriginalPos = 0;

// enc velocities
float motorEncVel = 0;
float armEncVel = 0;

// controller
float controlEffort = 0;
float umax = 200; //q = 100
//float umax = 1500; //q = 1000
//float umax = 55; //q = 1000, with zeros
float u = 0;

//PD controller
//effort = kp*posErr+kd*velErr
float kp = 5;
float kd = 0.15;

int remap;
//Timekeeping
float prevTime = 0;

//pin definition
const int pwm = 5;  //initializing pin 5 as pwm
const int motorDirection = 4;

//software contstraints
const float thetaMin = -130;
const float thetaMax = 130;

//sketchy encoder scaling
const float enc1Scale = 3.199;

// ros stuff
ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> hardwareInterface;//ros::NodeHandle hardwareInterface;
std_msgs::Float32MultiArray encSignals;
ros::Publisher encPub("encPub", &encSignals);

//std_msgs::Float32 testData;
//ros::Publisher testPub("testPub", &testData);

//Proper definition of MOD
//Ex.mod(-3, 5) = 2
float mod (float a, float b)
{
   return (a/b-floor(a/b))*b;
}

//Calculates the closest absolute distance between two angles
float angleDistance(float theta1, float theta2)
{
  return min(mod(theta1-theta2,360), 360-mod(theta1-theta2,360));
}

//Converts arm encoder position to the most likely theta based off motor encoder readings.
float armEncToTheta(float const motorTheta, float encoderPosition)
{
  encoderPosition /= 4.0;
  float baseTheta = 0;
  for(int i = 0; i < 4; i++)
  {
    if (angleDistance((encoderPosition+90*i),motorTheta) < angleDistance(encoderPosition+baseTheta,motorTheta))
    {
      baseTheta = 90.0*i;
    }
  }
  return encoderPosition+baseTheta;
}

void callback(const std_msgs::Float32& uData)
{
//  testData.data = float(1);
//  testPub.publish(&testData);
  //shutdown motor if turning too much
  if (absArmEncThetaPos < thetaMin|| absArmEncThetaPos > thetaMax)
  {
    analogWrite(pwm,0);
    //digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  if (abs(absArmEncThetaPos - motorEncPos) > 16.0){
    //sets pwm signal to 0 if the absolute encoder and motor encoder differ by more than 10 degrees
    analogWrite(pwm,0);
  }
  //digitalWrite(LED_BUILTIN, LOW);

  // send command to motor
  float u = uData.data;
  int remap = 10;
  controlEffort = round( min( abs(u), umax ) / umax * (255 - remap)) + remap;

  //command motor
  if (u > -0.001) {
    digitalWrite(motorDirection, LOW); //LOW: ccw
  }
  else if (u < 0.001) {
    digitalWrite(motorDirection, HIGH); //HIGH: cw
  }
  else {
    analogWrite(pwm, 0);
  }
  analogWrite(pwm, controlEffort);
  

}


void pdCallback(const std_msgs::Float32 & uData)
{
  //listens to angle target between 0 and 360
  float targetAngle = uData.data;
  //float errorMag = min(mod(targetAngle-motorEncPos,360), 360-mod(targetAngle-motorEncPos,360));
  while(abs(targetAngle-motorEncPos)>10)
  {
  //naively check ccw or cw, will break at the vicinity of 0:
    if (targetAngle>motorEncPos)
    {
      digitalWrite(motorDirection, HIGH); //LOW: ccw 
    }
    else
    {
      digitalWrite(motorDirection, LOW); //HIGH: cw
    }
    controlEffort = abs(kp*(targetAngle-motorEncPos)+kd*motorEncVel);//p control
    //controlEffort = abs(kp*(targetAngle-motorEncPos));//p control
  
    controlEffort = min(controlEffort, umax);
    analogWrite(pwm, int(controlEffort));
    readEncoders();
  }
  //reached target
  analogWrite(pwm, 0);
}
// subscribe to uPub
ros::Subscriber<std_msgs::Float32> uSub("uPub", &callback);
ros::Subscriber<std_msgs::Float32> pdPos("/sea/pdPos", &pdCallback);
void setup()
{  
  //pinMode(13, OUTPUT);
  prevTime = millis();
  armcntr = 0;
  // initialize ros
  hardwareInterface.initNode();

  // subscribe
  hardwareInterface.subscribe(uSub);
  hardwareInterface.subscribe(pdPos);
  // define message and advertise
  encSignals.layout.dim[0].size = 4;
  encSignals.data = (float *)malloc(sizeof(float)*4);
  encSignals.data_length = 4;
  hardwareInterface.advertise(encPub);

  Serial.begin(115200);      // Serial com for data output
    
  
  // initialize encoders

  
  initEncoders(); //Serial.println("Encoders Initialized...");  
  
  clearEncoderCount(); //Serial.println("Encoders Cleared...");
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  //pinMode(6, OUTPUT);




  // do first read from encoders to zero them out
  motorEncData = readEncoder(1);
  motorEncPos = mod(-motorEncData * 360*enc1Scale / (64 * 26.9 * 5),360);
  motorOriginalPos = motorEncPos;
  
  delay(1);

  armEncData = readAS5147P(2);
  calibArmEnc = armEncData;
  armEncPos = mod(-(armEncData - calibArmEnc)* 360 / 16384,360);
  armEncThetaPos = armEncToTheta(motorEncPos, armEncPos);
//  // initialize interrupt
//  Timer1.initialize(dtInterrupt);  /* Unit:Microsecond */
//  Timer1.attachInterrupt(readEncoders);




}


void loop()
{
  readEncoders();

  
  hardwareInterface.spinOnce();
}

void readEncoders()
{
  
 
  // read in from encoders and assign to x
  // motor position
  motorEncData = readEncoder(1);
  //Serial.print(motorEncData); Serial.print("  ");
  //Store old value
  motorOriginalPosOld = motorOriginalPos;
  motorEncPos = motorEncData * 360*enc1Scale / (64 * 26.9 * 5);
  motorOriginalPos = -motorEncPos;
  motorEncPos = mod(-motorEncPos,360);
  //motorEncPos = mod(-(motorEncPos/196)*360,360); // SKETC H CODE 
  hardwareInterface.spinOnce();

  // arm position
  armEncData = readAS5147P(2);
  //Serial.println(armEncData);
  armEncData = mod(int(armEncData-calibArmEnc),16384);
  armEncPosOld = armEncPos;
  armEncPos = mod(-armEncData * 360 / 16384, 360);  
  
  armEncThetaPosOld = armEncThetaPos;
  //armEncThetaPos = armEncToTheta(motorEncPos, armEncPos);
  armEncThetaPos = armEncPos;
  
  
  if (((armEncThetaPos >= 0)&&(armEncThetaPos < 20))&&((armEncThetaPosOld > 340)&&(armEncThetaPosOld <= 360))){
    armcntr++;
    //Serial.print(String(armEncThetaPosOld)+ "     ");
    //Serial.println("hello");
  }
  else if(((armEncThetaPosOld >= 0)&&(armEncThetaPosOld<20))&&((armEncThetaPos<=360)&&(armEncThetaPos>340))){
    armcntr = armcntr - 1;
    //Serial.println("bye");
  }
  absArmEncThetaPosOld = absArmEncThetaPos;
  absArmEncThetaPos = (armEncThetaPos + float(armcntr*360.0))/4;
  //Serial.println(absArmEncThetaPos);
  //Serial.print("   " + String(armcntr) + "     ");
  
  //motorVelocity

  //Workaround: use original value
  //Not applicable to theta2 (arm encoder)!
  float a = 0.005;
  float b = 0.005;
  motorEncVel = (1-a)*motorEncVel+a*(motorOriginalPos-motorOriginalPosOld)/(millis() - prevTime)*1000;//deg/s
  armEncVel = (1-b)*armEncVel+b*(absArmEncThetaPos - absArmEncThetaPosOld)/(millis() - prevTime)*1000;
  prevTime = millis();
  
  hardwareInterface.spinOnce();

  Serial.print("Enc1: "); Serial.print(motorOriginalPos);Serial.print(" Enc2: "); Serial.println(absArmEncThetaPos); 
  //motor velocity
  //TODO
  //motorEncVel = (motorEncData - motorEncPosOld)/dt; /// dt;

  //arm velocity
  //TODO
  //  armEncVel = (armEncData - armEncPosOld)/dt; /// dt;

  // assign to states
  encSignals.data[0] = motorOriginalPos;
  encSignals.data[1] = absArmEncThetaPos;
  encSignals.data[2] = motorEncVel;
  encSignals.data[3] = armEncVel;

  // set encoder positions to old
  motorEncPosOld = motorEncData;
  armEncPosOld = armEncData;

  //publish values
  encPub.publish(&encSignals);


  //MOTORRRRRRRRRRR
  pinMode(pwm,OUTPUT) ;   //we have to set PWM pin as output
  pinMode(motorDirection,OUTPUT);

//  if (abs(motorEncPos)<360){
//    digitalWrite(motorDirection,LOW) ;
//    analogWrite(pwm,60) ;
//  }
//  else {
//    digitalWrite(motorDirection,HIGH) ;
  //digitalWrite(pwm,LOW) ;
//  }
}


