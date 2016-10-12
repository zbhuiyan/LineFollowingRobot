#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Variables for PID control
double pTerm, iTerm, dTerm;
double pGain = 2, iGain = 0, dGain = 0;
double iState = 0, dState = 0;
const double IMAX = 5, IMIN = -5;
double pid = 0;

// Initialize the motorshield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create the motor objects
Adafruit_DCMotor *left = AFMS.getMotor(1);
Adafruit_DCMotor *right = AFMS.getMotor(2);

// vRef used in subtractor circuit
const double REF = 2.80;

// pins
int reflectancePin = A0;
double err = 0;

// debouncing
unsigned long counter = 0;
const unsigned long SAMPLING_RATE = 100;

// speed calculation
const uint8_t BASE_SPEED = 50;
uint8_t leftSpeed = 1, rightSpeed = 1;
const int MAX_SCALING = 5;

void setup() {
  Serial.begin(9600);
  
  pinMode(reflectancePin, INPUT);
  
  AFMS.begin();
  
  left->run(FORWARD);
  right->run(FORWARD);
  
  left->setSpeed(BASE_SPEED);
  right->setSpeed(BASE_SPEED);
}

void loop() {
  if(Serial.available()) {
    iGain = (double) Serial.readStringUntil(',').toFloat();
    pGain = (double) Serial.readStringUntil(',').toFloat();
    dGain = (double) Serial.readStringUntil(',').toFloat();
  }
  
  if(millis() - counter >= SAMPLING_RATE) {
    err = analogRead(reflectancePin) - REF;
    //pid = updatePID(err);
    scaleSpeeds();
    left->setSpeed(BASE_SPEED * leftSpeed);
    right->setSpeed(BASE_SPEED * rightSpeed);
    Serial.println(leftSpeed);
    Serial.println(rightSpeed);
  }
}

// Calculate the three parts of PID
double updatePID(double pos) {
 calcProportional(pos);
 calcIntegral(pos);
 calcDerivative(pos);
 return iTerm + pTerm + dTerm;
}

// calculating the proportional
void calcProportional(double error) {
 pTerm = pGain * error; 
}

// calculating the integral
void calcIntegral(double error) {
  iState += error;
  
  if(iState > IMAX) {
   iState = IMAX; 
  }
  
  if(iState < IMIN) {
   iState = IMIN; 
  }
  
  iTerm = iState * iGain;
}

// calculating the derivative
void calcDerivative(double pos) {
 dTerm = dGain * (dState - pos);
 dState = pos;
}

// scaling
void scaleSpeeds() {
 if(pid > 0) {
   leftSpeed = (int) (pid/20); 
   rightSpeed = 1;
 } else if(pid < 0) {
   rightSpeed = (int) (pid*-1/20);
   leftSpeed = 1;
 } else {
   leftSpeed = 1;
   rightSpeed = 1;
 } 
 
 if(leftSpeed > MAX_SCALING) {
  leftSpeed = MAX_SCALING; 
 }
 
 if(rightSpeed > MAX_SCALING) {
  rightSpeed = MAX_SCALING;
 }
}
