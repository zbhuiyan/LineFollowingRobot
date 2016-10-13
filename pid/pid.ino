#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Initialize the motorshield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create the motor objects
Adafruit_DCMotor *left = AFMS.getMotor(3);
Adafruit_DCMotor *right = AFMS.getMotor(4);

// vRef used in subtractor circuit
const double REF = 2.88;

// pins
int reflectancePin = A2;
double err = 0;

// debouncing
unsigned long counter = 0;
const unsigned long SAMPLING_RATE = 20;

// speed calculation
int baseSpeed = 30;
int leftWheelError = 10;
uint8_t leftSpeed = 1, rightSpeed = 1;
int scaling = 12;

void setup() {
  Serial.begin(9600);
  
  pinMode(reflectancePin, INPUT);
  
  AFMS.begin();
  
  left->run(FORWARD);
  right->run(FORWARD);
  
  left->setSpeed(baseSpeed);
  right->setSpeed(baseSpeed);
}

void loop() {
  if(Serial.available()) {
    scaling = (int) Serial.readStringUntil(',').toFloat();
    baseSpeed = (int) Serial.readStringUntil(',').toFloat();
    //sampling = (int) Serial.readStringUntil(',').toFloat();
  }
  
  if(millis() - counter >= SAMPLING_RATE) {
    err = analogRead(reflectancePin) - REF;
    scaleSpeeds();
    left->setSpeed((baseSpeed+leftWheelError) + leftSpeed);
    right->setSpeed(baseSpeed + rightSpeed);
  }
}

// scaling
void scaleSpeeds() {
 if(err > 0) {
   leftSpeed = (int) ((err*10)/50)*scaling; 
   rightSpeed = (int) ((err*-10)/50)*scaling;
 } else if(err < 0) {
   rightSpeed = (int) ((err*-10)/50)*scaling;
   leftSpeed = (int) ((err*10)/50)*scaling;
 } else {
   leftSpeed = 1;
   rightSpeed = 1;
 } 
}
