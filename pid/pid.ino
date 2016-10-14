#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Initialize the motorshield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create the motor objects
Adafruit_DCMotor *left = AFMS.getMotor(3);
Adafruit_DCMotor *right = AFMS.getMotor(4);

// vRef used in subtractor circuit
const double REF = 2.92;

// pins
int reflectancePin = A3;
int refPin = A5;
double err = 0;

// debouncing
unsigned long counter = 0;
const unsigned long SAMPLING_RATE = 50;

// voltage reading
double refVolt = 0;
double senseVolt = 0;

// speed calculation
int baseSpeed = 20;
int leftWheelError = 0;
int rightWheelError = 10;
uint8_t leftSpeed = 1, rightSpeed = 1;
int scaling = 15;
double tolerance = 0.03;

void setup() {
  Serial.begin(9600);
  
  pinMode(reflectancePin, INPUT);
  pinMode(refPin, INPUT);
  
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
  
  Serial.println(senseVolt);
  
  if(millis() - counter >= SAMPLING_RATE) {
    counter = millis();
    refVolt = (analogRead(refPin)*5.0/1023);
    senseVolt = (analogRead(reflectancePin)*5.0/1023);
    err = senseVolt-refVolt;
    scaleSpeeds();
    left->setSpeed((baseSpeed+leftWheelError) + leftSpeed);
    right->setSpeed((baseSpeed+rightWheelError) + rightSpeed);
    
  }
}

// scaling
void scaleSpeeds() {
 if(err > 0) {
   leftSpeed = -1 * scaling; 
   rightSpeed = scaling;
 } else if(err < 0) {
   leftSpeed = scaling;
   rightSpeed = -1 * scaling;
   // (int) ((err*-100)/50)*
 } else {
   leftSpeed = 0;
   rightSpeed = 0;
 } 
}
