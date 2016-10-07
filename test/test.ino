/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground
 
 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to
const int analogInPin2 = A2;
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0; // value read from the pot
int sensorValue2 = 0;
float outputValue = 0;
float outputValue2 = 0; // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin); 
  sensorValue2 = analogRead(analogInPin2);  
  // map it to the range of the analog out:
  outputValue = sensorValue * 5.0/1023.0;
  outputValue2 = sensorValue2 * 5.0/1023.0;
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);           

  // print the results to the serial monitor:
  Serial.print("right = " );                       
  Serial.print(outputValue);      
  Serial.print("\t left = ");      
  Serial.println(outputValue2);   

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);                     
}