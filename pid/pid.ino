// Variables for PID control
double pTerm, iTerm, dTerm;
double pGain = 5, iGain = .1, dGain = 50;
double iState = 0, dState = 0;
const double IMAX = 5, IMIN = -5;

// vRef used in subtractor circuit
const double REF = 2.80;

// pins
int reflectancePin = A0;
double err = 0;

// debouncing
unsigned long counter = 0;
const unsigned long SAMPLING_RATE = 100;

void setup() {
  Serial.begin(9600);
  
  pinMode(reflectancePin, INPUT);
}

void loop() {
  if(millis() - counter >= SAMPLING_RATE) {
    err = analogRead(reflectancePin) - REF;
    Serial.println(updatePID(err));
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
   iState = iMax; 
  }
  
  if(iState < IMIN) {
   iState = iMin; 
  }
  
  iTerm = iState * iGain;
}

// calculating the derivative
void calcDerivative(double pos) {
 dTerm = dGain * (dState - pos);
 dState = pos;
}
