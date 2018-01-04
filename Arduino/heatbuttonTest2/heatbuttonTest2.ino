// adapted from 'blink.ino' example sketch 12/29/2017 by Corey Rice
// ==============================================================================
// ==============================================================================
// =========== BE CAREFUL: Only run until bed is warm, then power off =========== 
// ==============================================================================
// ==============================================================================

int HEATER = 6;
int button = 2;

void setup() {
  // initialize digital pin HEATER as an output.
  pinMode(HEATER, OUTPUT);
  pinMode(button, INPUT_PULLUP);
}

void loop() {
  int buttonState = digitalRead(button);
  
  if(!buttonState){
    digitalWrite(HEATER, HIGH);   // turn the HEATER on (HIGH is the voltage level)
    delay(100);                       // wait for 1/10 of a second
  }else{
    digitalWrite(HEATER, LOW);    // turn the HEATER off by making the voltage LOW
    delay(50);                        // wait for 1/20 of a second
  }
}

// This (very slow) 67% duty cycle is enough that it should show the ability of the HEATER
// to function, without getting too hot too fast...

