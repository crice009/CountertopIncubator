#include "temperature.cpp"
#include "temperature.h"
#include "thermistortables.h"

#define THERMISTORHEATER_0 1
#define PIDTEMPBED 1
#define PROBES 0

#define HEATER_BED_PIN 6

// A Whole bunch of fancy options from Marlin
//#define TEMP_SENSOR_1_AS_REDUNDANT 1  //I think this is for a back-up temp sensor
//#define PIDTEMP 1                     //This is for if the 3D printer's extruder's temps are PID controlled
#define PIDTEMPBED                    //This is for if the 3D printer's bed's temps are PID controlled
//in case you are wondering at this point, non PID controlled temps are bang-bang controlled (google it)
//#define FAN_SOFT_PWM 1                //this may be to control the fan speed with a software PWM implimentation
//#define BABYSTEPPING 1                //this likely has to do with extruder babystepping effecting temp goals... not for us

#define led 13
// the setup function runs once when you press reset or power the board
void setup() {
    pinMode(led, OUTPUT);
    Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
    int temp = analog2tempBed(0);
    Serial.println(temp);
    delay(100);
    blinkWOD(temp);
}

//==============================================================================
//==============================================================================
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated

void blinkWOD(int temp){

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= temp*10) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(led, ledState);
  }
}
