#include "temperature.cpp"
#include "Configuration.h"

#define BED_USES_THERMISTOR  //this tells the firmware that the bed uses a thermistor...
#define TEMP_SENSOR_BED A0   //this defines the Arduino pin that is the bed's temp sensor
#define THERMISTORBED 1      //this defines what type of thermistor is used, and hence the voltage-temp table that is referenced. all in this build are a 100k thermistor with a 4.7k pull-up resistor.
#define HEATER_BED_PIN 6     //this defines the Arduino pin that controls the heater
#define heater_id 1


#define led 13
// the setup function runs once when you press reset or power the board
void setup() {
    pinMode(led, OUTPUT);
    Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
    float temp = analog2tempBed(0);
    float ftemp = ((9/5)*temp) + 32;
    Serial.print(temp);
    Serial.print("C -----> ");
    Serial.print(ftemp);
    Serial.println("F");
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
