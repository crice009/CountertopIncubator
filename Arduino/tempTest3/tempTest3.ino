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
//============================ Functions =======================================
//==============================================================================

int ledState = LOW;                   //global -- used in blinkWOD()
unsigned long previousMillis = 0;     //global -- used in blinkWOD()

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

//================================================================================
byte pwm_bed = 0;                   //global -- used in many many functions

void PID_autotune(float temp, int PROBE, int ncycles)
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  if (false){               //need to figure out what would cause PID autotune to fail at startup, and put that here...
    Serial.println("PID Autotune failed. Bad PROBE number.");
    return;
  }
  
  Serial.println("PID Autotune start");
  
  disable_heater(); // switch off all heaters.

  pwm_bed = (MAX_BED_POWER)/2;
  bias = d = (MAX_BED_POWER)/2;

  for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();   //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

      input = current_temperature_bed;

      max=max(max,input);
      min=min(min,input);

      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) { 
          heating=false;
          pwm_bed = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 , MAX_BED_POWER-20);
            if(bias > MAX_BED_POWER) {
              d = MAX_BED_POWER - 1 - bias;
            }else{
              d = bias;
            }

            Serial.print(" bias: "); Serial.print(bias);
            Serial.print(" d: "); Serial.print(d);
            Serial.print(" min: "); Serial.print(min);
            Serial.print(" max: "); Serial.println(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              Serial.print(" Ku: "); Serial.print(Ku);
              Serial.print(" Tu: "); Serial.println(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              Serial.println(" Classic PID ");
              Serial.print(" Kp: "); Serial.println(Kp);
              Serial.print(" Ki: "); Serial.println(Ki);
              Serial.print(" Kd: "); Serial.println(Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;
              Serial.println(" Some overshoot ");
              Serial.print(" Kp: "); Serial.println(Kp);
              Serial.print(" Ki: "); Serial.println(Ki);
              Serial.print(" Kd: "); Serial.println(Kd);
              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;
              Serial.println(" No overshoot ");
              Serial.print(" Kp: "); Serial.println(Kp);
              Serial.print(" Ki: "); Serial.println(Ki);
              Serial.print(" Kd: "); Serial.println(Kd);
              */
            }
          }
          pwm_bed = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      } 
    }
    if(input > (temp + 20)) {
      Serial.println("PID Autotune failed! Temperature too high");
      return;
    }
    if(millis() - temp_millis > 2000) {
      int p;
      p=pwm_bed;       
      Serial.print("ok. Temp:");
      Serial.print(input);   
      Serial.print(" @pwm:");
      Serial.println(p);       

      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      Serial.println("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      Serial.println("PID Autotune finished! Put the last Kp, Ki and Kd constants from above into Configuration.h");
      return;
    }
    //should include some OLED display commands, so this could be done done independent of a computer ...... maybe sometime in the future....
  }
}

//===========================================================================
void updatePID(){
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;  
}

//===========================================================================
int getHeaterPower() {
  return pwm_bed;
}

//===========================================================================
void manage_heater()
{
  float pid_input;
  float pid_output;

  if(temp_meas_ready != true) return; 

  updateTemperaturesFromRawValues(); //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   
  
  previous_millis_bed_heater = millis();

  thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, 9, THERMAL_RUNAWAY_PROTECTION_BED_PERIOD, THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS);
  pid_input = current_temperature_bed;

  #ifndef PID_OPENLOOP
    pid_error_bed = target_temperature_bed - pid_input;
    pTerm_bed = bedKp * pid_error_bed;
    temp_iState_bed += pid_error_bed;
    temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
    iTerm_bed = bedKi * temp_iState_bed;

    //K1 defined in Configuration.h in the PID settings
    #define K2 (1.0-K1)
    dTerm_bed= (bedKd * (pid_input - temp_dState_bed))*K2 + (K1 * dTerm_bed);
    temp_dState_bed = pid_input;

    pid_output = pTerm_bed + iTerm_bed - dTerm_bed;
    if(pid_output > MAX_BED_POWER) {
      if (pid_error_bed > 0 )  temp_iState_bed -= pid_error_bed; // conditional un-integration
      pid_output=MAX_BED_POWER;
    }else if(pid_output < 0){
      if (pid_error_bed < 0 )  temp_iState_bed -= pid_error_bed; // conditional un-integration
      pid_output=0;
    }
  #else 
    pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
  #endif //PID_OPENLOOP

  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)) {
    pwm_bed = (int)pid_output >> 1;
  }else{
    pwm_bed = 0;
  }

    #elif !defined(BED_LIMIT_SWITCHING)
      // Check if temperature is within the correct range
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)){
        if(current_temperature_bed >= target_temperature_bed){
          pwm_bed = 0;
        }else{
          pwm_bed = MAX_BED_POWER>>1;
        }
      }else{
        pwm_bed = 0;
        digitalWrite(HEATER_BED_PIN,LOW);
      }
    #else //#ifdef BED_LIMIT_SWITCHING
      // Check if temperature is within the correct band
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)){
        if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS){
          pwm_bed = 0;
        }else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS){
          pwm_bed = MAX_BED_POWER>>1;
        }
      }else{
        soft_pwm_bed = 0;
        digitalWrite(HEATER_BED_PIN,LOW);
      }
    #endif
}

//=============================================================================================
#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD PROBE::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
    float celsius = 0;
    byte i;

    for (i=1; i<BEDTEMPTABLE_LEN; i++)
    {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
      {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) + 
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) * 
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);

    return celsius;
}

//==============================================================================================
        //derived from public variables of temperature.cpp in MARLIN
int target_temperature_bed = 0;
#define HEATER_BED_PIN 6
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;
float bedKp=DEFAULT_bedKp;
float bedKi=(DEFAULT_bedKi*PID_dT);
float bedKd=(DEFAULT_bedKd/PID_dT);
        //derived from private variables of temperature.cpp in MARLIN
static volatile bool temp_meas_ready = false;
//static cannot be external:
static float temp_iState_bed = { 0 };
static float temp_dState_bed = { 0 };
static float pTerm_bed;
static float iTerm_bed;
static float dTerm_bed;
//int output;
static float pid_error_bed;
static float temp_iState_min_bed;
static float temp_iState_max_bed;
//
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
//
static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
void tp_init(){
  temp_iState_min_bed = 0.0;
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
    
  pinMode(HEATER_BED_PIN, OUTPUT);
  #if defined(FAN_PIN) && (FAN_PIN > -1) 
    SET_OUTPUT(FAN_PIN);
    #ifdef FAST_PWM_FAN
      setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #ifdef FAN_SOFT_PWM
      soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif  

  // Set analog inputs  //this uses a strange implimentation of bit-shifts (not common in Arduino --> possibly change)
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;                    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  DIDR0 = 0;                                                      //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  #ifdef DIDR2                                                    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    DIDR2 = 0;                                                    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  #endif                                                          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)                //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #if TEMP_BED_PIN < 8                                          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
       DIDR0 |= 1<<TEMP_BED_PIN;                                  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #else                                                         //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
       DIDR2 |= 1<<(TEMP_BED_PIN - 8);                            //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #endif                                                        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  #endif                                                          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
 // OCR0B = 512; //a test to decrease the temperature management, and thus LCD update, frequency; Josh, 8/19/2015
  TIMSK0 |= (1<<OCIE0B);  
  
  // Wait for temperature measurement to settle
  delay(250);

#ifdef BED_MINTEMP
  /* No bed MINTEMP error implemented?!? */ /*
  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
    #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
    #else
        bed_minttemp_raw -= OVERSAMPLENR;
    #endif
  }
  */
#endif //BED_MINTEMP  no error?
#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
    #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
    #else
        bed_maxttemp_raw += OVERSAMPLENR;
    #endif
  }
#endif //BED_MAXTEMP
}

//=================================================================================================
void thermal_runaway_protection(int *state, unsigned long *timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc)
{
/*
      SERIAL_ECHO_START;
      SERIAL_ECHO("Thermal Thermal Runaway Running. Heater ID:");
      SERIAL_ECHO(heater_id);
      SERIAL_ECHO(" ;  State:");
      SERIAL_ECHO(*state);
      SERIAL_ECHO(" ;  Timer:");
      SERIAL_ECHO(*timer);
      SERIAL_ECHO(" ;  Temperature:");
      SERIAL_ECHO(temperature);
      SERIAL_ECHO(" ;  Target Temp:");
      SERIAL_ECHO(target_temperature);
      SERIAL_ECHOLN("");    
*/
  if ((target_temperature == 0) || thermal_runaway)
  {
    *state = 0;
    *timer = 0;
    return;
  }
  switch (*state)
  {
    case 0: // "Heater Inactive" state
      if (target_temperature > 0) *state = 1;
      break;
    case 1: // "First Heating" state
      if (temperature >= target_temperature) *state = 2;
      break;
    case 2: // "Temperature Stable" state
      if (temperature >= (target_temperature - hysteresis_degc))
      {
        *timer = millis();
      } 
      else if ( (millis() - *timer) > ((unsigned long) period_seconds) * 1000)
      {
        Serial.print("Thermal Runaway, system stopped!");
        //LCD_ALERTMESSAGEPGM("THERMAL RUNAWAY");   // make an onscreen error message show
        thermal_runaway = true;
        while(1)
        {
          disable_heater();
          manage_heater();
        }
      }
      break;
  }
}

void disable_heater()
{
  target_temperature_bed=0;
  pwm_bed=0; 
  digitalWrite(HEATER_BED_PIN,LOW);
}

void bed_max_temp_error() {
  digitalWrite(HEATER_BED_PIN, 0);
  Serial.println("Temperature heated bed switched off. MAXTEMP triggered !!");
  Serial.println("Err: MAXTEMP BED");
    //LCD_ALERTMESSAGEPGM("THERMAL RUNAWAY");    // make an onscreen error message show
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  disable_heater();
  #endif
}

float scalePID_i(float i)
{
  return i*PID_dT;
}

float unscalePID_i(float i)
{
  return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
  return d*PID_dT;
}
