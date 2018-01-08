/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */
#include "Configuration.h"
#include "temperature.h"
#include "thermistortables.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
//int target_temperature[PROBES] = { 0 };
int target_temperature_bed = 0;
#define HEATER_BED_PIN 6

//int current_temperature_raw[PROBES] = { 0 };
//float current_temperature[PROBES] = { 0.0 };

int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;

#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  int redundant_temperature_raw = 0;
  float redundant_temperature = 0.0;
#endif

#ifdef PIDTEMP
  float Kp=DEFAULT_Kp;
  float Ki=(DEFAULT_Ki*PID_dT);
  float Kd=(DEFAULT_Kd/PID_dT);
#endif //PIDTEMP

#ifdef PIDTEMPBED
  float bedKp=DEFAULT_bedKp;
  float bedKi=(DEFAULT_bedKi*PID_dT);
  float bedKd=(DEFAULT_bedKd/PID_dT);
#endif //PIDTEMPBED
  
#ifdef FAN_SOFT_PWM
  unsigned char fanSpeedSoftPwm;
#endif

unsigned char soft_pwm_bed;
  
#ifdef BABYSTEPPING
  volatile int babystepsTodo[3]={0,0,0};
#endif

//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[PROBES] = { 0 };
  static float temp_dState[PROBES] = { 0 };
  static float pTerm[PROBES];
  static float iTerm[PROBES];
  static float dTerm[PROBES];
  //int output;
  static float pid_error[PROBES];
  static float temp_iState_min[PROBES];
  static float temp_iState_max[PROBES];
  // static float pid_input[PROBES];
  // static float pid_output[PROBES];
  static bool pid_reset[PROBES];
#endif //PIDTEMP

#ifdef PIDTEMPBED
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
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm[PROBES];

#ifdef FAN_SOFT_PWM
  static unsigned char soft_pwm_fan;
#endif
#if (defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1) || \
    (defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1) || \
    (defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1)
  static unsigned long PROBE_autofan_last_check;
#endif  

#if PROBES > 3
  # error Unsupported number of PROBES
#elif PROBES > 2
  static int minttemp_raw[PROBES] = { HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP };
  static int maxttemp_raw[PROBES] = { HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP };
  static int minttemp[PROBES] = { 0, 0, 0 };
  static int maxttemp[PROBES] = { 16383, 16383, 16383 };
#elif PROBES > 1
  static int minttemp_raw[PROBES] = { HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP };
  static int maxttemp_raw[PROBES] = { HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP };
  static int minttemp[PROBES] = { 0, 0 };
  static int maxttemp[PROBES] = { 16383, 16383 };
#else
  static int minttemp_raw[PROBES] = { HEATER_0_RAW_LO_TEMP };
  static int maxttemp_raw[PROBES] = { HEATER_0_RAW_HI_TEMP };
  static int minttemp[PROBES] = { 0 };
  static int maxttemp[PROBES] = { 16383 };
#endif

#ifdef BED_MAXTEMP
  static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  static void *heater_ttbl_map[2] = {(void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE };
  static uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
#endif

static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
  int watch_start_temp[PROBES] = ARRAY_BY_PROBES(0,0,0);
  unsigned long watchmillis[PROBES] = ARRAY_BY_PROBES(0,0,0);
#endif //WATCH_TEMP_PERIOD

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

//===========================================================================
//=============================   functions      ============================
//===========================================================================

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

#if (defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1) || \
    (defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1) || \
    (defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1)
  unsigned long PROBE_autofan_last_check = millis();
#endif

  if ((PROBE >= PROBES)
  #if (TEMP_BED_PIN <= -1)
       ||(PROBE < 0)
  #endif
       ){
          Serial.println("PID Autotune failed. Bad PROBE number.");
          return;
        }
	
  Serial.println("PID Autotune start");
  
  disable_heater(); // switch off all heaters.

  if (PROBE<0)
  {
     soft_pwm_bed = (MAX_BED_POWER)/2;
     bias = d = (MAX_BED_POWER)/2;
   }
   else
   {
     soft_pwm[PROBE] = (PID_MAX)/2;
     bias = d = (PID_MAX)/2;
  }


 for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = (PROBE<0)?current_temperature_bed:current_temperature[PROBE];

      max=max(max,input);
      min=min(min,input);

      #if (defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1) || \
          (defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1) || \
          (defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1)
      if(millis() - PROBE_autofan_last_check > 2500) {
        checkPROBEAutoFans();
        PROBE_autofan_last_check = millis();
      }
      #endif

      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) { 
          heating=false;
          if (PROBE<0)
            soft_pwm_bed = (bias - d) >> 1;
          else
            soft_pwm[PROBE] = (bias - d) >> 1;
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
            bias = constrain(bias, 20 ,(PROBE<0?(MAX_BED_POWER):(PID_MAX))-20);
            if(bias > (PROBE<0?(MAX_BED_POWER):(PID_MAX))/2) d = (PROBE<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
            else d = bias;

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
          if (PROBE<0)
            soft_pwm_bed = (bias + d) >> 1;
          else
            soft_pwm[PROBE] = (bias + d) >> 1;
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
      if (PROBE<0){
        p=soft_pwm_bed;       
        Serial.print("ok B:");
      }else{
        p=soft_pwm[PROBE];       
        Serial.print("ok T:");
      }
			
      Serial.print(input);   
      Serial.print(" @:");
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
  }
}

void updatePID()
{
#ifdef PIDTEMP
  for(int e = 0; e < PROBES; e++) { 
     temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;  
  }
#endif //PIDTEMP
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;  
#endif //PIDTEMPBED
}
  
int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
}

#if (defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1) || \
    (defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1) || \
    (defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1)

  #if defined(FAN_PIN) && FAN_PIN > -1
    #if PROBE_0_AUTO_FAN_PIN == FAN_PIN 
       #error "You cannot set PROBE_0_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
    #if PROBE_1_AUTO_FAN_PIN == FAN_PIN 
       #error "You cannot set PROBE_1_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
    #if PROBE_2_AUTO_FAN_PIN == FAN_PIN 
       #error "You cannot set PROBE_2_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
  #endif 

void setPROBEAutoFanState(int pin, bool state)
{
  unsigned char newFanSpeed = (state != 0) ? PROBE_AUTO_FAN_SPEED : 0;
  // this idiom allows both digital and PWM fan outputs (see M42 handling).
  pinMode(pin, OUTPUT);
  digitalWrite(pin, newFanSpeed);
  analogWrite(pin, newFanSpeed);
}

void checkPROBEAutoFans()
{
  uint8_t fanState = 0;

  // which fan pins need to be turned on?      
  #if defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1
    if (current_temperature[0] > PROBE_AUTO_FAN_TEMPERATURE) 
      fanState |= 1;
  #endif
  #if defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1
    if (current_temperature[1] > PROBE_AUTO_FAN_TEMPERATURE) 
    {
      if (PROBE_1_AUTO_FAN_PIN == PROBE_0_AUTO_FAN_PIN) 
        fanState |= 1;
      else
        fanState |= 2;
    }
  #endif
  #if defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1
    if (current_temperature[2] > PROBE_AUTO_FAN_TEMPERATURE) 
    {
      if (PROBE_2_AUTO_FAN_PIN == PROBE_0_AUTO_FAN_PIN) 
        fanState |= 1;
      else if (PROBE_2_AUTO_FAN_PIN == PROBE_1_AUTO_FAN_PIN) 
        fanState |= 2;
      else
        fanState |= 4;
    }
  #endif
  
  // update PROBE auto fan states
  #if defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1
    setPROBEAutoFanState(PROBE_0_AUTO_FAN_PIN, (fanState & 1) != 0);
  #endif 
  #if defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1
    if (PROBE_1_AUTO_FAN_PIN != PROBE_0_AUTO_FAN_PIN) 
      setPROBEAutoFanState(PROBE_1_AUTO_FAN_PIN, (fanState & 2) != 0);
  #endif 
  #if defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1
    if (PROBE_2_AUTO_FAN_PIN != PROBE_0_AUTO_FAN_PIN 
        && PROBE_2_AUTO_FAN_PIN != PROBE_1_AUTO_FAN_PIN)
      setPROBEAutoFanState(PROBE_2_AUTO_FAN_PIN, (fanState & 4) != 0);
  #endif 
}

#endif // any PROBE auto fan pins set

void manage_heater()
{
  float pid_input;
  float pid_output;

  if(temp_meas_ready != true)   //better readability
    return; 

  updateTemperaturesFromRawValues();

  for(int e = 0; e < PROBES; e++) 
  {

  #if defined (THERMAL_RUNAWAY_PROTECTION_PERIOD) && THERMAL_RUNAWAY_PROTECTION_PERIOD > 0
    thermal_runaway_protection(&thermal_runaway_state_machine[e], &thermal_runaway_timer[e], current_temperature[e], target_temperature[e], e, THERMAL_RUNAWAY_PROTECTION_PERIOD, THERMAL_RUNAWAY_PROTECTION_HYSTERESIS);
  #endif

  #ifdef PIDTEMP
    pid_input = current_temperature[e];

    #ifndef PID_OPENLOOP
        pid_error[e] = target_temperature[e] - pid_input;
        if(pid_error[e] > PID_FUNCTIONAL_RANGE) {
          pid_output = BANG_MAX;
          pid_reset[e] = true;
        }
        else if(pid_error[e] < -PID_FUNCTIONAL_RANGE || target_temperature[e] == 0) {
          pid_output = 0;
          pid_reset[e] = true;
        }
        else {
          if(pid_reset[e] == true) {
            temp_iState[e] = 0.0;
            pid_reset[e] = false;
          }
          pTerm[e] = Kp * pid_error[e];
          temp_iState[e] += pid_error[e];
          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
          iTerm[e] = Ki * temp_iState[e];

          //K1 defined in Configuration.h in the PID settings
          #define K2 (1.0-K1)
          dTerm[e] = (Kd * (pid_input - temp_dState[e]))*K2 + (K1 * dTerm[e]);
          pid_output = pTerm[e] + iTerm[e] - dTerm[e];
          if (pid_output > PID_MAX) {
            if (pid_error[e] > 0 )  temp_iState[e] -= pid_error[e]; // conditional un-integration
            pid_output=PID_MAX;
          } else if (pid_output < 0){
            if (pid_error[e] < 0 )  temp_iState[e] -= pid_error[e]; // conditional un-integration
            pid_output=0;
          }
        }
        temp_dState[e] = pid_input;
    #else 
          pid_output = constrain(target_temperature[e], 0, PID_MAX);
    #endif //PID_OPENLOOP
    #ifdef PID_DEBUG
    SERIAL_ECHO_START;
    SERIAL_ECHO(" PID_DEBUG ");
    SERIAL_ECHO(e);
    SERIAL_ECHO(": Input ");
    SERIAL_ECHO(pid_input);
    SERIAL_ECHO(" Output ");
    SERIAL_ECHO(pid_output);
    SERIAL_ECHO(" pTerm ");
    SERIAL_ECHO(pTerm[e]);
    SERIAL_ECHO(" iTerm ");
    SERIAL_ECHO(iTerm[e]);
    SERIAL_ECHO(" dTerm ");
    SERIAL_ECHOLN(dTerm[e]);
    #endif //PID_DEBUG
  #else /* PID off */
    pid_output = 0;
    if(current_temperature[e] < target_temperature[e]) {
      pid_output = PID_MAX;
    }
  #endif

    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e])) 
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
    }

    #ifdef WATCH_TEMP_PERIOD
    if(watchmillis[e] && millis() - watchmillis[e] > WATCH_TEMP_PERIOD)
    {
        if(degHotend(e) < watch_start_temp[e] + WATCH_TEMP_INCREASE)
        {
            setTargetHotend(0, e);
            LCD_MESSAGEPGM("Heating failed");
            SERIAL_ECHO_START;
            SERIAL_ECHOLN("Heating failed");
        }else{
            watchmillis[e] = 0;
        }
    }
    #endif
  } // End PROBE for loop

  #if (defined(PROBE_0_AUTO_FAN_PIN) && PROBE_0_AUTO_FAN_PIN > -1) || \
      (defined(PROBE_1_AUTO_FAN_PIN) && PROBE_1_AUTO_FAN_PIN > -1) || \
      (defined(PROBE_2_AUTO_FAN_PIN) && PROBE_2_AUTO_FAN_PIN > -1)
  if(millis() - PROBE_autofan_last_check > 2500)  // only need to check fan state very infrequently
  {
    checkPROBEAutoFans();
    PROBE_autofan_last_check = millis();
  }  
  #endif       
  
  #ifndef PIDTEMPBED
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  #endif

  #if TEMP_SENSOR_BED != 0
  
    #ifdef THERMAL_RUNAWAY_PROTECTION_BED_PERIOD && THERMAL_RUNAWAY_PROTECTION_BED_PERIOD > 0
      thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, 9, THERMAL_RUNAWAY_PROTECTION_BED_PERIOD, THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS);
    #endif

  #ifdef PIDTEMPBED
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
          	  if (pid_output > MAX_BED_POWER) {
            	    if (pid_error_bed > 0 )  temp_iState_bed -= pid_error_bed; // conditional un-integration
                    pid_output=MAX_BED_POWER;
          	  } else if (pid_output < 0){
            	    if (pid_error_bed < 0 )  temp_iState_bed -= pid_error_bed; // conditional un-integration
                    pid_output=0;
                  }

    #else 
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif //PID_OPENLOOP

	  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)) 
	  {
	    soft_pwm_bed = (int)pid_output >> 1;
	  }
	  else {
	    soft_pwm_bed = 0;
	  }

    #elif !defined(BED_LIMIT_SWITCHING)
      // Check if temperature is within the correct range
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed >= target_temperature_bed)
        {
          soft_pwm_bed = 0;
        }
        else 
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        digitalWrite(HEATER_BED_PIN,LOW);
      }
    #else //#ifdef BED_LIMIT_SWITCHING
      // Check if temperature is within the correct band
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS)
        {
          soft_pwm_bed = 0;
        }
        else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS)
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        digitalWrite(HEATER_BED_PIN,LOW);
      }
    #endif
  #endif 
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

// Derived from RepRap FiveD PROBE::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
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
  #else
    return 0;
  #endif
}

void tp_init(){
  // Finish init of mult PROBE arrays 
  for(int e = 0; e < PROBES; e++) {
    // populate with the first value 
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
#ifdef PIDTEMPBED
    temp_iState_min_bed = 0.0;
    temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif //PIDTEMPBED
  }
   
  #if defined(HEATER_BED_PIN) && (HEATER_BED_PIN > -1) 
    pinMode(HEATER_BED_PIN, OUTPUT);
  #endif  
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
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8); 
    #endif
  #endif
  #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
    #if TEMP_1_PIN < 8
       DIDR0 |= 1<<TEMP_1_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_1_PIN - 8); 
    #endif
  #endif
  #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
    #if TEMP_2_PIN < 8
       DIDR0 |= 1 << TEMP_2_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_2_PIN - 8); 
    #endif
  #endif
  #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
    #if TEMP_BED_PIN < 8
       DIDR0 |= 1<<TEMP_BED_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_BED_PIN - 8); 
    #endif
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
 // OCR0B = 512; //a test to decrease the temperature management, and thus LCD update, frequency; Josh, 8/19/2015
  TIMSK0 |= (1<<OCIE0B);  
  
  // Wait for temperature measurement to settle
  delay(250);

#ifdef HEATER_0_MINTEMP
  minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
    #if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
        minttemp_raw[0] += OVERSAMPLENR;
    #else
        minttemp_raw[0] -= OVERSAMPLENR;
    #endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
    #if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
        maxttemp_raw[0] -= OVERSAMPLENR;
    #else
        maxttemp_raw[0] += OVERSAMPLENR;
    #endif
  }
#endif //MAXTEMP

#if (PROBES > 1) && defined(HEATER_1_MINTEMP)
  minttemp[1] = HEATER_1_MINTEMP;
  while(analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
    #if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
        minttemp_raw[1] += OVERSAMPLENR;
    #else
        minttemp_raw[1] -= OVERSAMPLENR;
    #endif
  }
#endif // MINTEMP 1
#if (PROBES > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = HEATER_1_MAXTEMP;
  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
    #if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
        maxttemp_raw[1] -= OVERSAMPLENR;
    #else
        maxttemp_raw[1] += OVERSAMPLENR;
    #endif
  }
#endif //MAXTEMP 1

#if (PROBES > 2) && defined(HEATER_2_MINTEMP)
  minttemp[2] = HEATER_2_MINTEMP;
  while(analog2temp(minttemp_raw[2], 2) < HEATER_2_MINTEMP) {
    #if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
        minttemp_raw[2] += OVERSAMPLENR;
    #else
        minttemp_raw[2] -= OVERSAMPLENR;
    #endif
  }
#endif //MINTEMP 2
#if (PROBES > 2) && defined(HEATER_2_MAXTEMP)
  maxttemp[2] = HEATER_2_MAXTEMP;
  while(analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
    #if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
        maxttemp_raw[2] -= OVERSAMPLENR;
    #else
        maxttemp_raw[2] += OVERSAMPLENR;
    #endif
  }
#endif //MAXTEMP 2

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

void setWatch() 
{  
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < PROBES; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    } 
  }
#endif 
}

#if (defined (THERMAL_RUNAWAY_PROTECTION_PERIOD) && THERMAL_RUNAWAY_PROTECTION_PERIOD > 0) || (defined (THERMAL_RUNAWAY_PROTECTION_BED_PERIOD) && THERMAL_RUNAWAY_PROTECTION_BED_PERIOD > 0)
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
        Serial.print("Thermal Runaway, system stopped! Heater_ID: ");
        Serial.println((int)heater_id);
        //LCD_ALERTMESSAGEPGM("THERMAL RUNAWAY");                                                   // make an onscreen error message show
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
#endif

void disable_heater()
{
  #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
    target_temperature_bed=0;
    soft_pwm_bed=0;
    #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1  
      digitalWrite(HEATER_BED_PIN,LOW);
    #endif
  #endif 
}

void bed_max_temp_error(void) {
#if HEATER_BED_PIN > -1
  digitalWrite(HEATER_BED_PIN, 0);
#endif
    Serial.println("Temperature heated bed switched off. MAXTEMP triggered !!");
    Serial.println("Err: MAXTEMP BED");
    //LCD_ALERTMESSAGEPGM("THERMAL RUNAWAY");                                                   // make an onscreen error message show
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  disable_heater();
  #endif
}

void kill(){
  disable_heater();
  Serial.println("Kill(); routine. Cut power to heater"); //added by Corey
}

#ifdef PIDTEMP
// Apply the scale factors to the PID values

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

#endif //PIDTEMP


