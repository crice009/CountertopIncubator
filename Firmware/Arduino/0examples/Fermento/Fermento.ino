
/*
 * (c) Fakufaku 2013
 * This is the code for the homebrew incubator 'Fermento'
 * This code is released under the CC-BY-SA 3.0 License.
 * The part of the code that's used for the display on the 7-segment was ripped off the BigTime code from Nathan Seidl. I owe him a beer then.
 */

/*
 * This is derived from the code of Fakufaku and Nathan Seidi (as mentioned above). 
 * See their original code here: https://github.com/BioDesignRealWorld/Fermento
 * 
 * I guess I owe them both a beer then. 
 * 
 */

#include <math.h>
#include <PID_v1.h>

#define TRUE 1
#define FALSE 0

#define ENTER_CRIT()    {byte volatile saved_sreg = SREG; cli()
#define LEAVE_CRIT()    SREG = saved_sreg;}

// slow continuous PWM variables
#define PWM_MS 1000
#define PWM_MIN 5
#define HEAT_ON  HIGH
#define HEAT_OFF LOW

// PID loop parameter
#define PID_KP  50
#define PID_KI  1
#define PID_KD  1

// Some display related parameters (time in seconds, temperature in degrees Celsius)
#define TIME_INCREMENT 1800
#define MAX_TIME 356459
#define TEMP_INCREMENT 1
#define MAX_TEMP 65

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// OPTIONS
//
// Voltage reference of the ADC
#define ADC_REF 5000

int temp_sen = A0;  // to read the temperature sensor

const static int button1 = 2;
const static int button2 = A5;
const static int button3 = 3;
const static int pwm_pin = A3;  // Pin 4

// The interrupt of button1 is on external interrupt 0
// The interrupt of button2 is on PCINT13 (A5), on PCIE1.
#define BUTTON1_INT_VECT INT0_vect
#define BUTTON2_INT_VECT PCINT1_vect
#define BUTTONS_INT_SET() do  \
  {                           \
    EICRA = (1<<ISC01); /* falling edge */     \
    EIMSK = (1<<INT0);        \
    PCICR = (1 << PCIE1);     \
    PCMSK1 = (1 << PCINT13);  \
  }                           \
  while (0)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


// Display status variable
#define TIME 1
#define TEMP 2
int display_status = TEMP;
int backoff = 0;  // just a short backoff for when we switch the display between TIME/TEMP
#define BACKOFFTIME 2
char disp_str[4];

// Variable to keep track of PWM window
unsigned long windowStartTime;

// control variables for PID
double t_incub;      // input to PID (temperature in incubator)
double t_incub_N;    // number of samples in t_incub average
double pwm_duty;     // output of PID
double t_set = 0;    // set point (target temperature)

//Specify the links and initial tuning parameters
PID myPID(&t_incub, &pwm_duty, &t_set, PID_KP, PID_KI, PID_KD, DIRECT);

// display timer (freeze parameters to display for 2 seconds)
int t_incub_disp;

// Timer variables
unsigned long timer_seconds = 0;

//The very important 32.686kHz interrupt handler
SIGNAL(TIMER2_OVF_vect)
{

  ENTER_CRIT();

  if (timer_seconds > 1)
  {
    timer_seconds--;
  }
  else if (timer_seconds == 1)
  {
    // reset timer
    timer_seconds = 0;
    // turn off by setting target temperature to zero
    t_set = 0;
    // Reset the PID
    myPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);
  }

  // decrement backoff
  if (backoff > 0)
    backoff--;

  LEAVE_CRIT();
}

//The interrupt occurs when you push the button
SIGNAL(BUTTON1_INT_VECT)
{
  ENTER_CRIT();
  display_status = TEMP;
  backoff = BACKOFFTIME;
  LEAVE_CRIT();
}

SIGNAL(BUTTON2_INT_VECT)
{
  ENTER_CRIT();
  display_status = TIME;
  backoff = BACKOFFTIME;
  LEAVE_CRIT();
}

void setup()
{
  Serial.begin(57600);

  // set ADC reference to 3.3V
  analogReference(DEFAULT);

  // initialize pwm drive pin, and turn it off
  pinMode(pwm_pin, OUTPUT);
  digitalWrite(pwm_pin, HIGH);

  // initialize the buttons
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  BUTTONS_INT_SET();

  //Setup TIMER2
  TCCR2A = 0x00;
  TCCR2B = (1<<CS22)|(1<<CS20); //Set CLK/128 or overflow interrupt every 1s
  ASSR   = (1<<AS2); //Enable asynchronous operation
  TIMSK2 = (1<<TOIE2); //Enable the timer 2 interrupt

  // setup PID stuff
  myPID.SetOutputLimits(PWM_MIN-1, PWM_MS-PWM_MIN+1);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(PWM_MS);
  windowStartTime = millis();

  // initialize pwm
  pwm_duty = 0; // off
  t_incub = read_temperature();
  t_incub_N = 1;

  // initialize slow variables for display
  t_incub_disp = t_incub;

}

void loop()
{
  
  temperatureControl();

  // display and output to serial
  if (display_status == TEMP)
  {
    delay(1);  //place holder for displaying the temperature on screen
  }
  else
  {
    unsigned int timer_disp;
    if (timer_seconds % 60 != 0)
      timer_disp = seconds2hours_minutes(timer_seconds + (60 - (timer_seconds%60)));
    else
      timer_disp = seconds2hours_minutes(timer_seconds);
    delay(1);  //place holder for displaying the time on screen
  }

  // handle buttons
  int b1 = digitalRead(button1);
  int b2 = digitalRead(button2);
  int b3 = digitalRead(button3);
  delay(1); // placeholder for button operation

}


// Simple PWM based on millis directly
void temperatureControl()
{
  unsigned long now = millis();

  // averaging over the whole second
  t_incub += (read_temperature() - t_incub)/(++t_incub_N);

  if(now - windowStartTime > PWM_MS)
  { 
    // compute new pwm value for that window
    if (t_set != 0)
      myPID.Compute();

    // start a new HEAT window
    windowStartTime = now;

    // restart averaging of temperature
    t_incub_disp = t_incub;
    t_incub = read_temperature();
    t_incub_N = 1;

  }

  // if t_set is zero, that means incubator is off
  if (t_set < 1)
  {
    pwm_duty = 0;
  }

  // control the slow PWM
  if (pwm_duty < PWM_MIN)
  {
    digitalWrite(pwm_pin, HEAT_OFF);
  }
  else if (pwm_duty < PWM_MS-PWM_MIN)
  {
    if(pwm_duty > now - windowStartTime) 
      digitalWrite(pwm_pin, HEAT_ON);
    else 
      digitalWrite(pwm_pin, HEAT_OFF);
  }
  else
  {
    digitalWrite(pwm_pin, HEAT_ON);
  }
}

float read_temperature()
{
  // make an average of 10 readings of the ADC
  float A = 0;
  for (int i=0 ; i < 10 ; i++)
    A += analogRead(temp_sen);
  A /= 10;

  // convert the ADC output to a temperature
#ifndef LM35
  // LM61 sensor
  return (A/1023.*ADC_REF - 600)/10;
#else
  // LM35 sensor
  return (A/1023.*ADC_REF)/10;
#endif
}

//This routine occurs when you hold the button 1 down
//The colon blinks indicating we are in this mode
//Holding the button down will increase the time (accelerates)
//Releasing the button for more than 2 seconds will exit this mode
void setTemperature()
{
  ENTER_CRIT();
  delay(1); //place holder
  LEAVE_CRIT();
}

//This routine occurs when you hold the button 2 down
//The colon blinks indicating we are in this mode
//Holding the button down will increase the time (accelerates)
//Releasing the button for more than 2 seconds will exit this mode
void setTime()
{
  ENTER_CRIT();
    delay(1); //place holder
  LEAVE_CRIT();
}

/* transform a number of seconds into a 4 digit hhmm (hours/minutes) number to display */
unsigned int seconds2hours_minutes(unsigned long seconds)
{
    unsigned int hours = seconds/3600;
    unsigned int minutes = (seconds % 3600)/60;
    return hours*100 + minutes;
}

