#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define TEMP_SENSOR_BED A0   //this defines the Arduino pin that is the bed's temp sensor
#define THERMISTORBED 1      //this defines what type of thermistor is used, and hence the voltage-temp table that is referenced. all in this build are a 100k thermistor with a 4.7k pull-up resistor.
//#define HEATER_BED_PIN 6     //this defines the Arduino pin that controls the heater
#define led 13
#define ARRAY_LENGTH 20 //Change this value to modulate the smoothing (can only work for all with smoothing)
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

int tHours = 96;
int tTemp  = 86;
byte seconds  = 0;                 //used in timeCounter() and runningOLED()
byte minutes  = 0;                 //used in timeCounter() and runningOLED()
byte hours    = 0;                 //used in timeCounter() and runningOLED()
byte days     = 0;                 //used in timeCounter() and runningOLED()
byte Rdays    = tHours / 24;       //used in timeCounter() and runningOLED()
byte Rhours   = tHours % 24;       //used in timeCounter() and runningOLED()
byte Rminutes = Rhours % 60;       //used in timeCounter() and runningOLED()
byte Rseconds = Rminutes % 60;     //used in timeCounter() and runningOLED()




void setup() {
  pinMode(TEMP_SENSOR_BED, INPUT);
  //pinMode(HEATER_BED_PIN, OUTPUT);
  Serial.begin(9600);

  for(short i=0; i<ARRAY_LENGTH; i++){
    float raw = analogRead(TEMP_SENSOR_BED);
    raw = smoothing(raw);
  }

  display.begin(SSD1306_SWITCHCAPVCC);
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();  
}

void loop() {
    float raw = analogRead(TEMP_SENSOR_BED);
    raw = smoothing(raw);
    Serial.print(raw);
    Serial.print(" analogRead        ");
    float temp = analog2temp(raw);
    float ftemp = ((9*temp)/5) + 32;
    Serial.print(temp);
    Serial.print("C          ");
    Serial.print(ftemp);
    Serial.println("F");
    timeCounter(tHours);
    runningOLED(round(ftemp));
    delay(10);
    //bangBang(tTemp, ftemp);
}

#define OVERSAMPLENR 1
const short BEDTEMPTABLE[][2] PROGMEM = {
{       23*OVERSAMPLENR ,       300     },        
{       25*OVERSAMPLENR ,       295     },
{       27*OVERSAMPLENR ,       290     },
{       28*OVERSAMPLENR ,       285     },
{       31*OVERSAMPLENR ,       280     },
{       33*OVERSAMPLENR ,       275     },
{       35*OVERSAMPLENR ,       270     },
{       38*OVERSAMPLENR ,       265     },
{       41*OVERSAMPLENR ,       260     },
{       44*OVERSAMPLENR ,       255     },
{       48*OVERSAMPLENR ,       250     },
{       52*OVERSAMPLENR ,       245     },
{       56*OVERSAMPLENR ,       240     },
{       61*OVERSAMPLENR ,       235     },
{       66*OVERSAMPLENR ,       230     },
{       71*OVERSAMPLENR ,       225     },
{       78*OVERSAMPLENR ,       220     },
{       84*OVERSAMPLENR ,       215     },
{       92*OVERSAMPLENR ,       210     },
{       100*OVERSAMPLENR        ,       205     },
{       109*OVERSAMPLENR        ,       200     },
{       120*OVERSAMPLENR        ,       195     },
{       131*OVERSAMPLENR        ,       190     },
{       143*OVERSAMPLENR        ,       185     },
{       156*OVERSAMPLENR        ,       180     },
{       171*OVERSAMPLENR        ,       175     },
{       187*OVERSAMPLENR        ,       170     },
{       205*OVERSAMPLENR        ,       165     },
{       224*OVERSAMPLENR        ,       160     },
{       245*OVERSAMPLENR        ,       155     },
{       268*OVERSAMPLENR        ,       150     },
{       293*OVERSAMPLENR        ,       145     },
{       320*OVERSAMPLENR        ,       140     },
{       348*OVERSAMPLENR        ,       135     },
{       379*OVERSAMPLENR        ,       130     },
{       411*OVERSAMPLENR        ,       125     },
{       445*OVERSAMPLENR        ,       120     },
{       480*OVERSAMPLENR        ,       115     },
{       516*OVERSAMPLENR        ,       110     },
{       553*OVERSAMPLENR        ,       105     },
{       591*OVERSAMPLENR        ,       100     },  //212F
{       628*OVERSAMPLENR        ,       95      },  //204F
{       665*OVERSAMPLENR        ,       90      },  //194F
{       702*OVERSAMPLENR        ,       85      },  //185F
{       737*OVERSAMPLENR        ,       80      },  //176F
{       770*OVERSAMPLENR        ,       75      },  //167F
{       801*OVERSAMPLENR        ,       70      },  //158F
{       830*OVERSAMPLENR        ,       65      },  //149F
{       857*OVERSAMPLENR        ,       60      },  //140F
{       881*OVERSAMPLENR        ,       55      },  //131F
{       903*OVERSAMPLENR        ,       50      },  //122F
{       922*OVERSAMPLENR        ,       45      },  //113F
{       939*OVERSAMPLENR        ,       40      },  //104F
{       954*OVERSAMPLENR        ,       35      },  //95F
{       966*OVERSAMPLENR        ,       30      },  //86F
{       977*OVERSAMPLENR        ,       25      },  //77F
{       985*OVERSAMPLENR        ,       20      },  //68F
{       993*OVERSAMPLENR        ,       15      },  //59F
{       999*OVERSAMPLENR        ,       10      },  //50F
{       1004*OVERSAMPLENR       ,       5       },  //41F
{       1008*OVERSAMPLENR       ,       0       } //safety
};

//==============================================================================
//============================ Functions =======================================
//==============================================================================
unsigned long previousMillis = 0; //used in timeCounter() only
#define PGM_RD_W(x)   (short)pgm_read_word(&x) //used in analog2temp()

void runningOLED(int temp){           //display is 128x32 pixels and the cursor can be set...
  //temperature line
    display.setTextSize(2);           //'size 2' font is 16 pixels high
    display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
    display.setCursor(0,0);
    display.print("Temp: ");
    display.print(temp);
    display.println("F");
  //end temperature line
  //remaining time line
    display.setTextSize(1);          //'size 1' font is 8 pixels high
    display.setCursor(0,16);         //can place text >> setCursor is top left of first char
    display.print("Left: ");
    display.print(Rdays);
    display.print("d ");
    display.print(Rhours);
    display.print("h ");
    display.print(Rminutes);
    display.print("m ");
    display.print(Rseconds);
    display.print("s");
  //end remaining time lines
  //Elapsed timer line
    display.setTextSize(1);          //'size 1' font is 8 pixels high
    display.setCursor(0,24);         //can place text >> setCursor is top left of first char 
    display.print("Done: ");
    display.print(days);
    display.print("d ");
    display.print(hours);
    display.print("h ");
    display.print(minutes);
    display.print("m ");
    display.print(seconds);
    display.print("s");
  //end Elapsed timer line
    display.display();
    display.clearDisplay();
}

void timeCounter(int tHours){
  unsigned long currentMillis = millis();
  if ( (currentMillis - previousMillis) >= 1000){
    previousMillis = previousMillis + 1000;
    seconds = seconds +1;
    if (seconds == 60){
      seconds = 0;
      minutes = minutes +1;
      if(minutes == 60){
        minutes = 0;
        hours = hours +1;
        if (hours == 24){
          days = days +1;
        } // end hrs check
      } // end minutes check
    } // end seconds check
  } // end time check
  Rdays = (tHours/24)- days;
  Rhours = tHours - hours - Rdays*24 - days*24;
  Rminutes = tHours*60 - minutes - 60*(tHours - hours - Rdays*24 - days*24);
  Rseconds = tHours*3600 - seconds - 60*(tHours*60 - minutes - 60*(tHours - hours - Rdays*24 - days*24));
}


void setUpTemp(){
  
}

void setUpTime(){
  
}

void adjustTemp(){
  
}

void adjustTime(){
  
}
/
/=============================================================================================
float analog2temp(float raw) {
    float celsius = 0;
    byte i;
    for (i=1; i<61; i++){
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw){                                              //  A linear (Y=mX+b)interpolation from the table's points:::
        celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) +                                         //  Y=b+
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) *                                          //  X
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /          //  m
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
        break;
      }
    }
    // Overflow: Set to last value in the table
    if (i == 61){
      celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);
    }
    return celsius;
}

float dataArray[ARRAY_LENGTH] = {};
short currentDataIndex = 0;
float smoothing (float raw){    //log the analog input value into an array, and output the average of the array
  currentDataIndex = ++currentDataIndex % ARRAY_LENGTH;    
  dataArray[currentDataIndex] = raw;

  double arrayTotal = 0;
  for(int i = 0; i<ARRAY_LENGTH; i++){
    arrayTotal = arrayTotal + dataArray[i];
  }
  
  double smoothCriminal = arrayTotal / ARRAY_LENGTH;
  
  return smoothCriminal;
}

//void bangBang(int tTemp, float ftemp){
//  if(tTemp > ftemp){
//    digitalWrite(HEATER_BED_PIN,HIGH);  
//  }else{
//    digitalWrite(HEATER_BED_PIN,LOW);
//  }
//}
