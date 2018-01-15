#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define TEMP_SENSOR_BED A0   //this defines the Arduino pin that is the bed's temp sensor
#define THERMISTORBED 1      //this defines what type of thermistor is used, and hence the voltage-temp table that is referenced. all in this build are a 100k thermistor with a 4.7k pull-up resistor.
#define HEATER_BED_PIN 6     //this defines the Arduino pin that controls the heater
#define buttonR 2
#define buttonU 4
#define buttonD 3

#define ARRAY_LENGTH 20 //Change this value to modulate the smoothing (can only work for all with smoothing)
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define countdown 25
unsigned long tHours = 72;
unsigned long tTemp  = 86;
unsigned long seconds  = 0;                 //used in timeCounter() and runningOLED()
unsigned long minutes  = 0;                 //used in timeCounter() and runningOLED()
unsigned long hours    = 0;                 //used in timeCounter() and runningOLED()
unsigned long days     = 0;                 //used in timeCounter() and runningOLED()
unsigned long Rdays    = tHours / 24;       //used in timeCounter() and runningOLED()
unsigned long Rhours   = tHours % 24;       //used in timeCounter() and runningOLED()
unsigned long Rminutes = Rhours % 60;       //used in timeCounter() and runningOLED()
unsigned long Rseconds = Rminutes % 60;     //used in timeCounter() and runningOLED()

unsigned long startTime = 0; //used as a master timer, and to protect from millis() rollover

void setup() {
  pinMode(TEMP_SENSOR_BED, INPUT);
  pinMode(HEATER_BED_PIN, OUTPUT);
  pinMode(buttonR, INPUT_PULLUP);
  pinMode(buttonU, INPUT_PULLUP);
  pinMode(buttonD, INPUT_PULLUP);
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

  setUpTemp();
  setUpTime();
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
    timeCounter();
    runningOLED(round(ftemp));
    //delay(10);    //delay for arduino stability, appears to not be needed...
    if(((millis() - startTime)/3600000)<tHours){
      bangBang(tTemp, ftemp); 
    }else{
      endHeating();
      done();
    }
    if(digitalRead(buttonR) == LOW){
      while(digitalRead(buttonR) == LOW){
        delay(750); //debounce time
        //wait to unpress
      }
      menu();
    }

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

unsigned long previousMillis = 0; //used in adjTemp() and adjTime()
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
    display.setCursor(0,24);         //can place text >> setCursor is top left of first char
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
    display.setCursor(0,16);         //can place text >> setCursor is top left of first char 
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

void timeCounter(){     //this is only for the onscreen timers, while running. will work fine without this. 
  unsigned long elapsed = (millis() - startTime);
  // all of the elapsed time, turned into DD:HH:MM:SS
  days     = elapsed / 1000 / (86400);  //3600*24
  hours    = elapsed / 1000 / 3600 % 24;
  minutes  = elapsed / 1000 % 3600 / 60;
  seconds  = elapsed / 1000 % 3600 % 60 % 60;
  // all of the remaining time, turned into DD:HH:MM:SS  
  
  //this timer still sucks....?
  Rdays    = ((tHours*3600) - (elapsed/1000)) / (86400);  //3600*24
  Rhours   = ((tHours*3600) - (elapsed/1000)) / 3600 % 24;
  Rminutes = ((tHours*3600) - (elapsed/1000)) % 3600 / 60;
  Rseconds = ((tHours*3600) - (elapsed/1000)) % 3600 % 60 % 60; 
}

void setUpTemp(){
  bool singlepress;
  while(1){
//onscreen things
  //temperature line
    display.setTextSize(2);           //'size 2' font is 16 pixels high
    display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
    display.setCursor(12,12);
    display.print(tTemp);
    display.println("F");
  //end temperature line
  //remaining time line
    display.setTextSize(1);          //'size 1' font is 8 pixels high
    display.setCursor(0,0);         //can place text >> setCursor is top left of first char
    display.print("Set Temp:");
    display.setCursor(112,0);         //can place text >> setCursor is top left of first char
    display.print("UP");
    display.setCursor(100,24);         //can place text >> setCursor is top left of first char
    display.print("DOWN");
  //end Elapsed timer line
    display.display();
    display.clearDisplay();
//wait for buttons
    while(digitalRead(buttonU) == HIGH && digitalRead(buttonD) == HIGH && digitalRead(buttonR) == HIGH ){
      singlepress = true;
      //do nothing...
    }
//button responses
    if((digitalRead(buttonU) == LOW) && (singlepress)){
      tTemp = tTemp +1;   // Raise temp with UP button
      singlepress = false;
    }
    if((digitalRead(buttonD) == LOW) && (singlepress)){
      tTemp = tTemp -1;   // Lower temp with DOWN button
      singlepress = false;
    }
    if(digitalRead(buttonR) == LOW){
      while(digitalRead(buttonR) == LOW){
        delay(750); //debounce time
        //wait to unpress
      }
      return;             // Exit temp set-up with Right button
    }
  }
}

void setUpTime(){
  bool singlepress;
  while(1){
//onscreen things
  //temperature line
    display.setTextSize(2);           //'size 2' font is 16 pixels high
    display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
    display.setCursor(6,12);
    display.print(tHours);
    display.println("hrs");
  //end temperature line
  //remaining time line
    display.setTextSize(1);          //'size 1' font is 8 pixels high
    display.setCursor(0,0);         //can place text >> setCursor is top left of first char
    display.print("Set Time:");
    display.setCursor(112,0);         //can place text >> setCursor is top left of first char
    display.print("UP");
    display.setCursor(100,24);         //can place text >> setCursor is top left of first char
    display.print("DOWN");
  //end Elapsed timer line
    display.display();
    display.clearDisplay();
//wait for buttons
    while(digitalRead(buttonU) == HIGH && digitalRead(buttonD) == HIGH && digitalRead(buttonR) == HIGH ){
      singlepress = true;
      //do nothing...
    }
//button responses
    if((digitalRead(buttonU) == LOW) && (singlepress)){
      tHours = tHours +1;   // Raise temp with UP button
      singlepress = false;
    }
    if((digitalRead(buttonD) == LOW) && (singlepress)){
      tHours = tHours -1;   // Lower temp with DOWN button
      singlepress = false;
    }
    if(digitalRead(buttonR) == LOW){
      while(digitalRead(buttonR) == LOW){
        delay(1500); //debounce time
        startTime = 0;
        //wait to unpress
      }
      if(48 < (tHours/24)) tooLongTimeError(); 
      return;             // Exit temp set-up with Right button
    }
  }
}

void menu(){
  bool singlepress;
  while(1){
//onscreen things
    display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
    display.setTextSize(1);          //'size 1' font is 8 pixels high
    display.setCursor(50,0);         //can place text >> setCursor is top left of first char
    display.print("Adjust temp");
    display.setCursor(50,24);         //can place text >> setCursor is top left of first char
    display.print("Adjust time");
  //end Elapsed timer line
    display.display();
    display.clearDisplay();
//wait for buttons
    while(digitalRead(buttonU) == HIGH && digitalRead(buttonD) == HIGH && digitalRead(buttonR) == HIGH ){
      singlepress = true;
      //do nothing...
    }
//button responses
    if(digitalRead(buttonU) == LOW && singlepress){
      previousMillis = millis();
      adjustTemp();   // Raise temp with UP button
      return;
    }
    if(digitalRead(buttonD) == LOW && singlepress){
      previousMillis = millis();
      adjustTime();   // Lower temp with DOWN button
      return;
    }
    if(digitalRead(buttonR) == LOW){
      while(digitalRead(buttonR) == LOW){
        delay(750); //debounce time
        //wait to unpress
      }
      return;             // Exit temp set-up with Right button
    }
  }
}


void adjustTemp(){
  bool singlepress;
  while(1){
  //wait for buttons
    while(digitalRead(buttonU) == HIGH && digitalRead(buttonD) == HIGH && digitalRead(buttonR) == HIGH ){
      singlepress = true;
      //countdown timer
  //calculate remaining time
    int remaining = (countdown*1000 - (millis()-previousMillis))/1000;
    if(remaining < 0) return;
  //onscreen things
    //temperature line
      display.setTextSize(2);           //'size 2' font is 16 pixels high
      display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
      display.setCursor(25,12);
      display.print(tTemp);
      display.println("F");
    //end temperature line
    //remaining time line
      display.setTextSize(1);          //'size 1' font is 8 pixels high
      display.setCursor(0,0);         //can place text >> setCursor is top left of first char
      display.print("Set Temp:");
      display.setCursor(112,0);         //can place text >> setCursor is top left of first char
      display.print("UP");
      display.setCursor(100,24);         //can place text >> setCursor is top left of first char
      display.print("DOWN");
      display.setCursor(0,24);         //can place text >> setCursor is top left of first char
      display.print(remaining);
      display.print("s...");    
    //end Elapsed timer line
      display.display();
      display.clearDisplay();
    }
//button responses
    if((digitalRead(buttonU) == LOW) && (singlepress)){
      tTemp = tTemp +1;   // Raise temp with UP button
      singlepress = false;
    }
    if((digitalRead(buttonD) == LOW) && (singlepress)){
      tTemp = tTemp -1;   // Lower temp with DOWN button
      singlepress = false;
    }
    if((digitalRead(buttonR) == LOW) && (singlepress)){  
      while(digitalRead(buttonR) == LOW){
        delay(750); //debounce time
        //wait to unpress
      }
      return;             // Exit temp set-up with Right button
    }
  }
}

void adjustTime(){
  bool singlepress;
  int adjTime = 0;
  while(1){
//wait for buttons
    while(digitalRead(buttonU) == HIGH && digitalRead(buttonD) == HIGH && digitalRead(buttonR) == HIGH ){
      singlepress = true;
  //countdown timer
  //calculate remaining time
    int remaining = (countdown*1000 - (millis()-previousMillis))/1000;
    if(remaining < 0) return;
  //onscreen things
    //temperature line
      display.setTextSize(2);           //'size 2' font is 16 pixels high
      display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
      display.setCursor(25,8);
      if(adjTime > 0) display.print("+");
      display.print(adjTime);
      display.print("hrs");
    //end temperature line
    //remaining time line
      display.setTextSize(1);          //'size 1' font is 8 pixels high
      display.setCursor(0,0);         //can place text >> setCursor is top left of first char
      display.print("Adjust time:");
      display.setCursor(0,24);         //can place text >> setCursor is top left of first char
      display.print(remaining);
      display.print("s...");    
    //end Elapsed timer line
      display.display();
      display.clearDisplay();
    }
//button responses
    if((digitalRead(buttonU) == LOW) && (singlepress)){
      adjTime = adjTime +1;   // Raise temp with UP button
      singlepress = false;
    }
    if((digitalRead(buttonD) == LOW) && (singlepress)){
      adjTime = adjTime -1;   // Lower temp with DOWN button
      singlepress = false;
    }
    if((digitalRead(buttonR) == LOW) && (singlepress)){  
      while(digitalRead(buttonR) == LOW){
        delay(750); //debounce time
        if(48 < (tHours + adjTime)/24){
          tooLongTimeError(); 
        }else{
          tHours = tHours + adjTime;    
        }
        //wait to unpress
      }
      return;             // Exit temp set-up with Right button
    }
  }
}

void tooLongTimeError(){
    //onscreen things
    //temperature line
      display.setTextSize(2);           //'size 2' font is 16 pixels high
      display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
      display.setCursor(25,8);
      display.print("ERROR!! :(");
    //end temperature line
    //remaining time line
      display.setTextSize(1);          //'size 1' font is 8 pixels high
      display.setCursor(0,0);         //can place text >> setCursor is top left of first char
      display.print("Time Exceeds Max Limit");
      display.setCursor(0,24);         //can place text >> setCursor is top left of first char
      display.print("Going to default 72hrs");
      tHours = 72;    // actually change the target time to 72 hrs...    
    //end Elapsed timer line
      display.display();
      display.clearDisplay();
      delay(3000);
}

void done(){
    //onscreen things
    //temperature line
      display.setTextSize(2);           //'size 2' font is 16 pixels high
      display.setTextColor(WHITE);      //can place text >> setCursor is top left of first char
      display.setCursor(25,8);
      display.print("Done.");
    //end temperature line
    //remaining time line
      display.setTextSize(1);          //'size 1' font is 8 pixels high
      display.setCursor(0,0);         //can place text >> setCursor is top left of first char
      display.print("Time: ");
      display.print(tHours);
      display.print("  Temp: ");
      display.print(tTemp);
      display.setCursor(0,24);         //can place text >> setCursor is top left of first char
      display.print("Enjoy your tempeh :)");
      display.display();
      display.clearDisplay();
      while(1){
        endHeating();
      }
}

//=============================================================================================
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

void endHeating(){
  digitalWrite(HEATER_BED_PIN,LOW);
}

void bangBang(int tTemp, float ftemp){
  if(tTemp > ftemp){
    digitalWrite(HEATER_BED_PIN,HIGH);  
  }else{
    digitalWrite(HEATER_BED_PIN,LOW);
  }
}
