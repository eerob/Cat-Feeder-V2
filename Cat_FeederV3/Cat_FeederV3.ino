  #include "Arduino.h"
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <Time.h>

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address A4 (SDA), A5 (SCL)
//#include <Time.h>  
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

int button = 6;

bool feedSetting = 1;

int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B
int spead2 =255;//define the spead of motor 2

// Declare Feeding times ---------------------------------
int f1h = 5;  // feeding time 1 hours
int f1m = 1;  // feeding time 1 minutes
int f1s = 1;  // feeding time 1 seconds
int f2h = 13;  // feeding time 1 hours
int f2m = 10;  // feeding time 1 minutes
int f2s = 1;  // feeding time 1 seconds

unsigned long curTime;
unsigned long startTime;
unsigned long systemStartTime;

bool error = 1;
bool errLight = 0;
bool timeoutSetTime = 0;

void setup() {
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
  
  digitalWrite(button, HIGH); // turn on pull up resistor
  
  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight

// ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on  
  lcd.setCursor(0,1);
  lcd.print("   CROOKSHANKS V1");
  lcd.setCursor(0,2);
  lcd.print("   MEOW MEOW!");
  delay(2000);
  
  Serial.begin(9600);
  while (!Serial); // Needed for Leonardo only
  pinMode(13, OUTPUT);
  setSyncProvider(requestSync);  //set function to call when sync required
  lcd.clear();
  lcd.print("Waiting for sync msg");
 
  delay(1000);

  systemStartTime = millis();
  
}
//------------------------------------------------------------------------------
void loop () {
  if (millis() - systemStartTime > 30000 && timeoutSetTime == 0) {
    timeoutSetTime = 1;
    setTime(1450742400); 
  }
  
  if (Serial.available()) {
    timeoutSetTime = 1;
    processSyncMessage();
  }

  if (timeStatus()!= timeNotSet) {
    digitalClockDisplay(feedSetting);  
  }
  if (timeStatus() == 0) {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);  
    lcd.backlight();
    delay(250);
    
  } else {  // time not set blink LED
    digitalWrite(13, HIGH); // LED on if synced
  }
  //Serial.println("d6");
  //Serial.println(digitalRead(button));
  
  if(digitalRead(button) == 0) {
    startTime = millis();
    
    while(1) {
      if (digitalRead(button) == 0){
        curTime = millis();
      }

      else {

        if (curTime - startTime > 5000) {
          jog();
          break;
        }
        
        else if (curTime - startTime > 1000) {
          feedSetting = !feedSetting;
          break;
        }
        else
          break;
      }
    }
  }
    
  
  // If time is equal to feeding time and feedsetting = 0 then feed the damn cat once per day
  if ((hour() == f1h && minute() == f1m && second() == f1s && feedSetting == 0))
  {
    // ------- Quick 3 blinks of backlight  -------------
    for(int i = 0; i< 3; i++)
    {
      lcd.backlight();
      delay(250);
      lcd.noBacklight();
      delay(250);
      lcd.backlight();
      delay(250);
    }
    
    // Run Cat feeder
    jog();
    
  }

    // If time is equal to feeding time and feedsetting = 1 then feed the damn cat twice per day
  if ((hour() == f1h && minute() == f1m && second() == f1s && feedSetting == 1) || (hour() == f2h && minute() == f2m && second() == f2s && feedSetting == 1))
  {
    // ------- Quick 3 blinks of backlight  -------------
    for(int i = 0; i< 3; i++)
    {
      lcd.backlight();
      delay(250);
      lcd.noBacklight();
      delay(250);
      lcd.backlight();
      delay(250);
    }
    
    // Run Cat feeder
    jog();
    
  }
  
  delay(100);
}


void digitalClockDisplay(bool setting) {
  // digital clock display of the time
  delay(100);
  lcd.clear();
  
  if (feedSetting == 0) {
    lcd.print("Time: ");
    lcd.print(hour());
    lcd.print(":");
    lcd.print(minute());
    lcd.print(":");
    lcd.print(second()); 
    
    lcd.setCursor(0,1);
    lcd.print("Feed time 1 ");
    lcd.print(f1h);
    lcd.print(":");
    lcd.print(f1m);
    lcd.print(":");
    lcd.print(f1s); 
  }

  if (feedSetting == 1) {
    lcd.print("Time: ");
    lcd.print(hour());
    lcd.print(":");
    lcd.print(minute());
    lcd.print(":");
    lcd.print(second()); 
    
    lcd.setCursor(0,1);
    lcd.print("Feed time 1 ");
    lcd.print(f1h);
    lcd.print(":");
    lcd.print(f1m);
    lcd.print(":");
    lcd.print(f1s); 
    lcd.setCursor(0,2);
    lcd.print("Feed time 2 ");
    lcd.print(f2h);
    lcd.print(":");
    lcd.print(f2m);
    lcd.print(":");
    lcd.print(f2s); 
    }
  
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

//------------------------------------------------------------------------------

void forward()
{
     analogWrite(speedpinB,spead2);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
}

void backward()
{
     analogWrite(speedpinB,spead2);
     digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
     digitalWrite(pinI3,HIGH);
}

void stop()
{    // Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,LOW);
     delay(1000);
}

void jog() {
    
    startTime = millis();
    // go forward
    while(millis() - startTime < 1500) {
      forward();
    }
    stop();
}
