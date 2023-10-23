//
// Arduino code for the six (Nixie) digit clock board by Paolo Cravero IK1ZYW rev. 2023-05
// 
// IMPLEMENTED:
//  - Fast PWM generator *WITHOUT* feedback loop
//  - DS3231 RTC integration
//    - lost battery sense and error display (ERROR 9)
//    - hh:mm:ss time display
//  - HH zero blanking
//  - Depoison function with some settings
//  - Error display function (99___x)

// TO BE DONE and PLANNED:
//  - RTC set, date and time
//  - day and night mode
//  - brightness control via PWM adjust
//  - LDR integration for brightness control
//  - blinking or static colons 


// CREDITS
// Code of the HV generator by Ian Sparkes from https://bitbucket.org/isparkes/nixiefirmwarev1/src/v56/ardunixFade9_6_digit/ardunixFade9_6_digit.ino
// Also other subroutines taken or inspired by Ian Sparkes's code


#include <avr/io.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>
#include <digitalWriteFast.h>
#include "button.h"

#define EE_PULSE_LO           12     // The pulse on width for the PWM mode
#define EE_PULSE_HI           13     // The pulse on width for the PWM mode
#define EE_HV_VOLTAGE         22     // The HV voltage we want to use
#define EE_PWM_TOP_LO         27     // The PWM top value if we know it, 0xFF if we need to calculate
#define EE_PWM_TOP_HI         28     // The PWM top value if we know it, 0xFF if we need to calculate
#define EE_HVG_NEED_CALIB     29     // 1 if we need to calibrate the HVGenerator, otherwise 0
#define EE_NEED_SETUP         33     // used for detecting auto config for startup. By default the flashed, empty EEPROM shows us we need to do a setup 


// The target voltage we want to achieve
#define HVGEN_TARGET_VOLTAGE_DEFAULT 180
#define HVGEN_TARGET_VOLTAGE_MIN     150
#define HVGEN_TARGET_VOLTAGE_MAX     200

// The PWM parameters
//#define PWM_TOP_DEFAULT   800 // for 11V input
//#define PWM_PULSE_DEFAULT 50  // for 11V input
#define PWM_TOP_DEFAULT   550 // for 5V input
#define PWM_PULSE_DEFAULT 150  // for 5V input

#define PWM_TOP_MIN       200
#define PWM_TOP_MAX       10000
#define PWM_PULSE_MIN     20
#define PWM_PULSE_MAX     150
#define PWM_OFF_MIN       50


// ********************** HV generator variables *********************
int hvTargetVoltage = HVGEN_TARGET_VOLTAGE_DEFAULT;
int pwmTop = PWM_TOP_DEFAULT;
int pwmOn = PWM_PULSE_DEFAULT;

// precalculated values for turning on and off the HV generator
// Put these in TCCR1B to turn off and on
byte tccrOff;
byte tccrOn;
int rawHVADCThreshold;
double sensorHVSmoothed = 0;

// ***** Pin Definitions *****
#define HVDRIVERPIN 9      // package pin 15 // PB1 // PWM pin used to drive the DC-DC converter
#define HVSENSE     A0     // Package pin 23 // PC0 // Analog input pin for HV sense: HV divided through 350k and 4k7 divider, using 5V reference
#define ANODE1PIN    0     // package pin 2  // PD0 // output to the optocoupler // active HIGH
#define ANODE2PIN    1     // package pin 3  // PD1 // output to the optocoupler // active HIGH
#define ANODE3PIN    3     // package pin 5  // PD3 // output to the optocoupler // active HIGH
#define ANODE4PIN    17    // package pin 26 // PC3 // output to the optocoupler // active HIGH
#define ANODE5PIN    4     // package pin 6  // PD4 // output to the optocoupler // active HIGH
#define ANODE6PIN    16    // package pin 25 // PC2 // output to the optocoupler // active HIGH
#define BCD1PIN      5     // package pin 11 // PD5 // output to 74141
#define BCD2PIN      7     // package pin 13 // PD7 // output to 74141
#define BCD3PIN      8     // package pin 14 // PB0 // output to 74141
#define BCD4PIN      6     // package pin 12 // PD6 // output to 74141
#define COLONCTRL    10    // package pin 16 // PB2 // output to NPN transistor // active HIGH
#define BUTTON1      13    // package pin 19 // PB5 // SCK  // UPPER button
#define BUTTON2      12    // package pin 18 // PB4 // MISO // LOWER button
#define LDR          A1    // package pin 24 // PC1 // Light Dependant Resistor input for brightness
#define oneSecondInterruptPin 2  // package pin 4 // PD2 // 1 Hz input from RTC


// ******** GLOBAL VARIABLES *********
int secondsElapsed = 0;
int oldSecondsElapsed = 0;
unsigned long nowMillis = 0;
RTC_DS3231 rtc;
DateTime RTCnow;
byte seconds;
byte minutes;
byte hours;
byte weekday;
byte month_day;
byte month_nr;
byte year_nr;



// ******* Display handling ******* sourced from ardunix
#define DIGIT_DISPLAY_COUNT   400 //1000 // The number of times to traverse inner fade loop per digit
#define DIGIT_DISPLAY_ON      999    // Switch on the digit at the beginning by default
#define DIGIT_DISPLAY_OFF     0  // Switch off the digit at the end by default
#define DIGIT_DISPLAY_NEVER   -1   // When we don't want to switch on or off (i.e. blanking)
#define DISPLAY_COUNT_MAX     2000 // Maximum value we can set to
#define DISPLAY_COUNT_MIN     500  // Minimum value we can set to
#define DEPOISON_PER_DIGIT_MILLIS 30 // how long to keep each digit ON during the depoison routine
#define DEPOISON_PER_DIGIT_LOOPS 5  // how many times iterate all numbers on each tube. Total time = DEPOISON_PER_DIGIT_MILLIS * DEPOISON_PER_DIGIT_LOOPS

#define MIN_DIM_DEFAULT 100         // The default minimum dim count
#define MIN_DIM_MIN     100         // The minimum dim count
#define MIN_DIM_MAX     500         // The maximum dim count

#define ZERO_BLANK_TEN_HOURS true
#define ZERO_BLANK_UNIT_HOURS true

// ********* DISPLAY GLOBAL VARIABLE ********
byte NumberArray[6]    = {0,0,0,0,0,0};  // startup value, all zeros. REMEMBER array is LSD-to-MSD
byte DateArray[6]    = {0,0,0,0,0,0};  // startup value, all zeros. REMEMBER array is LSD-to-MSD
unsigned int DurationArray[6]  = {DIGIT_DISPLAY_COUNT, DIGIT_DISPLAY_COUNT, DIGIT_DISPLAY_COUNT, DIGIT_DISPLAY_COUNT, DIGIT_DISPLAY_COUNT, DIGIT_DISPLAY_COUNT}; // REMEMBER array is LSD-to-MSD
byte OnOffMaskArray[6] = {1, 1, 1, 1, 1, 1};  // takes 1 = ON or 0 = OFF



Button button1;
Button button2;


// ******** PROTOTYPES *********
void blinkDigit(byte digit, byte times = 4);
int ordered[10] = {1,5,0,2,9,6,7,4,8,3};

void setup() {
  // put your setup code here, to run once:

  // *** PIN SETUP ***
  // outputs to 74141 and DP transistors
  pinModeFast( BCD1PIN, OUTPUT); digitalWriteFast( BCD1PIN, LOW);
  pinModeFast( BCD2PIN, OUTPUT); digitalWriteFast( BCD2PIN, LOW);
  pinModeFast( BCD3PIN, OUTPUT); digitalWriteFast( BCD3PIN, LOW);
  pinModeFast( BCD4PIN, OUTPUT); digitalWriteFast( BCD4PIN, LOW);
  
  pinModeFast(ANODE1PIN, OUTPUT); digitalWriteFast(ANODE1PIN, LOW);
  pinModeFast(ANODE2PIN, OUTPUT); digitalWriteFast(ANODE2PIN, LOW);
  pinModeFast(ANODE3PIN, OUTPUT); digitalWriteFast(ANODE3PIN, LOW);
  pinModeFast(ANODE4PIN, OUTPUT); digitalWriteFast(ANODE4PIN, LOW);
  pinModeFast(ANODE5PIN, OUTPUT); digitalWriteFast(ANODE5PIN, LOW);
  pinModeFast(ANODE6PIN, OUTPUT); digitalWriteFast(ANODE6PIN, LOW);

  pinModeFast(COLONCTRL, OUTPUT); digitalWriteFast(COLONCTRL, LOW);
  
  (oneSecondInterruptPin, INPUT);

  pinModeFast(LDR, INPUT);
  
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  button1.begin(BUTTON1);
  button2.begin(BUTTON2);

  pinMode(HVSENSE, INPUT);

  pinMode(HVDRIVERPIN, OUTPUT);
//  digitalWrite(HVDRIVERPIN, LOW); // safety, stay off

  /* disable global interrupts while we set up them up */
  cli();

  // **************************** HV generator ****************************

  TCCR1A = 0;    // disable all PWM on Timer1 whilst we set it up
  TCCR1B = 0;    // disable all PWM on Timer1 whilst we set it up

  // Configure timer 1 for Fast PWM mode via ICR1, with prescaling=1
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  tccrOff = TCCR1A;

  TCCR1A |= (1 <<  COM1A1);  // enable PWM on port PD4 in non-inverted compare mode 2

  tccrOn = TCCR1A;

  // we don't need the HV yet, so turn it off
  TCCR1A = tccrOff;

  /* enable global interrupts */
  sei();


  // TODO: read buttons and do a hard reset if they are both pressed.

  // Read EEPROM values
  //readEEPROMValues();
  pwmTop = EEPROM.read(EE_PWM_TOP_HI) * 256 + EEPROM.read(EE_PWM_TOP_LO);
  if ((pwmTop < PWM_TOP_MIN) || (pwmTop > PWM_TOP_MAX)) {
    pwmTop = PWM_TOP_DEFAULT;
  }
  
  // set our PWM profile
  setPWMOnTime(pwmOn);
  setPWMTopTime(pwmTop);

  // if both buttons are pressed at startup, play with PWM top
  if (( digitalRead(BUTTON1) == LOW ) && ( digitalRead(BUTTON2) == LOW )) {
    set74141(8);
    
    digitalWriteFast(ANODE1PIN, HIGH);
    TCCR1A = tccrOn;
    unsigned long nowMillis = millis();
    do {
      if (button1.debounce()) {
        pwmTop += 50;
        setPWMTopTime(pwmTop);
        nowMillis = millis();
      }
      if (button2.debounce()) {
        pwmTop -= 50;
        setPWMTopTime(pwmTop);
        nowMillis = millis();
      }
    } while ( ( (millis() - nowMillis) < 5000) );  // wait 5 seconds of nothing changing to save to EEPROM
    EEPROM.write(EE_PWM_TOP_LO, pwmTop % 256);
    EEPROM.write(EE_PWM_TOP_HI, pwmTop / 256);
    setPWMTopTime(pwmTop);  // happily use the new value
  }

  // Set the target voltage
  rawHVADCThreshold = getRawHVADCThreshold(hvTargetVoltage);

  // HV GOOOO!!!!
  TCCR1A = tccrOn;

//  if (EEPROM.read(EE_HVG_NEED_CALIB)) {
//    calibrateHVG();
//
//    // Save the PWM values
//    EEPROM.write(EE_PULSE_LO, pwmOn % 256);
//    EEPROM.write(EE_PULSE_HI, pwmOn / 256);
//    EEPROM.write(EE_PWM_TOP_LO, pwmTop % 256);
//    EEPROM.write(EE_PWM_TOP_HI, pwmTop / 256);
//
//    // Mark that we don't need to do this next time
//    EEPROM.write(EE_HVG_NEED_CALIB, false);
//  }

//  // and return it to target voltage so we can regulate the PWM on time
  rawHVADCThreshold = getRawHVADCThreshold(hvTargetVoltage);  
//  
//  // mark that we have done the EEPROM setup
//  EEPROM.write(EE_NEED_SETUP, false);

  // say Hello!
  // TODO bootup sequence

// TODO TORE-ENABLE this works when there is an RTC connected
// Initialize I2C RTC
  if (! rtc.begin()) {
    IamStuck(0, true); // ERROR CODE 0 for RTC not present, blocking
  }

  // if a new battery was installed in the RTC or the operator requests a software reset...
  if (rtc.lostPower() || digitalRead(BUTTON2) == LOW) { // not debouncing the button
//    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtc.adjust(DateTime(2023, 06, 10, 0, 0, 0));  // reset the date and time 
    IamStuck(9, false); // ERROR CODE 9 for RTC was reset, need new battery, not blocking
  }

  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

  // End Init I2C RTC

  RTCnow = rtc.now();
  seconds = RTCnow.second();
  secondsElapsed = 60;
  randomSeed(seconds+RTCnow.day()+RTCnow.month()); // gathering some entropy for future use

  attachInterrupt(digitalPinToInterrupt(oneSecondInterruptPin), oneSecondISR, FALLING);
  
} // end setup()

void loop() {

//UNUSED  static int currTemp = 0;
  static unsigned long oldMillis = 0; // it's always useful ;)
  static unsigned long pauseMillis = 0; // it's always useful ;)
  static unsigned long digitMillis = 0; // how long a digit stays on
  static unsigned long nowSeconds = 0;
  static boolean loadDateDone = 0;
  static boolean zeroBlankTenHours = ZERO_BLANK_TEN_HOURS;
  static boolean zeroBlankUnitHours = ZERO_BLANK_UNIT_HOURS;
  static word myDelay = 1000;
  static word myPause = 500;
  int newHours;
  int newMinutes;
  int newDay;
  int newMonth;
  int newYear;
  int newDoW;
  int tempValue;
  int oldValue;
  int maxDayOfMonth;

//  checkHVVoltage();

//  nowMillis = millis();

  // Check button, we evaluate below

  // Enter the time-set mode
  // *************** BEGIN OF SET MODE ************
  // This part of code uses the Date display array so that the
  // ISR does not pollute the seconds digits.
  if (button1.debounce()) {
      newHours = RTCnow.hour();
      newMinutes = RTCnow.minute();
      newDay = RTCnow.day();
      newMonth = RTCnow.month();
      newYear = RTCnow.year() % 100;
      newDoW = RTCnow.dayOfTheWeek();

      // just MM are ON
      OnOffMaskArray[0] = 0;
      OnOffMaskArray[1] = 0;
      OnOffMaskArray[2] = 1;
      OnOffMaskArray[3] = 1;
      OnOffMaskArray[4] = 0;
      OnOffMaskArray[5] = 0;
      do {
        DateArray[2] = newMinutes%10;
        DateArray[3] = newMinutes/10;
        DateArray[4] = newHours%10;
        DateArray[5] = newHours/10;
        doDisplayDate();
        if (button2.debounce()) {
          newMinutes = (newMinutes+1)% 60;
        }
      } while (!button1.debounce());

      // just HH MM are ON
      OnOffMaskArray[0] = 0;
      OnOffMaskArray[1] = 0;
      OnOffMaskArray[2] = 1;
      OnOffMaskArray[3] = 1;
      OnOffMaskArray[4] = 1;
      OnOffMaskArray[5] = 1;
      do {
        DateArray[2] = newMinutes%10;
        DateArray[3] = newMinutes/10;
        DateArray[4] = newHours%10;
        DateArray[5] = newHours/10;
        doDisplayDate();
        if (button2.debounce()) {
          newHours = (newHours+1) % 24; // sorry, 24h format here!
        }
      } while (!button1.debounce());

      // Year
      OnOffMaskArray[0] = 1;
      OnOffMaskArray[1] = 1;
      OnOffMaskArray[2] = 0;
      OnOffMaskArray[3] = 0;
      OnOffMaskArray[4] = 0;
      OnOffMaskArray[5] = 0;
      do {
        DateArray[0] = newYear%10;
        DateArray[1] = newYear/10;
        DateArray[2] = newMonth%10;
        DateArray[3] = newMonth/10;
        DateArray[4] = newDay%10;
        DateArray[5] = newDay/10;
        doDisplayDate();
        if (button2.debounce()) {
          newYear = (newYear+1) % 100; // year from 2000 to 2099
        }
      } while (!button1.debounce());

      // Month
      OnOffMaskArray[0] = 1;
      OnOffMaskArray[1] = 1;
      OnOffMaskArray[2] = 1;
      OnOffMaskArray[3] = 1;
      OnOffMaskArray[4] = 0;
      OnOffMaskArray[5] = 0;
      do {
        DateArray[0] = newYear%10;
        DateArray[1] = newYear/10;
        DateArray[2] = newMonth%10;
        DateArray[3] = newMonth/10;
        DateArray[4] = newDay%10;
        DateArray[5] = newDay/10;
        doDisplayDate();
        if (button2.debounce()) {
          newMonth = (newMonth+1) % 13; // month from 1 to 12
          if (newMonth == 0) { newMonth = 1; }
        }
      } while (!button1.debounce());

      // set max days in the month
      // It is "+1" because of the % operation
      switch (newMonth) {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
          maxDayOfMonth = 32;
          break;
        case 4:
        case 6:
        case 9:
        case 11:
          maxDayOfMonth = 31;
          break;
        case 2: // alright, please don't set the time on THAT DAY of leap years!
          maxDayOfMonth = 29;
          break;
      }
      
      // Day
      allOn();
      do {
        DateArray[0] = newYear%10;
        DateArray[1] = newYear/10;
        DateArray[2] = newMonth%10;
        DateArray[3] = newMonth/10;
        DateArray[4] = newDay%10;
        DateArray[5] = newDay/10;
        doDisplayDate();
        if (button2.debounce()) {
          newDay = (newDay+1) % maxDayOfMonth; // sorry, 24h format here!
        }
        if (newDay == 0) { newDay = 1; };
      } while (!button1.debounce());
                  

    rtc.adjust(DateTime((2000+newYear), newMonth, newDay, newHours, newMinutes, 00));
    seconds = 0;
    secondsElapsed = 60; // so we read back the current timedate
    delay(300);
    allOn();
    
  } // end of time set routine!
  // *************** END OF SET MODE ************

  // *************** BEGIN OF DATE MODE ************
  if (button2.debounce()) {
      DateArray[4] = RTCnow.day()%10;
      DateArray[5] = RTCnow.day()/10;
      DateArray[2] = RTCnow.month()%10;
      DateArray[3] = RTCnow.month()/10;
      DateArray[0] = (RTCnow.year()-2000)%10;
      DateArray[1] = (RTCnow.year()-2000)/10;

      allOn();
      delay(300);
      do {
        doDisplayDate();        
      } while (!button2.debounce());
      secondsElapsed=60;
      allOff();
      delay(300);
  }
  // *************** END OF DATE MODE ************

  
  
//  // handle loop of 60 seconds (0..59)
  if (secondsElapsed > 59) {
    RTCnow = rtc.now();
    secondsElapsed = RTCnow.second();
    minutes = RTCnow.minute();
    hours = RTCnow.hour();

    NumberArray[0] = secondsElapsed%10;
    NumberArray[1] = secondsElapsed/10;
    NumberArray[2] = minutes%10;
    NumberArray[3] = minutes/10;
    NumberArray[4] = hours%10;
    NumberArray[5] = hours/10;
    allOn();
    if ((zeroBlankTenHours==true)&&(NumberArray[5]==0)) {
      OnOffMaskArray[5]=0;
      if ((zeroBlankUnitHours==true)&&(NumberArray[4]==0)) {
        OnOffMaskArray[4]=0;
      } else {
        OnOffMaskArray[4]=1;
      }
    } else {
      OnOffMaskArray[5]=1;
    }
    //UNUSED currTemp = 0; // prepare to read and display again the temperature
    loadDateDone = 0;
  } // end if secondsElapsed>59
    //fadeSpeed = random(1, 11) * 10; // randomize new fading timing

  // do a depoison routine at xx:x0:12
  if ((NumberArray[2]==0)&&(secondsElapsed==12)) {
    doDePoison();
    // repeating this code here. should be moved into the display routine?
    if ((zeroBlankTenHours==true)&&(NumberArray[5]==0)) {
      OnOffMaskArray[5]=0;
      if ((zeroBlankUnitHours==true)&&(NumberArray[4]==0)) {
        OnOffMaskArray[4]=0;
      } else {
        OnOffMaskArray[4]=1;
      }
    } else {
      OnOffMaskArray[5]=1;
    }
  }


  /* Read from RTC and display the rounded temperature. INCOMPLETE. */
  /* Does work, but the value is too much off the real temperature. */
//  if (secondsElapsed==20) {
//    if (currTemp == 0) { // we've not updated the temperature value, just yet
//      currTemp = round(rtc.getTemperature());
//    NumberArray[2] = currTemp%10;
//    NumberArray[3] = currTemp/10;
//    allOff();
//    OnOffMaskArray[2] = 1;
//    OnOffMaskArray[3] = 1;
//    }
//  }

//  if (secondsElapsed > 56) {
//    if (loadDateDone == 0) {
//      NumberArray[4] = RTCnow.day()%10;
//      NumberArray[5] = RTCnow.day()/10;
//      NumberArray[2] = RTCnow.month()%10;
//      NumberArray[3] = RTCnow.month()/10;
//      NumberArray[0] = (RTCnow.year()-2000)%10;
//      NumberArray[1] = (RTCnow.year()-2000)/10;
//      loadDateDone = 1;
//    }
//    if (secondsElapsed == 57) {
//      allOff();
//      OnOffMaskArray[5] = 1;
//      OnOffMaskArray[4] = 1;
//    }
//    if (secondsElapsed == 58) {
//      allOff();
//      OnOffMaskArray[3] = 1;
//      OnOffMaskArray[2] = 1;
//    }
//    if (secondsElapsed == 59) {
//      allOff();
//      OnOffMaskArray[1] = 1;
//      OnOffMaskArray[0] = 1;
//    }    
//  } else {

//    moved into the ISR
//    NumberArray[0] = secondsElapsed%10;
//    NumberArray[1] = secondsElapsed/10;

//  }
  
  doDisplay();
  
  
} // end main loop


void doDisplay() {

  // go through each tube
  // DIGIT 0 is RIGHTmost!
  // DIGIT 5 is LEFTmost!
  for (byte digit=0; digit<6; digit++) {
    // stay on each tube a DIGIT_DISPLAY_COUNT time or whatever
    // is written in the DurationArray for the digit.
    for (unsigned int impression=0; impression<(DurationArray[digit] * OnOffMaskArray[digit]); impression++) {
      nixieOn(digit);        
    }
    nixieOff();
  }
  

}

void doDisplayDate() {

  // go through each tube
  // DIGIT 0 is RIGHTmost!
  // DIGIT 5 is LEFTmost!
  for (byte digit=0; digit<6; digit++) {
    // stay on each tube a DIGIT_DISPLAY_COUNT time or whatever
    // is written in the DurationArray for the digit.
    for (unsigned int impression=0; impression<(DurationArray[digit] * OnOffMaskArray[digit]); impression++) {
      nixieOnDate(digit);        
    }
    nixieOff();
  }
  

}

void doDePoison() {

  unsigned long lnow;

  byte ArrayBackup[6];

  ArrayBackup[0] = NumberArray[0];
  ArrayBackup[1] = NumberArray[1];
  ArrayBackup[2] = NumberArray[2];
  ArrayBackup[3] = NumberArray[3];
  ArrayBackup[4] = NumberArray[4];
  ArrayBackup[5] = NumberArray[5];

  for (int y=5;y>=0;y--) { // digit position LtR
    for (int w=0;w<DEPOISON_PER_DIGIT_LOOPS;w++) {
      for (int x=0;x<10;x++) {  // digit value 0-9
        NumberArray[0] = x;  
        NumberArray[1] = x;
        NumberArray[2] = x;
        NumberArray[3] = x;
        NumberArray[4] = x;
        NumberArray[5] = x;
        allOff();
        OnOffMaskArray[y] = 1;
        lnow = millis();
        do {
          doDisplay();
        } while ((millis() - lnow) < DEPOISON_PER_DIGIT_MILLIS);
      }
    }
  }

  NumberArray[5] = ArrayBackup[5];
  NumberArray[4] = ArrayBackup[4];
  NumberArray[3] = ArrayBackup[3];
  NumberArray[2] = ArrayBackup[2];
  NumberArray[1] = ArrayBackup[1];
  NumberArray[0] = ArrayBackup[0];
  
  allOn();

}


void allOff() {
  OnOffMaskArray[0] = 0;
  OnOffMaskArray[1] = 0;
  OnOffMaskArray[2] = 0;
  OnOffMaskArray[3] = 0;
  OnOffMaskArray[4] = 0;
  OnOffMaskArray[5] = 0;
}

void allOn() {
  OnOffMaskArray[0] = 1;
  OnOffMaskArray[1] = 1;
  OnOffMaskArray[2] = 1;
  OnOffMaskArray[3] = 1;
  OnOffMaskArray[4] = 1;
  OnOffMaskArray[5] = 1;
}
// A black hole that displays all numbers and persists
// on one that is the error message.
// "blocking" can be 1, it never ends, or 0, it does it just once
// *******************
// ERROR CODES TABLE
// CODE 0: RTC not connected, blocking
// CODE 9: RTC battery, non blocking
// *******************
void IamStuck(int errorCode, byte blocking) {
  
  byte repeat = 4;

  do {
    NumberArray[0] = errorCode;  // Error code is shown as 99___X
    NumberArray[1] = 0;
    NumberArray[2] = 0;
    NumberArray[3] = 0;
    NumberArray[4] = 9;
    NumberArray[5] = 9;
    OnOffMaskArray[0] = 1;
    OnOffMaskArray[1] = 0;
    OnOffMaskArray[2] = 0;
    OnOffMaskArray[3] = 0;
    OnOffMaskArray[4] = 1;
    OnOffMaskArray[5] = 1;

    unsigned long lnow = millis();
    do {
      doDisplay();
      if ((millis() - lnow) > 700) {
        OnOffMaskArray[0] = 0;
        OnOffMaskArray[4] = 0;
        OnOffMaskArray[5] = 0;
      }
      if ((millis() - lnow) > 1000) {
        repeat--;
        lnow = millis();
        OnOffMaskArray[0] = 1;
        OnOffMaskArray[4] = 1;
        OnOffMaskArray[5] = 1;
      }
    } while (repeat > 0);
  } while (blocking);
  
  OnOffMaskArray[0] = 1;
  OnOffMaskArray[4] = 1;
  OnOffMaskArray[5] = 1; 
}




// ******************************************************************
// Calibrate the HV generator
// The idea here is to get the right combination of PWM on and top
// time to provide the right high voltage with the minimum power
// Consumption.
//
// Every combination of tubes and external power supply is different
// and we need to pick the right PWM total duration ("top") and
// PWM on time ("on") to match the power supply and tubes.
// Once we pick the "on" time, it is not adjusted during run time.
// PWM top is adjusted during run.
//
// The PWM on time is picked so that we reach just the point that the
// inductor goes into saturation - any more time on is just being used
// to heat the MOSFET and the inductor, but not provide any voltage.
//
// We go through two cycles: each time we decrease the PWM top
// (increase frequency) to give more voltage, then reduce PWM on
// until we notice a drop in voltage.
// ******************************************************************
void calibrateHVG() {

  // *************** first pass - get approximate frequency *************
  rawHVADCThreshold = getRawHVADCThreshold(hvTargetVoltage + 5);

  setPWMOnTime(PWM_PULSE_DEFAULT);
  // Calibrate HVGen at full
  for (int i = 0 ; i < 768 ; i++ ) {
//    loadNumberArraySameValue(8);
//    allBright();
//    outputDisplay();
    checkHVVoltage();
  }

  // *************** second pass - get on time minimum *************
  rawHVADCThreshold = getRawHVADCThreshold(hvTargetVoltage);

  // run up the on time from the minimum to where we reach the required voltage
  setPWMOnTime(PWM_PULSE_MIN);
  for (int i = 0 ; i < 768 ; i++ ) {
    outputDisplay();

    if (getSmoothedHVSensorReading() < rawHVADCThreshold) {
      if ((i % 8) == 0 ) {
        incPWMOnTime();
      }
    }
  }

  int bottomOnValue = pwmOn;

  // *************** third pass - get on time maximum *************
  setPWMOnTime(pwmOn + 50);
  for (int i = 0 ; i < 768 ; i++ ) {
        outputDisplay();

    if (getSmoothedHVSensorReading() > rawHVADCThreshold) {
      if ((i % 8) == 0 ) {
        decPWMOnTime();
      }
    }
  }

  int topOnValue = pwmOn;

  int aveOnValue = (bottomOnValue + topOnValue) / 2;
  setPWMOnTime(aveOnValue);

  // *************** fourth pass - adjust the frequency *************
  rawHVADCThreshold = getRawHVADCThreshold(hvTargetVoltage + 5);

  // Calibrate HVGen at full
  for (int i = 0 ; i < 768 ; i++ ) {
    //loadNumberArraySameValue(8);
    //allBright();
    //outputDisplay();
    checkHVVoltage();
  }
}


// ************************************************************
// Calculate the target value for the ADC reading to get the
// defined voltage
// ************************************************************
int getRawHVADCThreshold(double targetVoltage) {
  double externalVoltage = targetVoltage * 4.7 / 357 * 1023 / 5;
  int rawReading = (int) externalVoltage;
  return rawReading;
}

/**
   Set the PWM top time. Bounds check it so that it stays
   between the defined minimum and maximum, and that it
   does not go under the PWM On time (plus a safety margin).

   Set both the internal "pwmTop" value and the register.
*/
void setPWMTopTime(int newTopTime) {
  if (newTopTime < PWM_TOP_MIN) {
    newTopTime = PWM_TOP_MIN;
  }

  if (newTopTime > PWM_TOP_MAX) {
    newTopTime = PWM_TOP_MAX;
  }

  if (newTopTime < (pwmOn + PWM_OFF_MIN)) {
    newTopTime = pwmOn + PWM_OFF_MIN;
  }

  ICR1 = newTopTime;
  pwmTop = newTopTime;
}


/**
   Set the new PWM on time. Bounds check it to make sure
   that is stays between pulse min and max, and that it
   does not get bigger than PWM top, less the safety margin.

   Set both the internal "pwmOn" value and the register.
*/
void setPWMOnTime(int newOnTime) {
  if (newOnTime < PWM_PULSE_MIN) {
    newOnTime = PWM_PULSE_MIN;
  }

  if (newOnTime > PWM_PULSE_MAX) {
    newOnTime = PWM_PULSE_MAX;
  }

  if (newOnTime > (pwmTop - PWM_OFF_MIN)) {
    newOnTime = pwmTop - PWM_OFF_MIN;
  }

  OCR1A = newOnTime;
  pwmOn = newOnTime;
}

void incPWMOnTime() {
  setPWMOnTime(pwmOn + 1);
}

void decPWMOnTime() {
  setPWMOnTime(pwmOn - 1);
}

/**
   Get the HV sensor reading. Smooth it using a simple
   moving average calculation.
*/
int getSmoothedHVSensorReading() {
  int rawSensorVal = analogRead(HVSENSE);
  double sensorDiff = rawSensorVal - sensorHVSmoothed;
  sensorHVSmoothed += (sensorDiff / 100 / 8);
  int sensorHVSmoothedInt = (int) sensorHVSmoothed;
  return sensorHVSmoothedInt;
}

// ************************************************************
// Adjust the HV gen to achieve the voltage we require
// Pre-calculate the threshold value of the ADC read and make
// a simple comparison against this for speed
// We control only the PWM "off" time, because the "on" time
// affects the current consumption and MOSFET heating
// ************************************************************
void checkHVVoltage() {
  if (getSmoothedHVSensorReading() > rawHVADCThreshold) {
    setPWMTopTime(pwmTop + getInc());
  } else {
    setPWMTopTime(pwmTop - getInc());
  }
}

int getInc() {
  int diffValue = abs(getSmoothedHVSensorReading() - rawHVADCThreshold);
  int incValue = 1;
  if (diffValue > 20) incValue = 50;
  else if (diffValue > 10) incValue = 5;
  return incValue;  
}

// ************************************************************
// read EEPROM values
// ************************************************************
void readEEPROMValues() {

  hvTargetVoltage = EEPROM.read(EE_HV_VOLTAGE);
  if ((hvTargetVoltage < HVGEN_TARGET_VOLTAGE_MIN) || (hvTargetVoltage > HVGEN_TARGET_VOLTAGE_MAX)) {
    hvTargetVoltage = HVGEN_TARGET_VOLTAGE_DEFAULT;
  }

  pwmOn = EEPROM.read(EE_PULSE_HI) * 256 + EEPROM.read(EE_PULSE_LO);
  if ((pwmOn < PWM_PULSE_MIN) || (pwmOn > PWM_PULSE_MAX)) {
    pwmOn = PWM_PULSE_DEFAULT;

    // Hmmm, need calibration
    EEPROM.write(EE_HVG_NEED_CALIB, true);
  }

  pwmTop = EEPROM.read(EE_PWM_TOP_HI) * 256 + EEPROM.read(EE_PWM_TOP_LO);
  if ((pwmTop < PWM_TOP_MIN) || (pwmTop > PWM_TOP_MAX)) {
    pwmTop = PWM_TOP_DEFAULT;

    // Hmmm, need calibration
    EEPROM.write(EE_HVG_NEED_CALIB, true);
  }

}

// ISR triggered every 1 second
void oneSecondISR() {
  oldSecondsElapsed = secondsElapsed;
  secondsElapsed = secondsElapsed + 1;
  NumberArray[0] = secondsElapsed%10;
  NumberArray[1] = secondsElapsed/10;
}






void nixieOff() {
  TCCR1A = tccrOff; // turn OFF HV

  // TODO optimize with something like
  // turn all digits off - equivalent to digitalWrite(ledPin_a_n,LOW); (n=1,2,3,4,5,6) but much faster
  // PORTC = PORTC & B11110011;
  // PORTD = PORTD & B11101000;
  digitalWriteFast(ANODE6PIN, LOW);
  digitalWriteFast(ANODE5PIN, LOW);
  digitalWriteFast(ANODE4PIN, LOW);
  digitalWriteFast(ANODE3PIN, LOW);
  digitalWriteFast(ANODE2PIN, LOW);
  digitalWriteFast(ANODE1PIN, LOW);

  delay(1);
}

void nixieOn(byte digit) {

  set74141(NumberArray[digit]);
  
  switch (digit) {
    case 0: digitalWriteFast(ANODE6PIN, HIGH); break;
    case 1: digitalWriteFast(ANODE5PIN, HIGH); break;
    case 2: digitalWriteFast(ANODE4PIN, HIGH); break;
    case 3: digitalWriteFast(ANODE3PIN, HIGH); break;
    case 4: digitalWriteFast(ANODE2PIN, HIGH); break;
    case 5: digitalWriteFast(ANODE1PIN, HIGH); break;
  } // end switch digit
  
  
  TCCR1A = tccrOn;
}


void nixieOnDate(byte digit) {

  set74141(DateArray[digit]);
  
  switch (digit) {
    case 0: digitalWriteFast(ANODE6PIN, HIGH); break;
    case 1: digitalWriteFast(ANODE5PIN, HIGH); break;
    case 2: digitalWriteFast(ANODE4PIN, HIGH); break;
    case 3: digitalWriteFast(ANODE3PIN, HIGH); break;
    case 4: digitalWriteFast(ANODE2PIN, HIGH); break;
    case 5: digitalWriteFast(ANODE1PIN, HIGH); break;
  } // end switch digit
  
  
  TCCR1A = tccrOn;
}

void set74141(int digit) {

  // Set the bits we need
  switch ( digit )
  {
    case 0:       
      digitalWriteFast(BCD1PIN, LOW); // 0
      digitalWriteFast(BCD2PIN, LOW);
      digitalWriteFast(BCD3PIN, LOW);
      digitalWriteFast(BCD4PIN, LOW);                      
      break; // a=0;b=0;c=0;d=0
    case 1: 
      digitalWriteFast(BCD1PIN, HIGH); // 1
      digitalWriteFast(BCD2PIN, LOW);
      digitalWriteFast(BCD3PIN, LOW);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=1;b=0;c=0;d=0
    case 2:  
      digitalWriteFast(BCD1PIN, LOW); // 2
      digitalWriteFast(BCD2PIN, HIGH);
      digitalWriteFast(BCD3PIN, LOW);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=0;b=1;c=0;d=0
    case 3:  
      digitalWriteFast(BCD1PIN, HIGH); // 3
      digitalWriteFast(BCD2PIN, HIGH);
      digitalWriteFast(BCD3PIN, LOW);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=1;b=1;c=0;d=0
    case 4: 
      digitalWriteFast(BCD1PIN, LOW); // 4
      digitalWriteFast(BCD2PIN, LOW);
      digitalWriteFast(BCD3PIN, HIGH);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=0;b=0;c=1;d=0
    case 5:  
      digitalWriteFast(BCD1PIN, HIGH); // 5
      digitalWriteFast(BCD2PIN, LOW);
      digitalWriteFast(BCD3PIN, HIGH);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=1;b=0;c=1;d=0
    case 6: 
      digitalWriteFast(BCD1PIN, LOW); // 6
      digitalWriteFast(BCD2PIN, HIGH);
      digitalWriteFast(BCD3PIN, HIGH);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=0;b=1;c=1;d=0
    case 7:  
      digitalWriteFast(BCD1PIN, HIGH); // 7
      digitalWriteFast(BCD2PIN, HIGH);
      digitalWriteFast(BCD3PIN, HIGH);
      digitalWriteFast(BCD4PIN, LOW);
      break; // a=1;b=1;c=1;d=0
    case 8: 
      digitalWriteFast(BCD1PIN, LOW); // 8
      digitalWriteFast(BCD2PIN, LOW);
      digitalWriteFast(BCD3PIN, LOW);
      digitalWriteFast(BCD4PIN, HIGH);
      break; // a=0;b=0;c=0;d=1
    case 9:
      digitalWriteFast(BCD1PIN, HIGH); // 9
      digitalWriteFast(BCD2PIN, LOW);
      digitalWriteFast(BCD3PIN, LOW);
      digitalWriteFast(BCD4PIN, HIGH);
      break; // a=1;b=0;c=0;d=1
    default: 
      nixieOff();
      break;
  }

}

void fadeOut() {
  // TODO, doing nothing for now
}


// TODO TOFIX TOREDO
// user interaction to set the time
void doSetTime() {

//  byte newminutes;
//  byte newhours;
//  byte temp;
//  
//  RTCnow = rtc.now();
//  minutes = RTCnow.minute();
//  newminutes = minutes;
//  hours = RTCnow.hour();  
//  newhours = hours;
//
//  
//
//  // get the new hours decade
//  temp = hours/10;
//  outputToNixie(temp);
//  delay(600); // debouncing BUTTON1
//  
//  do {
//    outputToNixie(temp);
//    if (button2.debounce()) {
//      temp = (++temp) % 3;
//    }
//  } while (digitalRead(BUTTON1) == HIGH);
//
//
//
//  blinkDigit(temp);
//  newhours = temp*10; // newhours can be 00, 10, 20.
//
//  delay(100); // debouncing BUTTON1
//
//  // get the new hours unit
//  temp = hours%10;
//  do {
//    outputToNixie(temp);
//    if (button2.debounce()) {
//      if (newhours == 20) {
//        temp = (++temp) % 4;
//      } else {
//        temp = (++temp) % 10;
//      }
//    }
//  } while (digitalRead(BUTTON1) == HIGH);
//
//  blinkDigit(temp);
//  newhours = newhours + temp;
//
//  // get the new minutes decade
//  temp = minutes/10;
//  do {
//    outputToNixie(temp);
//    if (button2.debounce()) {
//      temp = (++temp) % 6;
//    }
//  } while (digitalRead(BUTTON1) == HIGH);
//
//
//  blinkDigit(temp);
//  newminutes = temp*10; // newhours can be 00, 10, 20.
//
//  delay(100); // debouncing BUTTON1
//
//  // get the new minutes unit
//  temp = minutes%10;
//  do {
//    outputToNixie(temp);
//    if (button2.debounce()) {
//        temp = (++temp) % 10;
//    }
//  } while (digitalRead(BUTTON1) == HIGH);
//
//  blinkDigit(temp);
//  newminutes = newminutes + temp;  
//
//  // update the running variables of current time
//  minutes = newminutes;
//  hours = newhours;
//  
//  rtc.adjust(DateTime(2022, 11, 21, newhours, newminutes, 20));
//  delay(300);
  
}


void blinkDigit(byte digit, byte times = 4) {
  // blink the new value for confirmation
  for (int i=0; i<times; i++) {
//    outputToNixie(digit);
    delay(300);
    nixieOff();
    delay(300);
  };
}

// ******** TEMPORARY FUNCTION ************
// TODO REWRITE
void outputDisplay() {
  digitalWrite(BCD1PIN, LOW);
  digitalWrite(BCD2PIN, LOW);
  digitalWrite(BCD3PIN, LOW);
  digitalWrite(BCD4PIN, LOW);
}


// ************************************************************
// Break the time into displayable digits
// from ArduNix code
// ************************************************************
void loadNumberArrayTime() {
  NumberArray[5] = seconds % 10;
  NumberArray[4] = seconds / 10;
  NumberArray[3] = minutes % 10;
  NumberArray[2] = minutes / 10;
// TODO enable 12h format
//  if (hourMode) {
//    NumberArray[1] = hourFormat12() % 10;
//    NumberArray[0] = hourFormat12() / 10;
//  } else {
    NumberArray[1] = hours % 10;
    NumberArray[0] = hours / 10;
  //}
}

// ************************************************************
// Break the date into displayable digits
// from ArduNix code
// ************************************************************
void loadNumberArrayDate() {
  NumberArray[5] = (year_nr-2000) % 10;
  NumberArray[4] = (year_nr-2000) / 10;
  NumberArray[3] = month_nr % 10;
  NumberArray[2] = month_nr / 10;
  NumberArray[1] = month_day % 10;
  NumberArray[0] = month_day / 10;
  
}
