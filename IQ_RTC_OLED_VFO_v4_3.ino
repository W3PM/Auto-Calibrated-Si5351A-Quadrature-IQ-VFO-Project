/*

  Arduino Controlled GPS Corrected Dual Output Si5351A VFO

  A quadrature VFO project that uses an Arduino Uno or Nano to control a SI5351A clock
  generator breakout board. The VFO uses a DS3231 RTC to software calibrate the output 
  frequency and provide accurate time.

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Copyright (C) 2020,  Gene Marcus W3PM GM4YRE

  14 May, 2020

  30 December 2021  v4_3 Newer baatches of Si5351's default to spread spectrum if
                         not set to disable. This version includes code to disable
                         the spread spectrum mode.     

 COMPILER NOTE:    Uses libraries SSD1306Ascii by Bill Greiman and PinChangeInterrupt
                   by NicoHood
                   Load from: Sketch > Include Library > Manage Libraries
                   (Windows users will find the menu bar on top of the active window. 
                   Linux and MAC users will find the menu bar at the top of the primary 
                   display screen.) 


  ------------------------------------------------------------------------
  Nano Digital Pin Allocation

  D0
  D1
  D2  RTC 1pps input
  D3  Rotary encoder pin A
  D4  Rotary encoder pin B
  D5  2.5 MHz input from Si5351 CLK0 pin
  D6  Decrease frequency button
  D7  Increase frequency button
  D8  Band Select button
  D9  Frequency resolution button
  D10
  D11 Offset enable
  D12
  D13
  A0/D14
  A1/D15
  A2/D16
  A3/D17
  A4/D18 Si5351 SDA
  A5/D19 Si5351 SCL

  ----------------------------------------------------------------
*/

/*
  _________________________________________________________________

  Enter any number of Band Select frequencies.
  Use (0,0,0) as the last entry.

  Restrict frequency entries to > 3.5 MHz and < 53 MHz

  ___________Enter Band Select frequencies below_____________________
*/
const unsigned long Freq_array [] = {

  3560000,
  7040000,
  10106000,
  14060000,
  18096000,
  21060000,
  24906000,
  28060000,
  50060000,

  0
};

//_________________________Enter callsign or name (Max 10 characters) below:___________________
char callsign[11] = "IQ VFO"; // This is used to personalize your project on the "CLOCK" display

//_________________________Enter offset frequency (Hz) below:__________________________________
int fOffset = -600;        // -600 Hz offset

//_________________________Auto calibrate using DS3231? (false = NO, true = YES)_______________
bool AutoCalibrate = true;


//___________________________Enter stand-alone calibration factor:______________________________
//    This entry is not required if GPS calibration is selected.
//  - Connect VFO to a frequency counter
//  - Set VFO to 25 MHz
//  - Annotate counter frequency in Hz
//  - Subtract 25 MHz from counter reading
//  - Enter the difference in Hz (i.e. -245)
int CalFactor = 0;

// include the library code:
#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "PinChangeInterrupt.h"

// Set up MCU pins
#define InterruptPin             2
#define encoderPinA              3
#define encoderPinB              4
#define Resolution               9
#define FreqDown                 6
#define FreqUp                   7
#define Offset                  11
#define BandSelect               8
#define SecLED                  13
#define calibrate                9
#define endCalibrate             8
#define CFup                     7
#define CFdown                   6
#define pushButton1              6
#define pushButton2              7
#define pushButton3              8
#define pushButton4              9
#define menu                    10

// Set DS3231 I2C address
#define DS3231_addr           0x68

// Set sI5351A I2C address
#define Si5351A_addr          0x60

// Define OLED address
#define I2C_ADDRESS           0x3C

// initialize oled display
SSD1306AsciiAvrI2c oled;

// Define Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define SSC_EN                 149
#define CLK0_PHOFF             165
#define CLK1_PHOFF             166
#define CLK2_PHOFF             167
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

typedef struct {  // Used in conjunction with PROGMEM to reduce RAM usage
  char description [4];
} descriptionType;

// configure variables
byte fStepcount, offsetFlag = 0, band;
String resolution = "1 Hz  ";
int hours, minutes, seconds, xday, xdate, xmonth, xyear, startCount = 0;
unsigned int tcount = 2, tcount2, encoderA, encoderB, encoderC = 1, CFcount, AutoCalFactor[50];
unsigned long fStep = 1, XtalFreq = 25000000;
unsigned long mult = 0, Freq_1, Freq_2, time_now = 0, VFOtime_now = 0;
const descriptionType daysOfTheWeek [7] PROGMEM = { {"SUN"}, {"MON"}, {"TUE"}, {"WED"}, {"THU"}, {"FRI"}, {"SAT"},};
const descriptionType monthOfTheYear [12] PROGMEM = { {"JAN"}, {"FEB"}, {"MAR"}, {"APR"}, {"MAY"}, {"JUN"}, {"JUL"}, {"AUG"}, {"SEP"}, {"OCT"}, {"NOV"}, {"DEC"},};
const byte daysInMonth [] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
bool StartCalc = true, LEDtoggle, suspendUpdateFlag = false, oneSecondTrigger = true, startFlag = false, toggle = false, timeSet_toggle = false;
volatile bool fired = false;
volatile long rotaryCount = 0;
int8_t PLLmult, tempPLLmult;
uint32_t tempPLL;
int multiplier;


//***********************************************************************
// This interrupt is used for Si5351 25MHz crystal frequency calibration
// Called every second by from the DS3231 RTC 1PPS to Nano pin D2
//***********************************************************************
void Interrupt()
{
  tcount++;
  if (tcount == 4)                                    // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                       //Clock on falling edge of pin 5
  }
  else if (tcount == 44)                              //Total count is = XtalFreq x 4 - stop counting
  {
    TCCR1B = 0;                                       //Turn off counter
    unsigned long TempFreq = (mult * 0x10000 + TCNT1);//Calculate corrected XtalFreq
    TCNT1 = 0;                                        //Reset count to zero
    mult = 0;
    tcount = 0;                                       //Reset the seconds counter

    // The following is an averageing alorithm used to smooth out DS3231 1pps jitter
    // Note: The upper and lower frequecny constraint prevents autocalibration data corruption
    //       which may occur if the COUNTER switch is in the wrong position.
    if (TempFreq > 99988000L & TempFreq < 100012000L) // Check bounds
    {
      int N = 12;
      if (CFcount == N)
      {
        CFcount = 0;
        StartCalc = true;
      }
      if (StartCalc == false)                       // This is the initial warm-up period
      {
        AutoCalFactor[CFcount] = 100000000UL - TempFreq;
        if (suspendUpdateFlag == true) CFcount++;   // Don't update corrected crystal frequency

        else
        {
          XtalFreq = TempFreq / 4UL;                // Update corrected crystal frequency
          CFcount++;
        }
      }
      else                                          // Warm-up period completed, go into averaging mode
      {
        long temp = 0;
        AutoCalFactor[CFcount] = 100000000UL - TempFreq;
        for (int i = 0; i < N ; i++) temp = temp + AutoCalFactor[i];
        if (suspendUpdateFlag == false) XtalFreq = (100000000UL - (round(temp / N))) / 4UL; //Average the correction factors and update crystal frequency
        CFcount++;
      }
    }
  }
  oneSecondTrigger = true;
  LEDtoggle = !LEDtoggle;
  if (LEDtoggle == true) digitalWrite(SecLED, HIGH);
  else digitalWrite(SecLED, LOW);
}



// Timer 1 overflow intrrupt vector.
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}


//******************************************************************
// Rotary encoder intrrupt vector.
// Credit for this algorithm goes to Nick Gammon http://www.gammon.com.au
// Called upon by VFO function when rotary encoder turns
//******************************************************************
void doEncoder()
{
  static boolean ready;
  static unsigned long lastFiredTime;
  static byte pinA, pinB;

  // wait for main program to process it
  if (fired) return;

  byte newPinA = digitalRead (encoderPinA);
  byte newPinB = digitalRead (encoderPinB);

  // Forward is: LH/HH or HL/LL
  // Reverse is: HL/HH or LH/LL

  // so we only record a turn on both the same (HH or LL)

  if (newPinA == newPinB)
  {
    if (ready)
    {
      long increment = 1 * fStep;

      // As the encoder turns faster, the count goes up more
      unsigned long now = millis ();
      unsigned long interval = now - lastFiredTime;
      lastFiredTime = now;

      if (interval < 10)
        increment = 5 * fStep;
      else if (interval < 20)
        increment = 3 * fStep;
      else if (interval < 50)
        increment = 2 * fStep;

      if (newPinA == HIGH)    // must be HH now
      {
        if (pinA == LOW)
          Freq_1 -= increment;
        else
          Freq_1 += increment;
      }
      else
      { // must be LL now
        if (pinA == LOW)
          Freq_1 += increment;
        else
          Freq_1 -= increment;
      }
      fired = true;
      ready = false;
    }  // end of being ready
  }  // end of completed click
  else
    ready = true;

  pinA = newPinA;
  pinB = newPinB;
}


void setup()
{

  Wire.begin(1);                  // join i2c bus (address = 1)
  //si5351aStart();

  // Set up DS3231 for 1 Hz squarewave output
  // Needs only be written one time provided DS3231 battery is not removed
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E);
  Wire.write(0);
  Wire.endTransmission();

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag

  // Inititalize 1 Hz interrupt input pin
  pinMode(InterruptPin, INPUT);
  digitalWrite(InterruptPin, HIGH);         // internal pull-up enabled

  // Set pin 2 for external 1 Hz interrupt input
  attachInterrupt(digitalPinToInterrupt(InterruptPin), Interrupt, FALLING);

  // Set up push buttons
  pinMode(Resolution, INPUT);
  digitalWrite(Resolution, HIGH);         // internal pull-up enabled
  pinMode(BandSelect, INPUT);
  digitalWrite(BandSelect, HIGH);         // internal pull-up enabled
  pinMode(FreqUp, INPUT);
  digitalWrite(FreqUp, HIGH);             // internal pull-up enabled
  pinMode(FreqDown, INPUT);
  digitalWrite(FreqDown, HIGH);           // internal pull-up enabled
  pinMode(Offset, INPUT);
  digitalWrite(Offset, HIGH);             // internal pull-up enabled
  pinMode(menu, INPUT);
  digitalWrite(menu, HIGH);              // internal pull-up enabled

  // Set up LED
  pinMode(SecLED, OUTPUT);                // Uses on board Nano (Uno) LED
  digitalWrite(SecLED, LOW);

  // Retrieve stored data
  EEPROM.get (10, band);                  // Get stored VFO band
  if (band > 20 | band < 0) band = 0;     //Ensures valid EEPROM data - if invalid will default to band 0
  EEPROM.get (12, fStepcount);            // Get stored VFO step resolution
  if (fStepcount > 6 | fStepcount < 0) fStepcount = 0;    //Ensures valid EEPROM data - if invalid will default to 0
  EEPROM.get (14, Freq_1);                // Get stored VFO CLK1 frequency
  if (Freq_1 > 120000000 | Freq_1 < 0) Freq_1 = 1000000;    //Ensures valid EEPROM data - if invalid will default to 1 MHz

  // Set up rotary encoder
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);         // internal pull-up enabled
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);         // internal pull-up enabled
  attachPCINT(digitalPinToPCINT(encoderPinA), doEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinB), doEncoder, CHANGE);

  // Set oled font size and type, then display startup menu message
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);

  // Check for DS3231 RTC calibration function
  if (digitalRead(calibrate) == LOW) Calibrate(); //Used to enter calibration function upon reset

  TCCR1B = 0;

  // Initialize the Si5351
  Si5351_write(XTAL_LOAD_CAP, 0b11000000);      // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000000); // Enable CLK0, CLK1 and CLK2
  Si5351_write(CLK0_CONTROL, 0b00001111);       // Set PLLA to CLK0, 8 mA output
  Si5351_write(CLK1_CONTROL, 0b00101111);       // Set PLLB to CLK1, 8 mA output
  Si5351_write(CLK2_CONTROL, 0b00101111);       // Set PLLB to CLK2, 8 mA output
  Si5351_write(PLL_RESET, 0b10100000);          // Reset PLLA and PLLB
  Si5351_write(SSC_EN,0b00000000);             // Disable spread spectrum

  // Clear OLED display
  oled.clear();


  if (AutoCalibrate == false)
  {
    // Add CalFactor to the Si5351 crystal frequency for non-DS3231 frequency updates
    XtalFreq += CalFactor;  // Use corection factor if 1pps not used.
    detachInterrupt(digitalPinToInterrupt(InterruptPin)); // Disable the 1pps interrupt
    setfreq();
    setResolution();
  }
  else
  {
    // Set CLK0 to 2.5 MHZ for autocalibration
    si5351aSetPLL(SYNTH_PLL_A, 750000000L); // Set PLLA to 750 MHz
    si5351aSetFreq(SYNTH_MS_0, 300); // Set CLK0 to 2.5 MHz (7500 / 2.5 = 300)
    oled.setCursor(0, 4);
    oled.println(F("PB3: scroll"));
    oled.print(F("PB2: select"));
  }
}


//******************************************************************
// Loop starts here:
// Loops consecutively to check MCU pins for activity
//******************************************************************
void loop()
{
  if (AutoCalibrate == false)
  {
    VFO();
  }
  else
  {
    // Loop through the  menu rota until a selection is made
    while (digitalRead(pushButton2) == HIGH & startFlag == false)
    {
      if (digitalRead(pushButton3) == LOW)
      {
        altDelay(500);
        startCount = startCount + 1;;
        if (startCount > 3) startCount = 0;
      }

      switch (startCount) {
        case 0:
          oled.setCursor(0, 0);
          oled.println(F("    VFO  "));
          break;
        case 1:
          oled.setCursor(0, 0);
          oled.println(F("   CLOCK "));
          break;
        case 2:
          oled.setCursor(0, 0);
          oled.println(F(" SET CLOCK"));
          break;
        case 3:
          oled.setCursor(0, 0);
          oled.println(F(" SET DATE "));
          break;
      }
    }

    if (startFlag == false)
    {
      oled.clear();
      oled.set1X();
      startFlag = true;
    }
    switch (startCount) {
      case 0: // VFO
        VFO();
        break;
      case 1: // Display the clock
        DisplayClock2();
        break;
      case 2: // RTC clock adjustment begins here:
        adjClock();
        break;
      case 3: // RTC date adjustment begins here:
        setDATE();
        break;
        // Selection is made at this point, now go to the selected function
    }

    if (digitalRead (menu) == LOW)
    {
      startFlag = false;
      //   Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1
      //   detachPCINT(digitalPinToPCINT(encoderPinA));
      //   detachPCINT(digitalPinToPCINT(encoderPinB));
      //   VFOstartFlag = true;
      //   tcount = 0;
      //   attachInterrupt(digitalPinToInterrupt(InterruptPin), Interrupt, FALLING);
      oled.clear();
      oled.setCursor(0, 4);
      oled.println(F("PB3: scroll"));
      oled.print(F("PB2: select"));
    }
  }
}



//******************************************************************
// VFO function follows:
// Used to select band, change frequency, and control offset function
// Called called by "switch case1" in loop
//******************************************************************
void VFO()
{
  if (tcount == 1)
  {
    // Update the SI5351A after every autocal correction
    tcount = 2;
    SetFreq_IQ();
  }

  // If rotary encoder was active update VFO frequency
  if (fired)
  {
    setfreq();                                   //Update and display new frequency
    resDisplay();                                //Update resolution display and set display timer
    fired = false;
  }


  // The frequency step resolution selection begins here:
  if (digitalRead(Resolution) == LOW)
  {
    // This function is used in lieu of "delay" to prevent timer/interrupt conflicts
    altDelay(500);
    fStepcount++;
    if (fStepcount > 6)fStepcount = 0;
    EEPROM.put(12, fStepcount);
    setResolution();     // Call the set resolution subroutine
  }

  // Band selection begins here:
  if (digitalRead(BandSelect) == LOW)
  {
    altDelay(500);
    band = band + 1;                         // Increment band selection
    EEPROM.put(10, band);
    if (Freq_array [band] == 0)band = 0; // Check for end of frequency array
    Freq_1 = Freq_array [band];                // Load CLK1 frequency
    setfreq();                                     // Display and set CLK1 frequency
  }

  // Frequency offset algorithm begins here:
  if (digitalRead(Offset) == LOW && offsetFlag == 0) // Check for offset pin A2 LOW
  {
    offsetFlag = 1;                                 // Set flag
    Freq_1 += fOffset;                              // Add offset frequency
    oled.setCursor(108, 4);                           // Display a "*" on the display
    oled.print("*");
    setfreq();                                      // Display and set CLK1 frequency + offset
  }

  if (digitalRead(Offset) == HIGH && offsetFlag == 1) // Check for offset pn A2 HIGH
  {
    offsetFlag = 0;                                  // Reset flag
    Freq_1 -= fOffset;                               // Subtract the offset frequency
    oled.setCursor(108, 4);                            // Clear the "*" on the display
    oled.print(" ");
    setfreq();                                       // Display and set CLK1 frequency - offset
  }

  // Frequency Up/Down pushbutton algorithm begin here:
  if (digitalRead(FreqUp) == LOW) // Check for frequency up pushbutton A1 LOW
  {
    altDelay(500);
    // Increase frequency by the selected frequency step
    Freq_1 += fStep;                                // Increase CLK1 by frequency step
    setfreq();                                      // Set and display new frequency
    resDisplay();                                   // Call the resolution display subroutine
  }

  if (digitalRead(FreqDown) == LOW) // Check for frequency up pushbutton A1 LOW
  {
    altDelay(500);
    // Decrease frequency by the selected frequency step and check for 1-80 MHz limits
    Freq_1 -= fStep;                               // Decrease CLK1 by frequency step
    setfreq();                                     // Set and display new frequency
    resDisplay();                                  // Call the resolution display subroutine
  }

  if (AutoCalibrate == true)
  {
    displayClock();
  }

  // per Arduino data: EEPROM.put only writes if Freq_1 is changed, however,
  // a 10 second delay is used to prevent unnecessary EEPROM updates during
  // short frequency changes
  if (millis() > VFOtime_now + 10000)
  {
    EEPROM.put(14, Freq_1);
    VFOtime_now = millis();
  }
}



//******************************************************************
// displayClock follows:
// Displays the time and date
//
// Called by loop()
//******************************************************************
void displayClock()
{
  oled.setCursor(12, 4);
  getTime();

  if (hours < 10) oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));

  if (minutes < 10) oled.print(F("0"));
  oled.print(minutes);
  oled.print(F(":"));

  if (seconds < 10) oled.print(F("0"));
  oled.print(seconds);

  getDate();

  oled.setCursor(0, 6);
  descriptionType oneItem;
  memcpy_P (&oneItem, &daysOfTheWeek [xday], sizeof oneItem);
  oled.print (oneItem.description);
  oled.print(F(" "));

  if (xdate < 10) oled.print(F("0"));
  oled.print(xdate);
  oled.print(F(" "));
  memcpy_P (&oneItem, &monthOfTheYear [xmonth - 1], sizeof oneItem);
  oled.print (oneItem.description);
  oled.print(F("  "));
}


//******************************************************************
//  Time update function follows:
//  Used to retrieve the correct time from the DS3231 RTC
//
//  Called by displayClock()
//******************************************************************
void  getTime()
{
  // Send request to receive data starting at register 0
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 3); // request three bytes (seconds, minutes, hours)

  while (Wire.available())
  {
    seconds = Wire.read(); // get seconds
    minutes = Wire.read(); // get minutes
    hours = Wire.read();   // get hours

    seconds = (((seconds & 0b11110000) >> 4) * 10 + (seconds & 0b00001111)); // convert BCD to decimal
    minutes = (((minutes & 0b11110000) >> 4) * 10 + (minutes & 0b00001111)); // convert BCD to decimal
    hours = (((hours & 0b00100000) >> 5) * 20 + ((hours & 0b00010000) >> 4) * 10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)
  }
}


//******************************************************************
//  Date update function follows:
//  Used to retrieve the correct date from the DS3231 RTC
//
//  The day of the week algorithm is a modified version
//  of the open source code found at:
//  Code by JeeLabs http://news.jeelabs.org/code/
//
//  Called by displayClock()
//******************************************************************
void  getDate()
{
  int nowDay;
  int nowDate;
  int tempdate;
  int nowMonth;
  int nowYear;

  // send request to receive data starting at register 3
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x03); // start at register 3
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 4); // request four bytes (day date month year)

  while (Wire.available())
  {
    nowDay = Wire.read();    // get day (serves as a placeholder)
    nowDate = Wire.read();   // get date
    nowMonth = Wire.read();  // get month
    nowYear = Wire.read();   // get year

    xdate = (((nowDate & 0b11110000) >> 4) * 10 + (nowDate & 0b00001111)); // convert BCD to decimal
    tempdate = xdate;
    xmonth = (((nowMonth & 0b00010000) >> 4) * 10 + (nowMonth & 0b00001111)); // convert BCD to decimal
    xyear = ((nowYear & 0b11110000) >> 4) * 10 + ((nowYear & 0b00001111)); // convert BCD to decimal

    if (xyear >= 2000) xyear -= 2000;
    for (byte i = 1; i < xmonth; ++i)
      tempdate += pgm_read_byte(daysInMonth + i - 1);
    if (xmonth > 2 && xyear % 4 == 0)
      ++tempdate;
    tempdate = tempdate + 365 * xyear + (xyear + 3) / 4 - 1;
    xday = (tempdate + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
  }
}


//******************************************************************
// Set VFO frequency function follows:
// Calculates the frequency data to be sent to the Si5351 clock generator
// and displays the frequency on the oled display
// Called by the VFO function
//******************************************************************
void setfreq()
{
  unsigned long  frequency = Freq_1; // Temporarily store Freq_1

  oled.setCursor(0, 0);
  char buf[11];
  SetFreq_IQ();
  
  if (frequency >= 1000000L)
  {
    int MHz = int(frequency / 1000000L);
    int kHz = int ((frequency - (MHz * 1000000L)) / 1000L);
    int Hz = int (frequency % 1000);
    snprintf(buf, sizeof(buf), "%2u,%03u,%03u", MHz, kHz, Hz);
  }

  else if (frequency >= 1000L & frequency < 1000000L)
  {
    int kHz = int (frequency / 1000L);
    int Hz = int (frequency % 1000L);
    snprintf(buf, sizeof(buf), "%6u,%03u", kHz, Hz);
  }
  else if (frequency < 1000L)
  {
    int Hz = int (frequency);
    snprintf(buf, sizeof(buf), "%10u", Hz);
  }
  oled.print(buf);
}


//******************************************************************
// Step resolution function follows:
// Sets the frequency step resolution
// Called by setup() and  loop()
//******************************************************************
void setResolution()
{
  switch (fStepcount)
  {
    case 0:
      fStep = 1;
      resolution = "1 Hz  ";
      break;
    case 1:
      fStep = 10;
      resolution = "10 Hz  ";
      break;
    case 2:
      fStep = 100;
      resolution = "100 Hz ";
      break;
    case 3:
      fStep = 1000;
      resolution = "1 kHz  ";
      break;
    case 4:
      fStep = 10000;
      resolution = "10 kHz ";
      break;
    case 5:
      fStep = 100000;
      resolution = "100 kHz";
      break;
    case 6:
      fStep = 1000000;
      resolution = "1 MHz  ";
      break;
  }
  resDisplay();
}




//******************************************************************
// Date/Time Clock display function follows:
// Used to display date and time as well as DS3231 chip temperature
// Note: displayed temperature is usually a few degrees warmer than
//       ambient temperature
// Called called by "switch case3" in loop
//******************************************************************
void DisplayClock2()
{
  //oled.setFont(fixed_bold10x15);
  oled.set2X();
  oled.setCursor(3, 2);

  getTime();

  if (hours < 10) oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));

  if (minutes < 10) oled.print(F("0"));
  oled.print(minutes);

  oled.set1X();
  oled.setCursor(0, 6);
  oled.print(callsign);
  oled.setCursor(96, 6);

  if (seconds < 10) oled.print(F("0"));
  oled.print(seconds);

  if (millis() > time_now + 10000)
  {
    tcount2++;
    if(tcount2 > 3) tcount2 = 0;
    //  toggle = !toggle;
    time_now = millis();
  }

  //  if (toggle == true)
  switch (tcount2)
  {
    case 0:
      getDate();
      oled.setCursor(0, 0);
      descriptionType oneItem;
      //   memcpy_P (&oneItem, &daysOfTheWeek [now.dayOfTheWeek()], sizeof oneItem);
      memcpy_P (&oneItem, &daysOfTheWeek [xday], sizeof oneItem);
      oled.print (oneItem.description);
      //    oled.print(daysOfTheWeek[now.dayOfTheWeek()]);
      oled.print(F(" "));

      if (xdate < 10) oled.print(F("0"));
      oled.print(xdate);
      oled.print(F(" "));
      memcpy_P (&oneItem, &monthOfTheYear [xmonth - 1], sizeof oneItem);
      oled.print (oneItem.description);
      oled.print(F("  "));
      break;

    case 1:
      oled.setCursor(0, 0);
      setfreq();
      break;


    case 2:
      Wire.beginTransmission(DS3231_addr);         // Start I2C protocol with DS3231 address
      Wire.write(0x11);                            // Send register address
      Wire.endTransmission(false);                 // I2C restart
      Wire.requestFrom(DS3231_addr, 2);            // Request 11 bytes from DS3231 and release I2C bus at end of reading
      int temperature_msb = Wire.read();           // Read temperature MSB
      int temperature_lsb = Wire.read();           // Read temperature LSB

      temperature_lsb >>= 6;

      // Convert the temperature data to F and C
      // Note: temperature is DS3231 temperature and not ambient temperature
      //       correction factor of 0.88 is used
      oled.setCursor(0, 0);
      int DegC = int(((temperature_msb * 100) + (temperature_lsb * 25)) * 0.88);
      oled.print(DegC / 100);
      //oled.print(temp);
      oled.print(F("."));
      oled.print((DegC % 100) / 10);
      oled.print(F("C "));
      int DegF = int(DegC * 9 / 5 + 3200);
      oled.print(DegF / 100);
      oled.print(F("."));
      oled.print((DegF % 100) / 10);
      oled.print(F("F"));
      break;
  }
}


//******************************************************************
// Clock adjust function follows:
// Used to adjust the system time
// Note: Quickly depressing pushbutton 1 will synchronize the clock
//       to the top of the minute "0"
//       Holding down pushbutton 4 while depressing pushbutton 1
//       will advance the hours.
//       Holding down pushbutton 3 while depressing pushbutton 2
//       will advance the minutes.
// Called called by "switch case4" in loop
//******************************************************************
void adjClock()
{
  getTime();

  // Display current time
  int tempHour = hours;
  int tempMinute = minutes;
  int tempSecond = seconds;
  oled.setCursor(0, 0);
  oled.print(F("  "));
  int a = hours;
  int b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);
  oled.print(F(":"));
  a = minutes;
  b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);
  oled.print(F(":"));
  a = seconds;
  b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);

  // Display legend
  oled.setCursor(0, 2);
  oled.print(F("Hold PB4"));
  oled.setCursor(0, 4);
  oled.print(F("PB1: Hours"));
  oled.setCursor(0, 6);
  oled.print(F("PB2: Minutes"));

  // Start one button time synchronization routine
  if (digitalRead(pushButton1) == LOW)
  {
    if (tempSecond > 30) tempMinute++;
    if (tempMinute > 59) tempMinute = 0;
    tempSecond = 0;
    updateTime(tempSecond, tempMinute, tempHour);
    altDelay(500);
  }
  timeSet_toggle = false;

  // Start set time routine
  while (digitalRead(pushButton4) == LOW)
  {
    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(500);
      tempHour++;
      if (tempHour > 23) tempHour = 0;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton2) == LOW)
    {
      altDelay(500);
      tempMinute++;
      if (tempMinute > 59) tempMinute = 0;
      timeSet_toggle = true;
    }

    // Display set time
    oled.setCursor(0, 0);
    oled.print(F("  "));
    int a = tempHour;
    int b = a % 10;
    a = a / 10;
    oled.print(a);
    oled.print(b);
    oled.print(F(":"));
    a = tempMinute;
    b = a % 10;
    a = a / 10;
    oled.print(a);
    oled.print(b);
    oled.print(F(":00"));
  }

  // Update time if change is made
  if (timeSet_toggle == true)
  {
    int tempSecond = 0;
    updateTime(tempSecond, tempMinute, tempHour);
    timeSet_toggle = false;
  }
}



//******************************************************************
//  Time set function follows:
//  Used to set the DS3231 RTC time
//
//  Called by adjClock()
//******************************************************************
void updateTime(int updateSecond, int updateMinute, int updateHour)
{
  // Convert BIN to BCD
  updateSecond = updateSecond + 6 * (updateSecond / 10);
  updateMinute = updateMinute + 6 * (updateMinute / 10);
  updateHour = updateHour + 6 * (updateHour / 10);

  // Write the data
  Wire.beginTransmission(DS3231_addr);
  Wire.write((byte)0); // start at location 0
  Wire.write(updateSecond);
  Wire.write(updateMinute);
  Wire.write(updateHour);
  Wire.endTransmission();
}


//******************************************************************
// Date adjust function follows:
// Used to adjust the system date
// Note:
//       Holding down pushbutton 4 while depressing pushbutton 1
//       will advance the date.
//       Holding down pushbutton 4 while depressing pushbutton 2
//       will advance the month.
//       Holding down pushbutton 4 while depressing pushbutton 3
//       will advance the year.
// Called called by "switch case5" in loop
//******************************************************************
void setDATE()
{
  getDate();

  int updateDate = xdate;
  int updateMonth = xmonth;
  int updateYear = xyear;

  // Display currently stored date
  oled.setCursor(0, 0);
  oled.print(updateDate);
  oled.print(F(" "));
  descriptionType oneItem;
  memcpy_P (&oneItem, &monthOfTheYear [updateMonth - 1], sizeof oneItem);
  oled.print (oneItem.description);
  oled.print(F(" "));
  oled.print(updateYear);

  // Display legend
  oled.setCursor(0, 2);
  oled.print(F("Hold PB4"));
  oled.setCursor(0, 4);
  oled.print(F("1:Date"));
  oled.setCursor(0, 6);
  oled.print(F("2:Month 3:Yr"));

  // Start update
  while (digitalRead(pushButton4) == LOW)
  {
    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(500);
      updateDate++;
      if (updateDate > 31) updateDate = 0;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton2) == LOW)
    {
      altDelay(500);
      updateMonth++;
      if (updateMonth > 12) updateMonth = 1;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton3) == LOW)
    {
      altDelay(500);
      updateYear++;
      if (updateYear > 30) updateYear = 19;
      timeSet_toggle = true;
    }

    // Display updates
    oled.setCursor(0, 0);
    //if (xdate < 10) oled.print(F("0"));
    oled.print(updateDate);
    oled.print(F(" "));
    descriptionType oneItem;
    memcpy_P (&oneItem, &monthOfTheYear [updateMonth - 1], sizeof oneItem);
    oled.print (oneItem.description);
    oled.print(F(" "));
    oled.print(updateYear);
  }

  // Save data if updated
  if (timeSet_toggle == true)
  {
    // Convert DEC to BCD
    updateDate = ((updateDate / 10) * 16) + (updateDate % 10);
    updateMonth = ((updateMonth  / 10) * 16) + (updateMonth % 10);
    updateYear = ((updateYear / 10) * 16) + (updateYear % 10);

    // Write the data
    Wire.beginTransmission(DS3231_addr);
    Wire.write((byte)0x04); // start at register 4
    Wire.write(updateDate);
    Wire.write(updateMonth);
    Wire.write(updateYear);
    Wire.endTransmission();
  }
}


//******************************************************************
// Step resolution display function follows:
// Display the frequency step resolution
// Called by the VFO and step resolution functions
//******************************************************************
void resDisplay()
{
  oled.setCursor(0, 2);
  oled.print(F("Res= "));
  oled.setCursor(54, 2);
  oled.print(resolution);
}



//******************************************************************
// Alternate delay function follows:
// altDelay is used because the command "delay" causes critical
// timing errors.
//
// Called by all functions
//******************************************************************
unsigned long altDelay(unsigned long delayTime)
{
  time_now = millis();
  while (millis() < time_now + delayTime) //delay 1 second
  {
    __asm__ __volatile__ ("nop");
  }
}



//******************************************************************
//  Si5351 Set frequency IQ processing
//******************************************************************
void SetFreq_IQ()
{
  PLLmult = 800000000 / Freq_1;
  if (Freq_1 < 6500000) PLLmult = 440000000 / Freq_1;

  if (PLLmult < 15 | PLLmult > 126)
  {
    oled.setCursor(0, 0);
    oled.print(F("  IQ ERROR  "));
  }


  if (tempPLLmult != PLLmult)
  {
    tempPLL = (Freq_1 * PLLmult);
    si5351aSetPLL(SYNTH_PLL_B, tempPLL);


    si5351aSetFreq(SYNTH_MS_1, PLLmult); // Set CLK0 to frequency
    si5351aSetFreq(SYNTH_MS_2, PLLmult); // Set CLK1 to frequency

    Si5351_write(CLK0_PHOFF, 0);    // Set phase CLK0 to 0
    Si5351_write(CLK1_PHOFF, PLLmult);   // Set phase CLK1 to 90 degrees using PLLmult

    Si5351_write(PLL_RESET, 0b10000000);          // Reset PLLB

    tempPLLmult = PLLmult;
  }

  else
  {
    tempPLL = (Freq_1 * PLLmult);
    si5351aSetPLL(SYNTH_PLL_B, tempPLL);
  }
}



//******************************************************************
//  Si5351 Multisynch processing for phasing CLK1 and CLK2
//******************************************************************
void si5351aSetFreq(int synth, int multiplier)
{
  unsigned long long CalcTemp;
  unsigned long  a, p1, p2;

  a = multiplier ;

  p1 = (128 * a) - 512;
  p2 = 0;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}


//******************************************************************
//  Si5351 PLL processing
//******************************************************************
void si5351aSetPLL(int synth, long long PLLfreq)
{
  unsigned long long CalcTemp, a;
  unsigned long  b, c, p1, p2, p3;

  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  a = PLLfreq / XtalFreq;
  CalcTemp = PLLfreq % XtalFreq;
  CalcTemp *= c;
  CalcTemp /= XtalFreq ;
  b = CalcTemp;  // Calculated numerator


  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}




//******************************************************************
// Write I2C data function for the Si5351A follows:
// Writes data over the I2C bus to the appropriate device defined by
// the address sent to it.

// Called by sketch setup, VFO, si5351aSetFreq, and
// si5351aStart functions.
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}


//******************************************************************
// Set the DS3231 RTC crystal oscillator aging offset function follows:
// This is effectively the system calibration routine. Small
// capacitors can be swithed in or out within the DS3231 RTC to control
// frequency accuracy. Positive aging values add capacitance to the
// array, slowing the oscillator frequency. Negative values remove
// capacitance from the array, increasing the oscillator frequency.
// One offset count provides about 0.1 ppm change in frequency.

// To enter the calibration function hold down pushbutton 4 and invoke
// a reset. Refer to the calibration notes below.
//******************************************************************

void Calibrate()
{
  int CF, tempCF, busy;
  //oled.setFont(fixed_bold10x15);
  oled.set1X();
  oled.clear();
  oled.print(F("Cal CF ="));
  oled.setCursor(0, 2);
  oled.println(F("PB1 = Down"));
  oled.println(F("PB2 = Up"));
  oled.print(F("reset = Exit"));

  while (digitalRead(endCalibrate) == HIGH)
  {
    // send request to receive data starting at register 3
    Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
    Wire.write((byte)0x10); // start at register 0x10
    Wire.endTransmission();
    Wire.requestFrom(DS3231_addr, 1); // request one byte (aging factor)
    while (Wire.available())
    {
      CF = Wire.read();
    }
    if (CF > 127) CF -= 256;
    tempCF = CF;
    oled.setCursor(96, 0);
    oled.print(F("    "));
    oled.setCursor(96, 0);
    oled.println(CF);
    if (digitalRead(CFdown) == LOW) CF -= 1;
    if (digitalRead(CFup) == LOW) CF += 1;

    Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
    Wire.write((byte)0x0F); // start at register 0x0F
    Wire.endTransmission();
    Wire.requestFrom(DS3231_addr, 1); // request one byte to determine DS3231 update status
    while (Wire.available())
    {
      busy = Wire.read();
    }
    busy = bitRead(busy, 2);
    if (CF != tempCF & bitRead(busy, 2) == 0)
    {
      setAgingOffset(CF);
      forceConversion();
    }
    altDelay(500);
  }
}


void setAgingOffset(int offset)
{
  if (offset < 0) offset += 256;

  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x10);
  Wire.write(offset);
  Wire.endTransmission();
}


void forceConversion()
{
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E);

  Wire.write(B00111100);
  Wire.endTransmission();
}

/*******************************************************************************************

                                 CALIBRATION

 *******************************************************************************************


   While the default RTC is already very accurate its accuracy can be pushed even higher
   by adjusting its aging offset register.

   For normal operation calibration is not required. The default 2 parts per million accuracy
   of the RTC will result in an uncertainty of less than +/- 30 Hz on 20 meters.

   If WSPR is used, the time will require synchronization every 7 – 10 days without calibration.
   The re-synchronization timeframe can be stretched to a month or more with calibration.


   There are three ways to perfom DS3231 RTC aging offset calibration:
   1) Measure and adjust the 32 kHz output
   2) Set VFO frequency to 10 MHz and use the frequency delta as the aging offset
   3) Track the time over a period of days.

   A calibration function is provided view the current aging offset and enter a new offset.
   To enter the calibration funtion, hold down pushbutton 4 and invoke a reset. Invoke a
   reset to exit the calibration function.

   IMPORTANT NOTE: When using methods 2 and 3 above, any change will not take place until
                   the auto-calibration algorithm calculates the correction factor for the
                   Si5351A’s 25 MHz clock and the DS3231's temperature compensation algorithm
                   performs its calculation. This may take a minute or two before any
                   change appears. Additionally, the auto-calibration routine uses an
                   averaging alogithm over a few minutes to reduce the effects of 1pps gate
                   time jitter. Any adjustment (other than the 32 KHz method) will be an
                   iterative process and will take some time.


   32 kHz & FREQUENCY COUNTER METHOD:
   Attach a 10 kohm pull-up resistor from the VCC to the 32K pin of the RTC. Attach a high
   resolution accurate counter from the 32K pin to ground.
   Enter the calibration function (see above) and use pushbuttons 1 and 2 to adjust the
   frequency as closely to 32768 Hz as possible.


   VFO & FREQUENCY COUNTER METHOD:
   Set the VFO to 10 Mhz and measure the frequency from CLK1 with an accurate frequency
   counter. Measure to the nearest Hz and subract this number from 10 MHz. This will be
   the aging offset. Enter the calibration function (see above). If this is the first
   time you performed a calibration you should see "CF = 0" on the display. If not, add
   the measured aging factor to the displayed number.  Use pushbuttons 1 and 2 to set the
   device to the aging factor. Invoke a reset to exit.

   Note: Per the DS3231 datasheet at  +25°C,  one LSB (in the offset register) typically
         provides about 0.1ppm change in frequency. At 10MHx 0.1ppm equates to 1 Hz.
         Theoretically, the process described above should produce get you right on
         target. Practically, I found that I had to go back and forth to obtain the
         greatest accuracy. Allow the system a few minutes to stabilize before making
         any adjustments (refer to the note above).


   TIME TRACKING METHOD:
   Sychronize the device's displayed time with an accuate time source such as WWV.
   After a few days compare the displayed time the reference time source. The
   following formula should bring you close to nominal:

     aging offset = (change in seconds / (86700 x number of days)) x 10^7

   Use a positive integer if the clock is running fast and a negative integer if it is
   running slow.

   Enter the calibration function (see above). If this is the first time you
   performed a calibration you should see "CF = 0" on the display. If not, add the
   measured aging offset to the displayed number.  Use pushbuttons 1 and 2 to set the
   device to the aging factor. Invoke a reset to exit.

 ********************************************************************************************
*/
//-----------------------------------------------------------------------------------------------
