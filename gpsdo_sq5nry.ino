/*
 Arduino Controlled GPS Corrected Generator
 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.

 Based on the projects: 
 W3PM (http://www.knology.net/~gmarcus/)
 &  SQ1GU (http://sq1gu.tobis.com.pl/pl/syntezery-dds/44-generator-si5351a)
*/
#include <TinyGPS++.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>

#include <EEPROMex.h>
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"

// GPS receiver
TinyGPSPlus gps;
#define ppsPin 2

// Si5351 PLL synthesiser
Si5351 si5351;
#define MIN_FREQ 8000
#define MAX_FREQ 160000000

// OLED Pins
#define CS_PIN  10
#define RST_PIN 9
#define DC_PIN  8

// display
SSD1306AsciiSpi oled;

// jumpers for pre-set frequency on start
#define Z1 4
#define Z2 6

// fixed frequency presets selected by jumpers
#define F1 10000000 // Z1
#define F2 40000000 // Z2
#define F3 5357000  // Z1 & Z2

// high-precision frequency lock (<1Hz) indicator
#define LED A0

// rotary encoder
#define encoderButton A2
#define ENCODER_CH_A 3
#define ENCODER_CH_B 7

// menu stages
#define MENU_ITEM_MAIN 0
#define MENU_ITEM_BANK 1
#define MENU_ITEM_STEP 2
#define MENU_ITEM_FREQ 3
#define MENU_ITEM_CORR 4
#define MENU_ITEM_ZONE 5

// frequency update fsm
#define FREQ_UPDATE_DONE  0
#define FREQ_CORR_UPDATED 1
#define FREQ_MAN_UPDATED  2


volatile bool fired;
volatile bool up;

unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;
long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0, Freq = 10000000;
int second = 0, minute = 0, hour = 0;
int day = 0, month = 0, year = 0;
int zone = 0;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
boolean LOCK_ACHEIVED = false;
byte new_freq = FREQ_CORR_UPDATED;
unsigned long freq_step = 1000;
byte encoderOLD, menu = 0, bank = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;
float stab_float = 1000;

void rotaryEncoderIsr() {
  cli();
  if (digitalRead (ENCODER_CH_A))
    up = digitalRead (ENCODER_CH_B);
  else
    up = !digitalRead (ENCODER_CH_B);
  fired = true;
  delay(150);
  sei();
}

void setup() {
  Serial.begin(9600);
  oled.begin(&Adafruit128x64, CS_PIN, DC_PIN, RST_PIN);
  oled.setFont(SystemFont5x7);  //ZevvPeep8x16  X11fixed7x14
  oled.clear();

  pinMode(ENCODER_CH_A, INPUT);              
  digitalWrite(ENCODER_CH_A, HIGH);
  pinMode(ENCODER_CH_B, INPUT);
  digitalWrite(ENCODER_CH_B, HIGH);
  pinMode(encoderButton, INPUT_PULLUP);                  
  pinMode(Z1, INPUT_PULLUP);
  pinMode(Z2, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  if (!digitalRead(Z1)) Freq = F1;
  if (!digitalRead(Z2)) Freq = F2;
  if ((!digitalRead(Z1))&& (!digitalRead(Z2))) Freq = F3;

  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  pinMode(ppsPin, INPUT);                        // Inititalize GPS 1pps input
  digitalWrite(ppsPin, HIGH);

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);

  Serial.begin(9600);

  // Set CLK0 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_freq(250000000ULL, SI5351_CLK0);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK1);
  si5351.update_status();

  if (digitalRead(encoderButton) == 0) {
    oled.setCursor(0, 0);
    oled.println("setting defaults");
    EEPROM.writeLong(1 * 4, 10000000);
    EEPROM.writeLong(2 * 4, 40000000);
    EEPROM.writeLong(3 * 4, 1000000);
    EEPROM.writeLong(4 * 4, 100000000);
    EEPROM.writeLong(5 * 4, 145000000);
    delay(3000);
  }

  oled.println("GPSDO");
  oled.println("based on projects by");
  oled.println("W3PM & SQ1GU");
  delay(4000);
  oled.clear();
  
  oled.setCursor(0, 0);
  oled.print("waiting for GPS");

  processGps(6000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    oled.setCursor(0, 2);
    oled.print("GPS failure");
    oled.setCursor(0, 4);
    oled.print ("check wiring! ");
    delay(5000);
    GPSstatus = false;
  }
  oled.clear();
  if (GPSstatus == true) {
    oled.setCursor(0, 2);
    oled.print("waiting for sats");
    printDateTime();
    printGpsDetails();
    do {
      processGps(1000);
    } while (gps.satellites.value() == 0);

    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();

    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
  
    oled.clear();
    printDateTime();
    printGpsDetails();
    attachInterrupt(0, PPSinterrupt, RISING);
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
  printFrequency();
  printGpsDetails();
  printDateTime();

  pinMode (ENCODER_CH_A, INPUT_PULLUP);     // enable pull-ups
  pinMode (ENCODER_CH_B, INPUT_PULLUP); 
  attachInterrupt (digitalPinToInterrupt (ENCODER_CH_A), rotaryEncoderIsr, FALLING);
}

void loop() {
  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }
  if (tcount < 4 ) {
    processGps(0);
  }
  if (gps.time.isUpdated()) {
    hour = (gps.time.hour() + zone) %24 ; // zmiany czasu
    minute = gps.time.minute();
    second = gps.time.second();
  }
  
  if (gps.date.isUpdated()) {
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
  }
  
  if (gps.satellites.isUpdated() && menu == 0) {
    printGpsDetails();
  }

  if (new_freq == FREQ_CORR_UPDATED) {
    correct_si5351a();
    new_freq = FREQ_UPDATE_DONE;
    if (abs(stab_float)<1)  {
      printHighPrecMarker(true);
      LOCK_ACHEIVED = true;
    }
    if (abs(stab_float)>1) {
      printHighPrecMarker(false);
      LOCK_ACHEIVED = false;
    }
  }

  if (new_freq == FREQ_MAN_UPDATED) {
    update_si5351a();
    printFrequency();
    new_freq = FREQ_UPDATE_DONE;
  }

  if (digitalRead(encoderButton) == 0) {
    delay(50);
    if (digitalRead(encoderButton) == 0) {
      menu++;
      if (menu > MENU_ITEM_ZONE) menu = MENU_ITEM_MAIN;
      oled.clear();
      oled.setCursor(0, 2);
      switch (menu) {
        case MENU_ITEM_MAIN:
          printGpsDetails();
          printDateTime();
          printFrequency();
          printHighPrecMarker(LOCK_ACHEIVED);
          time_enable = true;
          break;
        case MENU_ITEM_BANK:
          printBank();
          break;
        case MENU_ITEM_STEP:
          printStep();
          break;
        case MENU_ITEM_FREQ:
          printFrequency();
          break;
        case MENU_ITEM_CORR:
          EEPROM.writeLong(bank * 4, Freq);
          printStabilityInfo();
          break;
        case MENU_ITEM_ZONE:
          printTimezone();
          break;
      }
      delay(200);
    }
  }

  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
    time_enable = false;
    oled.setCursor (15, 0);
  }

  ENCread();
}

//**************************************************************************************
//                       INTERRUPT  ENC
//**************************************************************************************
void ENCread() {
  if (fired) {
      if (up) {
        switch (menu) {
          case MENU_ITEM_BANK: {
              bank++;
              if (bank > 5) bank = 5;
              printBank();
            }
            break;
          case MENU_ITEM_STEP: {
              f_step++;
              if (f_step > 8) f_step = 8;
              printStep();
            }
            break;
          case MENU_ITEM_FREQ: {
              Freq += freq_step;
              if (Freq > MAX_FREQ) Freq -= freq_step;
              new_freq = FREQ_MAN_UPDATED;
            }
            break;
          case MENU_ITEM_ZONE: {
              zone++;
              if (zone > 6)zone = 5;
              EEPROM.writeInt(80, zone);
              printTimezone();
            }
            break;
        }
      }
      else {
        switch (menu) {
          case MENU_ITEM_BANK: {
              bank--;
              if (bank == 0) bank = 1;
              printBank();
            }
            break;
          case MENU_ITEM_STEP: {
              f_step--;
              if (f_step == 0 ) f_step = 1;
              printStep();
            }
            break;
          case MENU_ITEM_FREQ: {
              Freq -= freq_step;
              if (Freq > MAX_FREQ || Freq < MIN_FREQ) Freq += freq_step;
              new_freq = FREQ_MAN_UPDATED;
            }
            break;
          case MENU_ITEM_ZONE: {
              zone--;
              if (zone < -6) zone = -5;
              EEPROM.writeInt(80, zone);
              printTimezone();
            }
            break;
        }
      }
      fired = false;
    }
}
//**************************************************************************************
//                       INTERRUPT  1PPS
//**************************************************************************************
void PPSinterrupt()
{
  tcount++;
  stab_count--;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
    // loop();
  }
  
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;                                  //Turn off counter
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
      XtalFreq = mult * 0x10000 + TCNT1;           //Calculate correction factor
      new_freq = FREQ_CORR_UPDATED;
    }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;
    Serial.begin(9600);
    stab_count = 44;
    printStabilityInfo();
  }

  if (validGPSflag == 1)                      //Start the UTC timekeeping process
  {
    second++;
    if (second == 60)                            //Set time using GPS NMEA data
    {
      minute++ ;
      second = 0 ;
    }
    if (minute == 60)
    {
      hour++;
      minute = 0 ;
    }
    if (hour == 24) hour = 0 ;
    if (time_enable) printDateTime();
  }
  if (menu == MENU_ITEM_CORR) {
    oled.setCursor(96, 6);
    if (stab_count < 10) oled.print(" ");
    oled.print(stab_count);
  }
}
//*******************************************************************************
// Timer 1 overflow intrrupt vector.
//*******************************************************************************
ISR(TIMER1_OVF_vect) {
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}

void printTimezone() {
  time_enable = false;
  oled.setCursor(0, 2);
  oled.print("time zone ");
  if (zone > 0) oled.print("+");
  oled.print(zone);
  oled.print(" < > ");
}

void printStabilityInfo() {
  time_enable = false;
  stab_float = getFreqCorrection(); 
  if (menu == MENU_ITEM_CORR) {
    oled.setCursor(0, 0);
    oled.print("freq. correction ");
    oled.setCursor(0, 2);
    oled.print("   ");
    oled.print(stab_float);
    oled.print(" Hz        ");
  }
}

float getFreqCorrection() {
  long pomocna;
  stab = XtalFreq - 100000000;
  stab *= 10 ;
  if (stab > 100 || stab < -100) {
    correction = correction + stab;
  } else if (stab > 20 || stab < -20) {
    correction += stab / 2;
  } else {
    correction += stab / 4;
  }
  pomocna = 10000 / (Freq / 1000000);
  stab *= 100;
  stab /= pomocna;
  stab_float = float(stab);
  return stab_float = stab_float / 10;
}

void printFrequency2() {
  time_enable = false;
  oled.setCursor(0, 2);
  oled.print("freq. bank ");
  oled.print(bank);
  oled.print(" < > ");
}

void printStep() {
  time_enable = false;
  oled.setCursor(0, 2);
  oled.print("step ");
  switch (f_step) {
    case 1: freq_step = 1, oled.print("   1 Hz");
      break;
    case 2: freq_step = 10, oled.print("  10 Hz");
      break;
    case 3: freq_step = 100, oled.print(" 100 Hz");
      break;
    case 4: freq_step = 1000, oled.print("  1 kHz");
      break;
    case 5: freq_step = 10000, oled.print(" 10 kHz");
      break;
    case 6: freq_step = 100000, oled.print("100 kHz");
      break;
    case 7: freq_step = 1000000 , oled.print("  1 MHz");
      break;
    case 8: freq_step = 10000000, oled.print(" 10 MHz");
      break;
  }
}

void printBank() {
  time_enable = false;
  oled.setCursor(0, 2);
  oled.print("bank ");
  oled.print(bank);
  oled.print("      < > ");
  Freq = EEPROM.readLong(bank * 4);
  printFrequency();
  update_si5351a();
}

void printDateTime() {
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d   %02d.%02d.%02d", hour, minute, second, day, month, year);
  oled.setCursor(0, 0);
  oled.print(sz);
}

void printGpsDetails()
{
  time_enable = false;
  oled.setCursor(0, 2);
  oled.print("sat=");
  oled.print(gps.satellites.value());
  TinyGPSHDOP hdop = gps.hdop;
  if (hdop.isValid()) {
    oled.print(" hdop=");
    oled.print(hdop.value());
    oled.print(" age=");
    oled.print(hdop.age());
    oled.println("ms      ");
  } else {
    oled.println("");
  }

  TinyGPSLocation location = gps.location;
  if (location.isValid()) {
    oled.print("lon=");
    oled.print(location.lng());
    oled.print(" lat=");
    oled.println(location.lat());
  }
  
  TinyGPSAltitude altitude = gps.altitude;
  if (altitude.isValid()) {
    oled.print("alt=");
    oled.print(altitude.meters());
    oled.println("m");
  }

  time_enable = true;
}

void printFrequency() {
  time_enable = false;
  oled.setCursor(16, 6);
  oled.print(Freq);
  oled.print(" Hz   ");
}

void printHighPrecMarker(bool enabled) {
  oled.setCursor(112, 6);
  if (enabled) {
    oled.print("@");
    digitalWrite(A0, HIGH);
  } else {
    oled.print(" ");
    digitalWrite(A0, LOW);
  }
}

void update_si5351a() {
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK1);
}

void correct_si5351a() {
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);
}

void processGps(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial.available()) {
      gps.encode(Serial.read());
    }
  } while (millis() - start < ms);
}
