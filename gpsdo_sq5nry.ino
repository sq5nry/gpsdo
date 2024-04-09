/*

 Arduino Controlled GPS Corrected Generator
 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.

 Based on the projects: 
 W3PM (http://www.knology.net/~gmarcus/)
 &
 SQ1GU (http://sq1gu.tobis.com.pl/pl/syntezery-dds/44-generator-si5351a)
 
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


// The TinyGPS++ object
TinyGPSPlus gps;

// The Si5351 object
Si5351 si5351;

// Zworki
#define Z1 4
#define Z2 6
#define LED A0

// Definicje czestotliwosci dla zworek Z1 i Z2 oraz Z1&Z2
#define F1 10000000 // zwora Z1
#define F2 40000000 // zwora Z2
#define F3 25000000  // zwora Z1&Z2

#define ppsPin                   2
#define przycisk                 A2

#define CHA 3
#define CHB 7
volatile bool fired;
volatile bool up;
const byte encoderPinA = 3;
const byte encoderPinB = 7;


// OLED Pins
#define CS_PIN  10
#define RST_PIN 9
#define DC_PIN  8

SSD1306AsciiSpi lcd;

unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;
long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0, Freq = 10000000;
int second = 0, minute = 0, hour = 0;
int day = 0, month = 0, year = 0;
int zone = 2;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
boolean fixed = false;
byte new_freq = 1;
unsigned long freq_step = 1000;
byte encoderOLD, menu = 0, band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;
float stab_float = 1000;


void isr()
{
  cli();
  if (digitalRead (encoderPinA))
    up = digitalRead (encoderPinB);
  else
    up = !digitalRead (encoderPinB);
  fired = true;
  sei();
}  // end of isr


//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
  Serial.begin(9600);
  lcd.begin(&Adafruit128x64, CS_PIN, DC_PIN, RST_PIN);
  lcd.setFont(ZevvPeep8x16);
  lcd.clear();


  pinMode(encoderPinA, INPUT);              
  digitalWrite(encoderPinA, HIGH);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);

  pinMode(przycisk, INPUT_PULLUP);                  
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
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);

  Serial.begin(9600);

  // Set CLK0 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_freq(250000000ULL, SI5351_CLK0);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK1);
  si5351.update_status();

  if (digitalRead(przycisk) == 0) {

    lcd.setCursor(0, 2);
    lcd.print(" Initialization");
    EEPROM.writeLong(1 * 4, 10000000);
    EEPROM.writeLong(2 * 4, 20000000);
    EEPROM.writeLong(3 * 4, 24000000);
    EEPROM.writeLong(4 * 4, 40000000);
    EEPROM.writeLong(5 * 4, 145000000);
    delay(3000);
  }

  lcd.println();
  lcd.print(" GPS GENERATOR");
  delay(1000);
  lcd.clear();
  lcd.setCursor(36, 0);
  lcd.print("Based on");
  lcd.setCursor(4, 2);
  lcd.print("the projects by");
  lcd.setCursor(20, 4);
  lcd.print("W3PM & SQ1GU");
  delay(2000);
  lcd.clear();
  
  lcd.setCursor(4, 2);
  lcd.print("Waiting for GPS");

  GPSproces(6000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    lcd.setCursor(0, 2);
    lcd.print("GPS not connected");
    lcd.setCursor(0, 4);
    lcd.print (" check wiring! ");
    delay(5000);
    GPSstatus = false;
  }
  lcd.clear();
  if (GPSstatus == true) {
    lcd.setCursor(4, 2);
    lcd.print("Waiting for SAT");
    time_on_lcd();
    date_on_lcd();
    sat_on_lcd();
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);

    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();

    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();

    
    lcd.clear();
    time_on_lcd();
    sat_on_lcd();
    attachInterrupt(0, PPSinterrupt, RISING);
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
  freq_on_lcd();
  sat_on_lcd();
  time_on_lcd();
  date_on_lcd();

  
  pinMode (encoderPinA, INPUT_PULLUP);     // enable pull-ups
  pinMode (encoderPinB, INPUT_PULLUP); 
  attachInterrupt (digitalPinToInterrupt (encoderPinA), isr, CHANGE);
}
//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{

  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }
  if (tcount < 4 ) {
    GPSproces(0);
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
    sat_on_lcd();
  }

  if (new_freq == 1) {
    correct_si5351a();
    new_freq = 0;
    if (abs(stab_float)<1)  {
      lcd.setCursor(112, 4);
      lcd.print("@");
      digitalWrite(LED, HIGH);
      fixed = true;
    }
    if (abs(stab_float)>1) {
      lcd.setCursor(112, 4);
      lcd.print(" ");
      digitalWrite(LED, LOW);
      fixed = false;
    }
  }

  if (new_freq == 2) {
    update_si5351a();
    freq_on_lcd();
    new_freq = 0;
  }

  if (digitalRead(przycisk) == 0) {
    delay(10);
    if (digitalRead(przycisk) == 0) {
      menu++;
      if (menu > 5) menu = 0;
      lcd.clear();
      lcd.setCursor(0, 2);
      switch (menu) {
        case 0:
          sat_on_lcd();
          date_on_lcd();                              
          time_on_lcd();
          freq_on_lcd();
          if (fixed)  {
            lcd.setCursor(112, 4);
            lcd.print("@");
          }
          else {
            lcd.setCursor(112, 4);
            lcd.print(" ");
          }
          time_enable = true;
          break;
        case 1:
          band_on_lcd();
          break;
        case 2:
          step_on_lcd();
          break;
        case 3:
          freq2_on_lcd();
          break;
        case 4:
          EEPROM.writeLong(band * 4, Freq);
          stab_on_lcd();
          break;
        case 5:
          timezone_on_lcd();
          break;
      }
      delay(200);
    }
  }

  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
    time_enable = false;
    lcd.setCursor (15, 0);
  }

  ENCread();
}

//**************************************************************************************
//                       INTERRUPT  ENC
//**************************************************************************************
void ENCread()
{
  if (fired) {
      if (up) {

        switch (menu) {
          case 1: {
              band++;
              if (band > 5) band = 5;
              band_on_lcd();
            }
            break;
          case 2: {
              f_step++;
              if (f_step > 8)f_step = 8;
              step_on_lcd();
            }
            break;
          case 3: {
              Freq += freq_step;
              if (Freq > 160000000) Freq -= freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone++;
              if (zone > 6)zone = 5;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
            }
            break;
        }
      }
      else {
        switch (menu) {
          case 1: {
              band--;
              if (band == 0) band = 1;
              band_on_lcd();
            }
            break;
          case 2: {
              f_step--;
              if (f_step == 0 )f_step = 1;
              step_on_lcd();
            }
            break;
          case 3: {
              Freq -= freq_step;
              if (Freq > 160000000 || Freq < 1000) Freq += freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone--;
              if (zone < -6)zone = -5;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
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
      new_freq = 1;
    }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;
    Serial.begin(9600);
    stab_count = 44;
    stab_on_lcd();
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
    if (time_enable) time_on_lcd();
  }
  if (menu == 4) {
    lcd.setCursor(96, 6);
    if (stab_count < 10) lcd.print(" ");
    lcd.print(stab_count);
  }
}
//*******************************************************************************
// Timer 1 overflow intrrupt vector.
//*******************************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}


//********************************************************************************
//                                TIMEZONE on LCD <>
//********************************************************************************
void timezone_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 2);
  lcd.print("TIME zone ");
  if (zone > 0) lcd.print("+");
  lcd.print(zone);
  lcd.print(" < > ");
}
//********************************************************************************
//                                STAB on LCD stabilnośc częstotliwości
//********************************************************************************
void stab_on_lcd() {
  long pomocna;
  time_enable = false;
  stab = XtalFreq - 100000000;
  stab = stab * 10 ;
  if (stab > 100 || stab < -100) {
    correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
    correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  pomocna = (10000 / (Freq / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;
  if (menu == 4) {
    lcd.setCursor(0, 0);
    lcd.print("Freq. correction ");
    lcd.setCursor(0, 2);
    lcd.print("   ");
    lcd.print(stab_float);
    lcd.print(" Hz        ");
  }
}

//********************************************************************************
//                                FREQ_2 on LCD <>
//********************************************************************************
void freq2_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 2);
  lcd.print("FREQ Bank ");
  lcd.print(band);
  lcd.print(" < > ");
}
//********************************************************************************
//                                STEP on LCD
//********************************************************************************
void step_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 2);
  lcd.print("STEP ");
  switch (f_step) {
    case 1: freq_step = 1, lcd.print("   1 Hz");
      break;
    case 2: freq_step = 10, lcd.print("  10 Hz");
      break;
    case 3: freq_step = 100, lcd.print(" 100 Hz");
      break;
    case 4: freq_step = 1000, lcd.print("  1 kHz");
      break;
    case 5: freq_step = 10000, lcd.print(" 10 kHz");
      break;
    case 6: freq_step = 100000, lcd.print("100 kHz");
      break;
    case 7: freq_step = 1000000 , lcd.print("  1 MHz");
      break;
    case 8: freq_step = 10000000, lcd.print(" 10 MHz");
      break;
  }
}
//********************************************************************************
//                                BAND on LCD
//********************************************************************************
void band_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 2);
  lcd.print("BANK ");
  lcd.print(band);
  lcd.print("      < > ");
  Freq = EEPROM.readLong(band * 4);
  freq_on_lcd();
  update_si5351a();
}
//********************************************************************************
//                                TIME on LCD
//********************************************************************************
void time_on_lcd()
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);
    lcd.setCursor(30, 0);
  lcd.print(sz);
}

//********************************************************************************
//                                DATE on LCD - podmiana F cor
//********************************************************************************

void date_on_lcd()
{
 char da[32];
  sprintf(da, "%02d/%02d/%02d ", day, month, year);
  lcd.setCursor(22, 2);
  lcd.print(da);
}

//********************************************************************************
//                                SAT nr. on LCD
//********************************************************************************
void sat_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 4);
  lcd.print("SAT: ");
  lcd.print(gps.satellites.value());
  lcd.print("   ");
  time_enable = true;
}

//*********************************************************************************
//                             Freq on LCD
//*********************************************************************************
void freq_on_lcd() {
  char buf[10]; /// tutaj zmiana

  // Print frequency to the LCD
 
  ltoa(Freq, buf, 10);
  time_enable = false;
    //lcd.home();
  
    lcd.setCursor(1,6);
 // lcd.print(Freq);
    
 if (Freq < 1000000)
  {
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
  }

  if (Freq >= 1000000 && Freq < 10000000)
  {
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print('.');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
  }

  if (Freq >= 10000000 && Freq < 100000000)
  {
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print('.');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
  }

  if (Freq >= 100000000)
  {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print('.');
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(buf[8]);
  }
  lcd.print(" Hz ");
}

//********************************************************************
//             NEW frequency
//********************************************************************
void update_si5351a()
{
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK1);
}
//********************************************************************
//             NEW frequency correction
//********************************************************************
void correct_si5351a()
{
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);
}
//*********************************************************************
//                    Odczyt danych z GPS
//**********************************************************************
static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}
//*********************************************************************
