/*
    This sketch sends ads1115 current sensor data via HTTP POST request to thingspeak server.
    It needs the following libraries to work (besides the esp8266 standard libraries supplied with the IDE):

    - https://github.com/adafruit/Adafruit_ADS1X15

    designed to run directly on esp8266-01 module, to where it can be uploaded using this marvelous piece of software:

    https://github.com/esp8266/Arduino

    2015 Tisham Dhar
    licensed under GNU GPL
*/

//#include "adc.h"
//#include <Wire.h>
//#include <Adafruit_GFX_AS.h>
//#include <Adafruit_SSD1306_STM32.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel-ANDnXOR.h>
#include <EEPROM.h>

//#define Serial Serial2

//Adafruit_SSD1306 display(-1);

volatile int interruptCounter;

const char vers[] = "2.0-04092019"; 

#define AFTERDISWAIT 300//300 //300s after charging wait time
#define AFTERCHGWAIT 60//60 //60s after charging wait time
#define LOOP_PERIOD 500 //In microseconds
#define CHARGE 1
#define DISCHARGE 2
#define DISCONNECT 0
#define REGEN 3
#define OFF 0
#define RED 1
#define YELLOW 2
#define GREEN 3
#define CYAN 4
#define BLUE 5
#define PURPLE 6
#define WHITE 7
#define MAX_CHG_CUR -10000
#define MAX_CHG_VOL 4500
#define MIN_CCC 10
#define MAX_DIS_CUR 10000
#define MIN_DIS_VOL 1000
#define MAX_DIS_VOL 4300
#define MINBUCKDUTY 1
#define MAXBUCKDUTY 397
#define DEF_CHG_CUR -1500
#define DEF_CHG_VOL 4200
#define DEF_CCC 50
#define DEF_DIS_CUR 1500
#define DEF_DIS_VOL 2700
#define DEF_DIS_MODE 0
#define DEF_PSU_MODE 1
#define DEF_CYCLES 1
#define OVT_THRESH 45
#define LED_BRIGHTNESS 80 //1-255 for LED brightness
volatile int16 charge_current_1 = -1500;
volatile uint16 charge_voltage_1 = 4200;
volatile uint16 discharge_current_1 = 1500;
volatile uint16 discharge_voltage_1 = 2700;
volatile uint16 ccc_1 = 50; //Charge CC cutoff in mA (50mA)
volatile uint16 num_cycles_1 = 1;
volatile uint8 discharge_mode_1 = 0;
volatile uint8 psu_dir_1 = 0;
volatile uint16 tm_duty_1 = 0;
volatile int16 charge_current_2 = -1500;
volatile uint16 charge_voltage_2 = 4200;
volatile uint16 discharge_current_2 = 1500;
volatile uint16 discharge_voltage_2 = 2700;
volatile uint16 ccc_2 = 50; //Charge CC cutoff in mA (50mA)
volatile uint16 num_cycles_2 = 1;
volatile uint8 discharge_mode_2 = 0;
volatile uint8 psu_dir_2 = 0;
volatile uint16 tm_duty_2 = 0;
volatile uint8 slot1_startup = 0;
volatile uint8 slot2_startup = 0;

#define STARTUP_CYCLES 2 //number of cycles-1 (0.25ms each) to delay before turning on cell

//Initialize reference voltage with default until read out of EEPROM
#define REFAVALINIT 2048000.0f
volatile float REFAVAL = REFAVALINIT;
//EEPROM address for reference value: 0
#define REFAVALADDR 0
//#define REFAVAL 2037325.6f //Board 2: Real ref voltage = 1.6425 (on), 1.6403 (off). Value = ADC val with 3.3V*1000 (board 1)
//#define REFAVAL 2059481.2f //Board 2: Real ref voltage = 1.68 (on), 1.6385 (off). Value = ADC val with 3.3V*1000 (board 2)
//Original ADC2I = 620.606
//Original ADC2V = 846.281
//Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
//Voltage reads high => increase ADC2V value
//Current reads high => increase ADC2I value
//EEPROM address for slot 1 voltage value: 1
#define ADC2V1ADDR 1
#define ADC2V1INIT 842.452f
//EEPROM address for slot 2 voltage value: 2
#define ADC2V2ADDR 2
#define ADC2V2INIT 842.452f
//todo: reset adc2v2init during "l" calibration for storing correction factors
//EEPROM address for slot 1 current value: 3
#define ADC2I1ADDR 3
#define ADC2I1INIT 310.303f
//EEPROM address for slot 2 current value: 4
#define ADC2I2ADDR 4
#define ADC2I2INIT 310.303f
//Board 1
/*#define ADC2I1INIT 313.763f
#define ADC2V1INIT 843.332f
#define ADC2I2INIT 323.219f
#define ADC2V2INIT 844.267f*/
//Board 2
/*#define ADC2I1INIT 318.608f
#define ADC2V1INIT 839.542f
#define ADC2I2INIT 316.440f
#define ADC2V2INIT 843.087f*/
//EEPROM address for buffer current value: 5
#define BUF2VADDR 5
#define BUF2VINIT 3.22266f //x/4096*3.3*1000*4
volatile float BUF2V = BUF2VINIT;
#define VR12V 1.61790f
#define VR22V 1.61694f
volatile float ADC2I1 = ADC2I1INIT;
volatile float ADC2V1 = ADC2V1INIT;
volatile float ADC2I2 = ADC2I2INIT;
volatile float ADC2V2 = ADC2V2INIT;
volatile unsigned int initbuckduty1 = 160;
volatile unsigned int initbuckduty2 = 160;
volatile float corr_factor = 1.0f;
#define TOFFS1 0.0f
#define TOFFS2 0.0f
#define TOFFSHS1 0.0f
#define TOFFSHS2 0.0f

//Pin definitions - Analog inputs
#define AC1V PB0 //Cell 1 voltage input
#define AC1A PB1 //Cell 1 current input
#define AC1T PA3 //Cell 1 temperature input
#define AC1R PA1 //Cell 1 reverse voltage input
#define AC2V PA6 //Cell 2 voltage input
#define AC2A PA7 //Cell 2 current input
#define AC2T PA2 //Cell 2 temperature input
#define AC2R PA0 //Cell 2 reverse voltage input
#define ABUFV PA4 //Buffer/input voltage input
#define AIREF PA5 //Reference voltage input
//Pin definitions - Digital outputs
#define HSTH1 0
#define HSTH2 1
#define BUFV 2
//CCR v2.2 HW configuration:
#define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
#define AUXSEL1 PB12 //Lo = ABUFV, Hi = HS thermistor #1
#define OC1PF PA8 //Buck FET control output, positive logic; higher PWM duty = higher output voltage
#define OC1ON PA9 //Buck disable, active low (0 = buck disabled)
#define OC2PF PB10 //Buck FET control output, positive logic; higher PWM duty = higher output voltage
#define OC2ON PB8 //Buck disable, active low (0 = buck disabled)
#define OC1NF2 PB6 //CC 1 setting output
#define OC2NF2 PB7 //CC 2 setting output
#define C1DOFF PB15 //CC 1 disable output
#define C2DOFF PA15 //CC 2 disable output
#define C1ON PB14 //Cell 1 protection bypass
#define C2ON PB3 //Cell 2 protection bypass
#define FANON PB4 //Fan control output
#define WSDI PB13 //LED control pin, inverted logic
//Pin definitions - Digital inputs
#define S1 PB9 //Switch 1
#define S2 PC13 //Switch 2
/*//CCR v2.0 HW configuration:
#define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
#define AUXSEL1 PB2 //Lo = ABUFV, Hi = HS thermistor #1
#define OC1NF1 PA8 //Boost NFET output
#define OC1PF PA9 //Buck PFET output, negative logic; lower PWM duty = higher output voltage
#define OC2NF1 PA10 //Boost NFET output
#define OC2PF PB8 //Buck PFET output, negative logic; lower PWM duty = higher output voltage
#define OC1NF2 PB6 //CC 1 setting output
#define OC2NF2 PB7 //CC 2 setting output
#define C1DOFF PB15 //CC 1 disable output
#define C2DOFF PA15 //CC 2 disable output
#define C1ON PB14 //Cell 1 protection bypass
#define C2ON PB3 //Cell 2 protection bypass
#define FANON PB4 //Fan control output
#define WSDI PB13 //LED control pin, inverted logic
//Pin definitions - Digital inputs
#define S1 PB9 //Switch 1
#define S2 PC13 //Switch 2*/

Adafruit_NeoPixel leds = Adafruit_NeoPixel(2, WSDI, NEO_GRB + NEO_KHZ800);

/*#if (SSD1306_LCDHEIGHT != 32)
  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
  #endif*/

uint16 adcval1 = 0;
uint16 adcval2 = 0;
uint16 adcval3 = 0;
uint16 adcval4 = 0;
uint16 adcval5 = 0;
uint16 adcval6 = 0;
uint32 adciref = 0;
uint16 vbuf_i = 0;
uint8 loopcnt = 0;
//uint32 interruptcnt = 0;
volatile int16 duty1 = 0;
volatile int16 duty2 = 0;
uint16 loop1 = 0;
uint16 loop2 = 1;
volatile uint8 state1 = 8;
volatile uint8 state2 = 8;
/* Mode List:
    0 = c = Charge
    1 = d = Discharge
    2 = y = Cycle
    3 = p = Power Supply
    4 = t = Test
    5 = n = None (Parking)
*/
volatile uint8 mode1 = 5;
uint8 irstate1 = 0;
uint16 cycle_count_1 = 0;
volatile uint16 settle1 = 0;
int16 vr1 = 0;
int16 vr2 = 0;
int16 vbati1 = 0;
int16 ibati1 = 0;
int16 vbat_now1 = 0;
int16 ibat_now1 = 0;
float tmpfl = 0;
float vbat1 = 0;
float ibat1 = 0;
volatile int vbat_1_1 = 0;
volatile int ibat_1_1 = 0;
float vload1 = 0;
float iload1 = 0;
float voc11 = 0;
float voc21 = 0;
volatile float mah1 = 0;
volatile float mwh1 = 0;
volatile float ir1 = 0;
volatile float temp1 = 0;
volatile uint8 mode2 = 5;
uint8 irstate2 = 0;
uint16 cycle_count_2 = 0;
volatile uint16 settle2 = 0;
int16 vbati2 = 0;
int16 ibati2 = 0;
int16 vbat_now2 = 0;
int16 ibat_now2 = 0;
float vbat2 = 0;
float ibat2 = 0;
volatile int vbat_1_2 = 0;
volatile int ibat_1_2 = 0;
float vload2 = 0;
float iload2 = 0;
float voc12 = 0;
float voc22 = 0;
volatile float mah2 = 0;
volatile float mwh2 = 0;
volatile float ir2 = 0;
volatile float temp2 = 0;
volatile float temp_t = 0;
volatile uint16 adctemp = 0;
volatile uint16 devicepwr = 0;
//HardwareTimer timer1(1);
//HardwareTimer timer2(2);

//y 1500 2700 0 1500 4200 1000 1\r\n
//32 chars with /r/n + 8 corrections (e.g. 4 backspace + 4 chars)
#define MAXCHARS 40
char receivedChars[MAXCHARS];
boolean newData = false;

unsigned short getAuxADC(unsigned char adcsel) {
  switch (adcsel)
  {
    /*#define ABUFV PA4
      #define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
      #define AUXSEL1 PB2 //Lo = ABUFV, Hi = HS thermistor #1
      #define HSTH1 0
      #define HSTH2 1
      #define BUFV 2*/
    case HSTH1:
      pinMode(AUXSEL1, OUTPUT);
      digitalWrite(AUXSEL1, HIGH);
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, HIGH);
      return analogRead(ABUFV);
      break;
    case HSTH2:
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, LOW);
      return analogRead(ABUFV);
      break;
    case BUFV:
      pinMode(AUXSEL1, OUTPUT);
      digitalWrite(AUXSEL1, LOW);
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, HIGH);
      return analogRead(ABUFV);
      break;
    default:
      pinMode(AUXSEL1, OUTPUT);
      digitalWrite(AUXSEL1, HIGH);
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, HIGH);
      return analogRead(ABUFV);
      break;
  }
}

unsigned short getCell1RV(){
  return (int)(((float)analogRead(AC1R)) * VR12V);
}

unsigned short getCell2RV(){
  return (int)(((float)analogRead(AC2R)) * VR22V);
}

void setChg1(unsigned char state) {
  switch (state)
  {
    /*#define OC1NF1 PA8 //Boost NFET
      #define OC1PF PA9 //Buck PFET
      #define OC1NF2 PB6 //CC setting
      #define C1DOFF PB15*/
    case DISCONNECT:
      pinMode(OC1ON, OUTPUT);
      digitalWrite(OC1ON, LOW);
      pinMode(OC1PF, OUTPUT);
      digitalWrite(OC1PF, LOW);
      pinMode(OC1NF2, OUTPUT);
      digitalWrite(OC1NF2, LOW);
      pinMode(C1DOFF, OUTPUT);
      digitalWrite(C1DOFF, HIGH);
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
      break;
    case CHARGE:
      pinMode(OC1NF2, OUTPUT);
      digitalWrite(OC1NF2, LOW);
      pinMode(C1DOFF, OUTPUT);
      digitalWrite(C1DOFF, HIGH); //Inverting
      presetDuty1();
      duty1 = initbuckduty1; //Assume 3.7V initial cell voltage - optimise later
      pinMode(OC1PF, PWM);
      pwmWrite(OC1PF, duty1);
      pinMode(OC1ON, OUTPUT);
      digitalWrite(OC1ON, HIGH);
      slot1_startup = STARTUP_CYCLES;
      digitalWrite(C1ON, HIGH); //override cell protection FET off until synchronous buck is started
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
      break;
    case DISCHARGE:
      pinMode(OC1ON, OUTPUT);
      digitalWrite(OC1ON, LOW);
      pinMode(OC1PF, OUTPUT);
      digitalWrite(OC1PF, LOW); //Non-inverting mode
      duty1 = 0;
      pinMode(OC1NF2, PWM);
      pwmWrite(OC1NF2, duty1);
      pinMode(C1DOFF, OUTPUT);
      digitalWrite(C1DOFF, LOW); //Non-inverting mode
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, HIGH); //Non-inverting mode
      break;
    case REGEN:
      pinMode(OC1NF2, OUTPUT);
      digitalWrite(OC1NF2, LOW);
      pinMode(C1DOFF, OUTPUT);
      digitalWrite(C1DOFF, HIGH); //Inverting
      presetDuty1();
      duty1 = initbuckduty1; //Assume 3.7V initial cell voltage - optimise later
      pinMode(OC1PF, PWM);
      pwmWrite(OC1PF, duty1);
      pinMode(OC1ON, OUTPUT);
      digitalWrite(OC1ON, HIGH);
      slot1_startup = STARTUP_CYCLES;
      digitalWrite(C1ON, HIGH); //override cell protection FET off until synchronous buck is started
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
    default:
      pinMode(OC1ON, OUTPUT);
      digitalWrite(OC1ON, LOW);
      pinMode(OC1PF, OUTPUT);
      digitalWrite(OC1PF, LOW);
      pinMode(OC1NF2, OUTPUT);
      digitalWrite(OC1NF2, LOW);
      pinMode(C1DOFF, OUTPUT);
      digitalWrite(C1DOFF, HIGH);
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
      break;
  }
}

void setChg2(unsigned char state) {
  switch (state)
  {
    /*#define OC2NF1 PA10 //Boost NFET
      #define OC2PF PB8 //Buck PFET
      #define OC2NF2 PB7 //CC setting
      #define C2DOFF PA15*/
    case DISCONNECT:
      pinMode(OC2ON, OUTPUT);
      digitalWrite(OC2ON, LOW);
      pinMode(OC2PF, OUTPUT);
      digitalWrite(OC2PF, LOW);
      pinMode(OC2NF2, OUTPUT);
      digitalWrite(OC2NF2, LOW);
      pinMode(C2DOFF, OUTPUT);
      digitalWrite(C2DOFF, HIGH);
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
      break;
    case CHARGE:
      pinMode(OC2NF2, OUTPUT);
      digitalWrite(OC2NF2, LOW);
      pinMode(C2DOFF, OUTPUT);
      digitalWrite(C2DOFF, HIGH); //Inverting
      presetDuty2();
      duty2 = initbuckduty2; //Assume 3.7V initial cell voltage - optimise later
      pinMode(OC2PF, PWM);
      pwmWrite(OC2PF, duty1);
      pinMode(OC2ON, OUTPUT);
      digitalWrite(OC2ON, HIGH);
      slot2_startup = STARTUP_CYCLES;
      digitalWrite(C2ON, HIGH); //override cell protection FET off until synchronous buck is started
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
      break;
    case DISCHARGE:
      pinMode(OC2ON, OUTPUT);
      digitalWrite(OC2ON, LOW);
      pinMode(OC2PF, OUTPUT);
      digitalWrite(OC2PF, LOW); //Non-inverting mode
      duty2 = 0;
      pinMode(OC2NF2, PWM);
      pwmWrite(OC2NF2, duty1);
      pinMode(C2DOFF, OUTPUT);
      digitalWrite(C2DOFF, LOW); //Non-inverting mode
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, HIGH); //Non-inverting mode
      break;
    case REGEN:
      pinMode(OC2NF2, OUTPUT);
      digitalWrite(OC2NF2, LOW);
      pinMode(C2DOFF, OUTPUT);
      digitalWrite(C2DOFF, HIGH); //Inverting
      presetDuty2();
      duty2 = initbuckduty2; //Assume 3.7V initial cell voltage - optimise later
      pinMode(OC2PF, PWM);
      pwmWrite(OC2PF, duty1);
      pinMode(OC2ON, OUTPUT);
      digitalWrite(OC2ON, HIGH);
      slot2_startup = STARTUP_CYCLES;
      digitalWrite(C2ON, HIGH); //override cell protection FET off until synchronous buck is started
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
    default:
      pinMode(OC2ON, OUTPUT);
      digitalWrite(OC2ON, LOW);
      pinMode(OC2PF, OUTPUT);
      digitalWrite(OC2PF, LOW);
      pinMode(OC2NF2, OUTPUT);
      digitalWrite(OC2NF2, LOW);
      pinMode(C2DOFF, OUTPUT);
      digitalWrite(C2DOFF, HIGH);
      //pinMode(FANON, OUTPUT);
      //digitalWrite(FANON, LOW); //Non-inverting mode
      break;
  }
}

void presetDuty1(void)
{
  unsigned int inputV;
  unsigned int cellV;
  
  inputV = (int)(((float)getAuxADC(BUFV)) * BUF2V);
  //Convert ADC value to value in Volts
  cellV = (int)((((float)analogRead(AC1V)) / ADC2V1) * 1000 + 0.5); //Multiply by 1000 for mV, round
  initbuckduty1 = (int)((float)cellV/(float)inputV*399);
  if(initbuckduty1 < MINBUCKDUTY)
    initbuckduty1 = MINBUCKDUTY;
  else if(initbuckduty1 > MAXBUCKDUTY)
    initbuckduty1 = MAXBUCKDUTY;
}

void presetDuty2(void)
{
  unsigned int inputV;
  unsigned int cellV;
  
  inputV = (int)(((float)getAuxADC(BUFV)) * BUF2V);
  //Convert ADC value to value in Volts
  cellV = (int)((((float)analogRead(AC2V)) / ADC2V2) * 1000 + 0.5); //Multiply by 1000 for mV, round
  initbuckduty2 = (int)((float)cellV/(float)inputV*399);
  if(initbuckduty2 < MINBUCKDUTY)
    initbuckduty2 = MINBUCKDUTY;
  else if(initbuckduty2 > MAXBUCKDUTY)
    initbuckduty2 = MAXBUCKDUTY;
}

void setLED1(unsigned char color) {
/*#define OFF 0
#define RED 1
#define YELLOW 2
#define GREEN 3
#define CYAN 4
#define BLUE 5
#define PURPLE 6*/
  switch (color)
  {
    case OFF:
      leds.setPixelColor(0,0,0,0);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, LOW);
      break;
    case RED:
      leds.setPixelColor(0,LED_BRIGHTNESS,0,0);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, LOW);
      break;
    case YELLOW:
      leds.setPixelColor(0,LED_BRIGHTNESS,LED_BRIGHTNESS,0);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, HIGH);
      //digitalWrite(LED1B, LOW);
      break;
    case GREEN:
      leds.setPixelColor(0,0,LED_BRIGHTNESS,0);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, HIGH);
      //digitalWrite(LED1B, LOW);
      break;
    case CYAN:
      leds.setPixelColor(0,0,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, HIGH);
      //digitalWrite(LED1B, HIGH);
      break;
    case BLUE:
      leds.setPixelColor(0,0,0,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, HIGH);
      break;
    case PURPLE:
      leds.setPixelColor(0,LED_BRIGHTNESS,0,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, HIGH);
      break;
    case WHITE:
      leds.setPixelColor(0,LED_BRIGHTNESS,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, HIGH);
      break;
    default:
      leds.setPixelColor(0,0,0,0);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, LOW);
      break;
  }
}

void setLED2(unsigned char color) {
/*#define OFF 0
#define RED 1
#define YELLOW 2
#define GREEN 3
#define CYAN 4
#define BLUE 5
#define PURPLE 6*/
  switch (color)
  {
    case OFF:
      leds.setPixelColor(1,0,0,0);
      leds.show();
      //digitalWrite(LED2R, LOW);
      //digitalWrite(LED2G, LOW);
      //digitalWrite(LED2B, LOW);
      break;
    case RED:
      leds.setPixelColor(1,LED_BRIGHTNESS,0,0);
      leds.show();
      //digitalWrite(LED2R, HIGH);
      //digitalWrite(LED2G, LOW);
      //digitalWrite(LED2B, LOW);
      break;
    case YELLOW:
      leds.setPixelColor(1,LED_BRIGHTNESS,LED_BRIGHTNESS,0);
      leds.show();
      //digitalWrite(LED2R, HIGH);
      //digitalWrite(LED2G, HIGH);
      //digitalWrite(LED2B, LOW);
      break;
    case GREEN:
      leds.setPixelColor(1,0,LED_BRIGHTNESS,0);
      leds.show();
      //digitalWrite(LED2R, LOW);
      //digitalWrite(LED2G, HIGH);
      //digitalWrite(LED2B, LOW);
      break;
    case CYAN:
      leds.setPixelColor(1,0,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED2R, LOW);
      //digitalWrite(LED2G, HIGH);
      //digitalWrite(LED2B, HIGH);
      break;
    case BLUE:
      leds.setPixelColor(1,0,0,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED2R, LOW);
      //digitalWrite(LED2G, LOW);
      //digitalWrite(LED2B, HIGH);
      break;
    case PURPLE:
      leds.setPixelColor(1,LED_BRIGHTNESS,0,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED2R, HIGH);
      //digitalWrite(LED2G, LOW);
      //digitalWrite(LED2B, HIGH);
      break;
    case WHITE:
      leds.setPixelColor(1,LED_BRIGHTNESS,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED2R, HIGH);
      //digitalWrite(LED2G, HIGH);
      //digitalWrite(LED2B, HIGH);
      break;
    default:
      leds.setPixelColor(1,0,0,0);
      leds.show();
      //digitalWrite(LED2R, LOW);
      //digitalWrite(LED2G, LOW);
      //digitalWrite(LED2B, LOW);
      break;
  }
}

void setup() {
  int i = 0;
  unsigned short int wData = 0, estatus = 0;
  
  setChg1(DISCONNECT);
  setChg2(DISCONNECT);
  disableDebugPorts();
  Serial.begin(115200);
  pinMode(C1ON, OUTPUT);
  pinMode(C2ON, OUTPUT);
  digitalWrite(C1ON, LOW); //Default reverse/low voltage detect override off
  digitalWrite(C2ON, LOW); //Default reverse/low voltage detect override off
  pinMode(AC1T, INPUT_ANALOG); //Temp1
  pinMode(AC1A, INPUT_ANALOG); //Ibat1
  pinMode(AC1V, INPUT_ANALOG); //Vbat1
  pinMode(AC1R, INPUT_ANALOG); //Vrev1
  pinMode(AC2T, INPUT_ANALOG); //Temp2
  pinMode(AC2A, INPUT_ANALOG); //Ibat2
  pinMode(AC2V, INPUT_ANALOG); //Vbat2
  pinMode(AC2R, INPUT_ANALOG); //Vrev2
  pinMode(ABUFV, INPUT_ANALOG); //Vbuf
  pinMode(AIREF, INPUT_ANALOG); //VIref
  adciref = analogRead(AIREF); //Iref voltage

  leds.begin();

  setLED1(OFF);
  setLED2(OFF);

  Timer1.pause();
  //timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer1.setPrescaleFactor(1);
  Timer1.setOverflow(400);
  Timer1.refresh();
  Timer1.resume();

  Timer4.pause();
  //timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer4.setPrescaleFactor(1);
  Timer4.setOverflow(400);
  Timer4.refresh();
  Timer4.resume();

  Timer2.pause(); // Pause the timer while we're configuring it
  Timer2.setPrescaleFactor(1);
  Timer2.setOverflow(36000); //72 MHz/36000 = 2000 Hz
  //Set up an interrupt on channel 1
  Timer2.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer2.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  Timer2.attachCompare1Interrupt(handler_loop);
  Timer2.refresh(); //Refresh the timer's count, prescale, and overflow
  delay(5000);
  //Timer2.resume(); //Start the timer counting

  // No display until HW -> SW I2C issue resolved
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  /*display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    // init done
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.display();*/

  estatus = EEPROM.init();
  if(estatus != 0)
  {
    Serial.print("> EEPROM init error, return value: ");
    Serial.println(estatus, HEX);
  }
  
  Serial.println("> Initializing cal. from EEPROM");
  estatus = EEPROM.read(REFAVALADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 6800 || wData < 6200)
    {
      Serial.println("> Reference cal. out of range, using default.");
    }
    else
    {
      Serial.print("> Reference cal. value found: 1.");
      Serial.print(wData);
      Serial.println("V");
      REFAVAL = (10000.0+(float)wData)*124.1212;
    }
  }
  else
  {
    Serial.println("> Reference cal. value not found, using default.");
  }
  estatus = EEPROM.read(ADC2V1ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4600 || wData < 3800)
    {
      Serial.println("> Slot 1 voltage cal. out of range, using default.");
    }
    else
    {
      Serial.print("> Slot 1 voltage cal. value found: ");
      Serial.print(wData);
      Serial.println("mV");
      //Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
      ADC2V1 = ADC2V1 * 4200.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> Slot 1 voltage cal. value not found, using default.");
  }
  estatus = EEPROM.read(ADC2V2ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4600 || wData < 3800)
    {
      Serial.println("> Slot 2 voltage cal. out of range, using default.");
    }
    else
    {
      Serial.print("> Slot 2 voltage cal. value found: ");
      Serial.print(wData);
      Serial.println("mV");
      //Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
      ADC2V2 = ADC2V2 * 4200.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> Slot 2 voltage cal. value not found, using default.");
  }
  estatus = EEPROM.read(ADC2I1ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 1200 || wData < 800)
    {
      Serial.println("> Slot 1 current cal. out of range, using default.");
    }
    else
    {
      Serial.print("> Slot 1 current cal. value found: ");
      Serial.print(wData);
      Serial.println("mA");
      //Current reported / Current real * ADC2IxINIT orig = ADC2IxINIT new
      ADC2I1 = ADC2I1 * 1000.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> Slot 1 current cal. value not found, using default.");
  }
  estatus = EEPROM.read(ADC2I2ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 1200 || wData < 800)
    {
      Serial.println("> Slot 2 current cal. out of range, using default.");
    }
    else
    {
      Serial.print("> Slot 2 current cal. value found: ");
      Serial.print(wData);
      Serial.println("mV");
      //Current reported / Current real * ADC2IxINIT orig = ADC2IxINIT new
      ADC2I2 = ADC2I2 * 1000.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> Slot 2 current cal. value not found, using default.");
  }
  estatus = EEPROM.read(BUF2VADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 6000 || wData < 4000)
    {
      Serial.println("> Input cal. out of range, using default.");
    }
    else
    {
      Serial.print("> Input cal. value found: ");
      Serial.print(wData);
      Serial.println("mV");
      BUF2V = (5000.0/(float)wData)*BUF2VINIT;
    }
  }
  else
  {
    Serial.println("> Input cal. value not found, using default.");
  }

  Serial.println("> Initializing reference...");
  adciref = 0;
  for(i=0;i<1000;i++)
  {
    adciref += analogRead(AIREF); //Iref voltage
    //Serial.println(adciref);
  }
  Serial.print("> ADC correction factor: ");
  corr_factor = (REFAVAL/((float)adciref));
  Serial.print(corr_factor*100);
  Serial.print(" ");
  ADC2I1 = ADC2I1/corr_factor;
  Serial.print(ADC2I1);
  Serial.print(" ");
  ADC2I2 = ADC2I2/corr_factor;
  ADC2V1 = ADC2V1/corr_factor;
  Serial.println(ADC2V1);
  ADC2V2 = ADC2V2/corr_factor;
  
  Serial.print("> Software version: ");
  Serial.println(vers);
  adciref = (uint32)((float)adciref/1000.0); //Iref voltage
  
  printMenu(mode1);
  
  Timer2.resume(); //Start the timer counting
}

void updateADCRef(){
  int i = 0;
  adciref = 0;
  for(i=0;i<1000;i++)
  {
    adciref += analogRead(AIREF); //Iref voltage
  }
  corr_factor = (REFAVAL/((float)adciref));
  ADC2I1 = ADC2I1/corr_factor;
  ADC2I2 = ADC2I2/corr_factor;
  ADC2V1 = ADC2V1/corr_factor;
  ADC2V2 = ADC2V2/corr_factor;
  adciref = (uint32)((float)adciref/1000.0); //Iref voltage
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarkers[] = {'c', 'd', 'y', 'p', 't', 'n', '?', 'v', 's', 'l', 'r', 'z', 'q'};
  char endMarkers[] = {'\n', '\r'};
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc == '\b')
      Serial.print("\b \b");
    else
      Serial.print(rc);
        
    if (recvInProgress == true) {
      //Serial.println("Receive in progress");
      if (rc == endMarkers[0] || rc == endMarkers[1]){
        //Serial.println("Received endMarker");
        receivedChars[ndx] = '\0'; // terminate the string
        /*Serial.print("ndx=");
          Serial.println(ndx);
          Serial.print("rc=");
          Serial.print(rc);*/
        recvInProgress = false;
        ndx = 0;
        newData = true;
      } 
      else if (rc == '\b') { //Backspace = do not increment character index, do not write new character
        /*Serial.println("Received BS");
        Serial.print("ndx=");
        Serial.println(ndx);
        Serial.print("rc=");
        Serial.print(rc);*/
        if (ndx == 0) {
          ndx = 0;
        }
        else {
          receivedChars[ndx] = 0;
          ndx--;
        }
      }
      else {
        /*Serial.println("Received normal");
          Serial.print("ndx=");
          Serial.println(ndx);
          Serial.print("rc=");
          Serial.print(rc);*/
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= MAXCHARS) {
          ndx = MAXCHARS - 1;
        }
      }
    }

    else if (rc == startMarkers[0] || rc == startMarkers[1] || rc == startMarkers[2] || rc == startMarkers[3] || rc == startMarkers[9] || rc == startMarkers[11]
             || rc == startMarkers[4] || rc == startMarkers[5] || rc == startMarkers[6] || rc == startMarkers[7] || rc == startMarkers[8] || rc == startMarkers[10] || rc == startMarkers[12]) {
      /*Serial.println("Received startMarker");
        Serial.print("ndx=");
        Serial.println(ndx);
        Serial.print("rc=");
        Serial.print(rc);*/
      recvInProgress = true;
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= MAXCHARS) {
        ndx = MAXCHARS - 1;
      }
    }

    else if (rc == endMarkers[0] || rc == endMarkers[1]) {
      Serial.print("\r\n");
      Serial.print("> ");
    }
  }
}

int fast_atoi_leading_pos( const char * p )
{
    int x = 0;
    ++p;
    while (*p >= '0' && *p <= '9') {
        x = (x*10) + (*p - '0');
        ++p;
    }
    return x;
}

int fast_atoi_leading_neg( const char * p )
{
    int x = 0;
    bool neg = false;
    ++p;
    if (*p == '-') {
        neg = true;
        ++p;
    }
    while (*p >= '0' && *p <= '9') {
        x = (x*10) + (*p - '0');
        ++p;
    }
    if (neg) {
        x = -x;
    }
    return x;
}

void printMenu(uint8 menu2) 
{
  switch (menu2) {
    case 0:
      Serial.print("\r\n");
      Serial.println("> Currently Charging");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 1:
      Serial.print("\r\n");
      Serial.println("> Currently Discharging");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 2:
      Serial.print("\r\n");
      Serial.println("> Currently Cycle Testing");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 3:
      Serial.print("\r\n");
      Serial.println("> Currently in Power Supply Mode");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 4:
      Serial.print("\r\n");
      Serial.println("> Currently in Test Mode");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 6:
      Serial.print("\r\n");
      Serial.println("> Currently in IR Test Mode");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    default:
      Serial.print("\r\n");
      Serial.println("> Select Mode:");
      Serial.println(">  Charge");
      Serial.println(">   c[1-2] i[charge current, mA] v[charge voltage, mV] o[cutoff current, mA]");
      Serial.println(">            100-1500, def.1500    2400-4500, def.4200   50-250, def.50");
      Serial.println(">  Discharge");
      Serial.println(">   d[1-2] i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]");
      Serial.println(">            100-1500, def.1500       1000-3900, def.2700   def.0");
      Serial.println(">  Cycle");
      Serial.println(">   y[1-2] i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]");
      Serial.println(">            100-1500, def.1500       1000-3900, def.2700   def.0");
      Serial.println(">          k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]");
      Serial.println(">            100-1500, def.1500    2400-4500, def.4200   50-250, def.50        def.1");
      Serial.println(">  Power Supply");
      Serial.println(">   p[1-2] r[direction: 0 = boost, 1 = buck] v[voltage setting, mV] i[current limit, mA]");
      Serial.println(">            def.1                             0-9500, def.4200       50-1500, def.1500");
      Serial.println(">          o[cutoff current, mA]");
      Serial.println(">           50-250, def.50");
      Serial.println(">  Calibration R/W Mode");
      Serial.println(">   t[r/w] a[address: 0-999] d[data, unsigned int (0-65535)]");
      Serial.println(">  IR Test Mode");
      Serial.println(">   r[1-2] i[test current, mA]");
      Serial.println(">            100-1500, def.1500");
      Serial.println(">  Help (Prints this menu)");
      Serial.println(">   ?");
      Serial.println(">  Version");
      Serial.println(">   v");
      Serial.println(">  Soft Reset");
      Serial.println(">   z");
      Serial.println(">  Status");
      Serial.println(">   s");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
  }
}

void parseCharge1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default charge current/voltage/cutoff
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  ccc_1 = DEF_CCC;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-100)
            strVal = -100;
          charge_current_1 = strVal;
          Serial.print(charge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<2400)
            strVal = 2400;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_1 = strVal;
          Serial.print(charge_voltage_1);
          Serial.println("mV");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_1 = strVal;
          Serial.print(ccc_1);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(charge_voltage_1 == DEF_CHG_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(charge_voltage_1);
    Serial.println("mV");
  }
  if(charge_current_1 == DEF_CHG_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(charge_current_1);
    Serial.println("mA");
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using default cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
}

void parseDischarge1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  discharge_voltage_1 = DEF_DIS_VOL;
  discharge_current_1 = DEF_DIS_CUR;
  discharge_mode_1 = DEF_DIS_MODE;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_DIS_VOL)
            strVal = MIN_DIS_VOL;
          else if(strVal>MAX_DIS_VOL)
            strVal = MAX_DIS_VOL;
          discharge_voltage_1 = strVal;
          Serial.print(discharge_voltage_1);
          Serial.println("mV");
          break;
        case 'm':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          discharge_mode_1 = strVal;
          if(discharge_mode_1 == 0)
            Serial.println("Constant current");
          else
            Serial.println("Stepped");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_1 == DEF_DIS_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(discharge_voltage_1);
    Serial.println("mV");
  }
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(discharge_mode_1 == DEF_DIS_MODE)
  {
    Serial.print("> Using default mode: ");
    if(discharge_mode_1 == 0)
      Serial.println("Constant current");
    else
      Serial.println("Stepped");
  }
}

void parseIR1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current
  discharge_current_1 = DEF_DIS_CUR;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
}

void parsePSU1(uint8 nArgs, char* args[])
{
  //Power Supply
  // p r[direction: 0 = boost, 1 = buck] v[voltage setting, mV] i[current limit, mA]
  //     default = 1                      default = 4200        default = 1500
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  discharge_voltage_1 = DEF_CHG_VOL;
  discharge_current_1 = DEF_DIS_CUR;
  psu_dir_1 = DEF_PSU_MODE;
  ccc_1 = DEF_CCC;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<5)
            strVal = 5;
          discharge_current_1 = strVal;
          charge_current_1 = -1*strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<100)
            strVal = 100;
          else if(strVal>9000)
            strVal = 9000;
          charge_voltage_1 = strVal;
          discharge_voltage_1 = strVal;
          Serial.print(charge_voltage_1);
          Serial.println("mV");
          break;
        case 'r':
          Serial.print("> Using direction: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Boost");
          else
            Serial.println("Buck");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_1 = strVal;
          Serial.print(ccc_1);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using default mode: ");
    Serial.println("Buck");
    if(charge_voltage_1 == DEF_CHG_VOL)
    {
      Serial.print("> Using default voltage: ");
      Serial.print(charge_voltage_1);
      Serial.println("mV");
    }
  }
  else
  {
    if(discharge_voltage_1 == DEF_DIS_VOL)
    {
      Serial.print("> Using default voltage: ");
      Serial.print(discharge_voltage_1);
      Serial.println("mV");
    }
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using default cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
}

void parseTM1(uint8 nArgs, char* args[])
{
  //Test Mode
  // t r[direction: 0 = boost, 1 = buck] l[duty cycle (0-199)]
  //     default = 1                      default = 0 (boost), 199 (buck)
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  psu_dir_1 = DEF_PSU_MODE;
  tm_duty_1 = 399;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'l':
          Serial.print("> Using duty cycle: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>399)
            strVal = 399;
          else if(strVal<0)
            strVal = 0;
          tm_duty_1 = strVal;
          Serial.println(tm_duty_1);
          break;
        case 'r':
          Serial.print("> Using direction: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Boost");
          else
            Serial.println("Buck");
          break;
        default:
          break;
      }
    }
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using default mode: ");
    if(discharge_mode_1 == 0)
    {
      Serial.println("Boost");
      if(discharge_voltage_1 == DEF_DIS_VOL)
      {
        Serial.print("> Using default voltage: ");
        Serial.print(discharge_voltage_1);
        Serial.println("mV");
      }
    }
    else
    {
      Serial.println("Buck");
      if(charge_voltage_1 == DEF_CHG_VOL)
      {
        Serial.print("> Using default voltage: ");
        Serial.print(charge_voltage_1);
        Serial.println("mV");
      }
    }
  }
}

void parseCycle1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  discharge_voltage_1 = DEF_DIS_VOL;
  discharge_current_1 = DEF_DIS_CUR;
  discharge_mode_1 = DEF_DIS_MODE;
  //Set default charge current/voltage/cutoff
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  ccc_1 = DEF_CCC;
  num_cycles_1 = DEF_CYCLES;
  
  //Cycle
  // y i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
  //     default = 1500          default = 2700       default = 0
  //   k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]
  //     default = 1500       default = 4200       default = 50         default = 1

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_DIS_VOL)
            strVal = MIN_DIS_VOL;
          else if(strVal>MAX_DIS_VOL)
            strVal = MAX_DIS_VOL;
          discharge_voltage_1 = strVal;
          Serial.print(discharge_voltage_1);
          Serial.println("mV");
          break;
        case 'm':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          discharge_mode_1 = strVal;
          if(discharge_mode_1 == 0)
            Serial.println("Constant current");
          else
            Serial.println("Stepped");
          break;
        case 'k':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-100)
            strVal = -100;
          charge_current_1 = strVal;
          Serial.print(charge_current_1);
          Serial.println("mA");
          break;
        case 'u':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<2400)
            strVal = 2400;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_1 = strVal;
          Serial.print(charge_voltage_1);
          Serial.println("mV");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_1 = strVal;
          Serial.print(ccc_1);
          Serial.println("mA");
          break;
        case 'l':
          Serial.print("> Using cycles: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<1)
            strVal = 1;
          else if(strVal>10000)
            strVal = 10000;
          num_cycles_1 = strVal;
          Serial.println(num_cycles_1);
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_1 == DEF_DIS_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(discharge_voltage_1);
    Serial.println("mV");
  }
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(discharge_mode_1 == DEF_DIS_MODE)
  {
    Serial.print("> Using default mode: ");
    if(discharge_mode_1 == 0)
      Serial.println("Constant current");
    else
      Serial.println("Stepped");
  }
  if(charge_voltage_1 == DEF_CHG_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(charge_voltage_1);
    Serial.println("mV");
  }
  if(charge_current_1 == DEF_CHG_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(charge_current_1);
    Serial.println("mA");
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using default cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
  if(num_cycles_1 == DEF_CYCLES)
  {
    Serial.print("> Using default cycles: ");
    Serial.println(num_cycles_1);
  }
}

void parseCharge2(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default charge current/voltage/cutoff
  charge_voltage_2 = DEF_CHG_VOL;
  charge_current_2 = DEF_CHG_CUR;
  ccc_2 = DEF_CCC;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-100)
            strVal = -100;
          charge_current_2 = strVal;
          Serial.print(charge_current_2);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<2400)
            strVal = 2400;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_2 = strVal;
          Serial.print(charge_voltage_2);
          Serial.println("mV");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_2 = strVal;
          Serial.print(ccc_2);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(charge_voltage_2 == DEF_CHG_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(charge_voltage_2);
    Serial.println("mV");
  }
  if(charge_current_2 == DEF_CHG_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(charge_current_2);
    Serial.println("mA");
  }
  if(ccc_2 == DEF_CCC)
  {
    Serial.print("> Using default cutoff: ");
    Serial.print(ccc_2);
    Serial.println("mA");
  }
}

void parseDischarge2(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  discharge_voltage_2 = DEF_DIS_VOL;
  discharge_current_2 = DEF_DIS_CUR;
  discharge_mode_2 = DEF_DIS_MODE;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_2 = strVal;
          Serial.print(discharge_current_2);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_DIS_VOL)
            strVal = MIN_DIS_VOL;
          else if(strVal>MAX_DIS_VOL)
            strVal = MAX_DIS_VOL;
          discharge_voltage_2 = strVal;
          Serial.print(discharge_voltage_2);
          Serial.println("mV");
          break;
        case 'm':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          discharge_mode_2 = strVal;
          if(discharge_mode_2 == 0)
            Serial.println("Constant current");
          else
            Serial.println("Stepped");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_2 == DEF_DIS_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(discharge_voltage_2);
    Serial.println("mV");
  }
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(discharge_mode_2 == DEF_DIS_MODE)
  {
    Serial.print("> Using default mode: ");
    if(discharge_mode_2 == 0)
      Serial.println("Constant current");
    else
      Serial.println("Stepped");
  }
}

void parseIR2(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current
  discharge_current_2 = DEF_DIS_CUR;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_2 = strVal;
          Serial.print(discharge_current_2);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
}

void parsePSU2(uint8 nArgs, char* args[])
{
  //Power Supply
  // p r[direction: 0 = boost, 1 = buck] v[voltage setting, mV] i[current limit, mA]
  //     default = 1                      default = 4200        default = 1500
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  charge_voltage_2 = DEF_CHG_VOL;
  charge_current_2 = DEF_CHG_CUR;
  discharge_voltage_2 = DEF_CHG_VOL;
  discharge_current_2 = DEF_DIS_CUR;
  psu_dir_2 = DEF_PSU_MODE;
  ccc_2 = DEF_CCC;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<50)
            strVal = 50;
          discharge_current_2 = strVal;
          charge_current_2 = -1*strVal;
          Serial.print(discharge_current_2);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<100)
            strVal = 100;
          else if(strVal>9000)
            strVal = 9000;
          charge_voltage_2 = strVal;
          Serial.print(charge_voltage_2);
          Serial.println("mV");
          break;
        case 'r':
          Serial.print("> Using direction: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          psu_dir_2 = strVal;
          if(psu_dir_2 == 0)
            Serial.println("Boost");
          else
            Serial.println("Buck");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_2 = strVal;
          Serial.print(ccc_2);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using default mode: ");
    Serial.println("Buck");
    if(charge_voltage_2 == DEF_CHG_VOL)
    {
      Serial.print("> Using default voltage: ");
      Serial.print(charge_voltage_2);
      Serial.println("mV");
    }
  }
  else
  {
    if(discharge_voltage_2 == DEF_DIS_VOL)
    {
      Serial.print("> Using default voltage: ");
      Serial.print(discharge_voltage_2);
      Serial.println("mV");
    }
  }
  if(ccc_2 == DEF_CCC)
  {
    Serial.print("> Using default cutoff: ");
    Serial.print(ccc_2);
    Serial.println("mA");
  }
}

void parseTM2(uint8 nArgs, char* args[])
{
  //Test Mode
  // t r[direction: 0 = boost, 1 = buck] l[duty cycle (0-199)]
  //     default = 1                      default = 0 (boost), 199 (buck)
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  psu_dir_2 = DEF_PSU_MODE;
  tm_duty_2 = 399;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'l':
          Serial.print("> Using duty cycle: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>399)
            strVal = 399;
          else if(strVal<0)
            strVal = 0;
          tm_duty_2 = strVal;
          Serial.println(tm_duty_2);
          break;
        case 'r':
          Serial.print("> Using direction: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          psu_dir_2 = strVal;
          if(psu_dir_2 == 0)
            Serial.println("Boost");
          else
            Serial.println("Buck");
          break;
        default:
          break;
      }
    }
  }
  
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using default mode: ");
    if(discharge_mode_1 == 0)
    {
      Serial.println("Boost");
      if(discharge_voltage_1 == DEF_DIS_VOL)
      {
        Serial.print("> Using default voltage: ");
        Serial.print(discharge_voltage_1);
        Serial.println("mV");
      }
    }
    else
    {
      Serial.println("Buck");
      if(charge_voltage_1 == DEF_CHG_VOL)
      {
        Serial.print("> Using default voltage: ");
        Serial.print(charge_voltage_1);
        Serial.println("mV");
      }
    }
  }
}

void parseCycle2(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  discharge_voltage_2 = DEF_DIS_VOL;
  discharge_current_2 = DEF_DIS_CUR;
  discharge_mode_2 = DEF_DIS_MODE;
  //Set default charge current/voltage/cutoff
  charge_voltage_2 = DEF_CHG_VOL;
  charge_current_2 = DEF_CHG_CUR;
  ccc_2 = DEF_CCC;
  num_cycles_2 = DEF_CYCLES;
  
  //Cycle
  // y i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
  //     default = 1500          default = 2700       default = 0
  //   k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]
  //     default = 1500       default = 4200       default = 50         default = 1

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_2 = strVal;
          Serial.print(discharge_current_2);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_DIS_VOL)
            strVal = MIN_DIS_VOL;
          else if(strVal>MAX_DIS_VOL)
            strVal = MAX_DIS_VOL;
          discharge_voltage_2 = strVal;
          Serial.print(discharge_voltage_2);
          Serial.println("mV");
          break;
        case 'm':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          discharge_mode_2 = strVal;
          if(discharge_mode_2 == 0)
            Serial.println("Constant current");
          else
            Serial.println("Stepped");
          break;
        case 'k':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-100)
            strVal = -100;
          charge_current_2 = strVal;
          Serial.print(charge_current_2);
          Serial.println("mA");
          break;
        case 'u':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<2400)
            strVal = 2400;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_2 = strVal;
          Serial.print(charge_voltage_2);
          Serial.println("mV");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_2 = strVal;
          Serial.print(ccc_2);
          Serial.println("mA");
          break;
        case 'l':
          Serial.print("> Using cycles: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<1)
            strVal = 1;
          else if(strVal>10000)
            strVal = 10000;
          num_cycles_2 = strVal;
          Serial.println(num_cycles_2);
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_2 == DEF_DIS_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(discharge_voltage_2);
    Serial.println("mV");
  }
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(discharge_mode_2 == DEF_DIS_MODE)
  {
    Serial.print("> Using default mode: ");
    if(discharge_mode_2 == 0)
      Serial.println("Constant current");
    else
      Serial.println("Stepped");
  }
  if(charge_voltage_2 == DEF_CHG_VOL)
  {
    Serial.print("> Using default voltage: ");
    Serial.print(charge_voltage_2);
    Serial.println("mV");
  }
  if(charge_current_2 == DEF_CHG_CUR)
  {
    Serial.print("> Using default current: ");
    Serial.print(charge_current_2);
    Serial.println("mA");
  }
  if(ccc_2 == DEF_CCC)
  {
    Serial.print("> Using default cutoff: ");
    Serial.print(ccc_2);
    Serial.println("mA");
  }
  if(num_cycles_2 == DEF_CYCLES)
  {
    Serial.print("> Using default cycles: ");
    Serial.println(num_cycles_2);
  }
}

void runStateMachine(void)
{
  loop1++;
  loop2++;

  //digitalWrite(PB12, HIGH);
  //2kHz loop:
  //Cell 1 - Ibat, Vbat, CC/CV control loops
  adcval1 = analogRead(AC1A); //Ibat raw
  //Convert ADC value to value in Amps
  tmpfl = ((float)adcval1 - adciref) / ADC2I1; //ADC to I = (ADC-IREF)/(4096b)*(3.3Vmax)/(0.5V/A)
  ibat1 += tmpfl; //Accumulate 2000 values for average (result = mA*2)
  ibat_now1 = (int)(tmpfl * 1000 + 0.5); //Multiply by 1000 for mA, round

  adcval2 = analogRead(AC1V); //adc_read(ADC, 8);
  //Convert ADC value to value in Volts
  tmpfl = ((float)adcval2) / ADC2V1; //ADC to V = (ADC)/(4096b)*(3.3Vmax)*220/150
  vbat1 += tmpfl; //Accumulate 2000 values for average (result = mV*2)
  vbat_now1 = (int)(tmpfl * 1000 + 0.5); //Multiply by 1000 for mV, round

  //Constant current/voltage control loops
  if ((state1 == 3) || (state1 == 4)) //Charging states
  {
    if (ibat_now1 > charge_current_1) //Ibat < 1.5A
    {
      if (vbat_now1 < charge_voltage_1) //Vbat < 4.2V, Ibat < 1.5A
      {
        duty1++;
        if (duty1 > MAXBUCKDUTY)
          duty1 = MAXBUCKDUTY;
      }
      else //Vbat >= 4.2V, Ibat < 1.5A
      {
        duty1--;
        if (duty1 < MINBUCKDUTY)
          duty1 = MINBUCKDUTY;
      }
    }
    else //Ibat >= 1.5A
    {
      duty1--;
      if (duty1 < MINBUCKDUTY)
        duty1 = MINBUCKDUTY;
    }
    pwmWrite(OC1PF, duty1);
  }
  if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
  {
    if (ibat_now1 < discharge_current_1) //Ibat < 1.5A
    {
      if (vbat_now1 > discharge_voltage_1) //Vbat > 2.7V, Ibat < 1.5A
      {
        duty1++;
        if (duty1 > 399)
          duty1 = 399;
      }
      else //Vbat <= 2.7V, Ibat < 1.5A
      {
        duty1--;
        if (duty1 < 0)
          duty1 = 0;
      }
    }
    else //Ibat >= 1.5A
    {
      duty1--;
      if (duty1 < 0)
        duty1 = 0;
    }
    pwmWrite(OC1NF2, duty1); //CC Load
  }
  //Cell 2 - Ibat, Vbat, CC/CV control loops
  adcval4 = analogRead(AC2A); //Ibat raw
  //Convert ADC value to value in Amps
  tmpfl = ((float)adcval4 - adciref) / ADC2I2; //ADC to I = (ADC-IREF)/(4096b)*(3.3Vmax)/(0.5V/A)
  ibat2 += tmpfl; //Accumulate 2000 values for average (result = mA*2)
  ibat_now2 = (int)(tmpfl * 1000 + 0.5); //Multiply by 1000 for mA, round

  adcval5 = analogRead(AC2V); //adc_read(ADC, 8);
  //Convert ADC value to value in Volts
  tmpfl = ((float)adcval5) / ADC2V2; //ADC to V = (ADC)/(4096b)*(3.3Vmax)*220/150
  vbat2 += tmpfl; //Accumulate 2000 values for average (result = mV*2)
  vbat_now2 = (int)(tmpfl * 1000 + 0.5); //Multiply by 1000 for mV, round

  //Constant current/voltage control loops
  if ((state2 == 3) || (state2 == 4)) //Charging states
  {
    if (ibat_now2 > charge_current_2) //Ibat < 1.5A
    {
      if (vbat_now2 < charge_voltage_2) //Vbat < 4.2V, Ibat < 1.5A
      {
        duty2++;
        if (duty2 > MAXBUCKDUTY)
          duty2 = MAXBUCKDUTY;
      }
      else //Vbat >= 4.2V, Ibat < 1.5A
      {
        duty2--;
        if (duty2 < MINBUCKDUTY)
          duty2 = MINBUCKDUTY;
      }
    }
    else //Ibat >= 1.5A
    {
      duty2--;
      if (duty2 < MINBUCKDUTY)
        duty2 = MINBUCKDUTY;
    }
    pwmWrite(OC2PF, duty2);
  }
  if ((state2 == 1) || (state2 == 6) || (state2 == 7)) //Discharging states
  {
    if (ibat_now2 < discharge_current_2) //Ibat < 1.5A
    {
      if (vbat_now2 > discharge_voltage_2) //Vbat > 2.7V, Ibat < 1.5A
      {
        duty2++;
        if (duty2 > MAXBUCKDUTY)
          duty2 = MAXBUCKDUTY;
      }
      else //Vbat <= 2.7V, Ibat < 1.5A
      {
        duty2--;
        if (duty2 < 0)
          duty2 = 0;
      }
    }
    else //Ibat >= 1.5A
    {
      duty2--;
      if (duty2 < 0)
        duty2 = 0;
    }
    pwmWrite(OC2NF2, duty2);
  }
  //digitalWrite(PB12, LOW);

  //500Hz loop - Cell 1 IR
  if ((loop1 % 4) == 0)
  {
    //digitalWrite(PB12, HIGH);
    if (irstate1 == 2)
    {
      //Iload in mA/8 (mA*2 * (500/2000) * 0.25)
      iload1 = iload1 + ((float)adcval1 - adciref) / ADC2I1; //Accumulate current samples from 250-500ms
      //Vload in mV/8 (mV*2 * (500/2000) * 0.25)
      vload1 = vload1 + ((float)adcval2) / ADC2V1; //Accumulate voltage samples from 250-500ms
    }
    else if (irstate1 == 3)
    {
      //Voc2 in mV/8 (mV*2 * (500/2000) * 0.25)
      voc21 = voc21 + ((float)adcval2) / ADC2V1;
    }

    if ((irstate1 == 1) && (loop1 == 500)) //Get Voc1 from first 250ms of sampling, then turn on load at 250ms
    {
      voc11 = vbat1; //Vbat in mV/2 (mV*2 * (500/2000))
      iload1 = 0;
      vload1 = 0;
      setChg1(DISCHARGE);
      irstate1 = 2;
    }
    if ((irstate1 == 2) && (loop1 == 1000)) //Get Vload, Iload from 250-500ms of sampling, then turn off load at 500ms
    {
      voc21 = 0;
      setChg1(DISCONNECT);
      irstate1 = 3;
    }
    if ((irstate1 == 3) && (loop1 == 1500)) //Get Voc2 from 500-750ms of sampling
    {
      //     mV/2 + mV/8*4    -  mV/8*4*2    /  mA/8  / 125
      //         mV           -  mV          /  A = mOhms
      ir1 = ((voc11 + voc21 * 4.0) - (vload1 * 8.0)) / (iload1 / 125.0);
      //Msg type 3 (IR): 3,4126.45,mV,4123.15,mOhms,1259.21,mA,4053.12,mV,56.92,mOhms, 7,1
      //(IR Complete,3,Voc1,mV,Voc2,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
      Serial.print("3,");
      Serial.print(voc11 * 2.0);
      Serial.print(",");
      Serial.print(voc21 * 8.0);
      Serial.print(",");
      Serial.print(iload1 * 8.0);
      Serial.print(",");
      Serial.print(vload1 * 8.0);
      Serial.print(",");
      Serial.print(ir1);
      Serial.print(",");
      Serial.println(state1);
      //Serial.print(",");
      //Serial.println(settle1);
      irstate1 = 0;
    }

    //1Hz loop: Periodic status printing, temp measurement
    if (loop1 == 2000)
    {
      //Serial.print("Int cnt: ");
      //Serial.println(interruptcnt);
      //interruptcnt++;
      //digitalWrite(PB12,HIGH);
      loop1 = 1;
      
      //mA*2 /2 /3600 = mAH
      mah1 = mah1 + ibat1 / 7200.0;
      mwh1 = mwh1 + vbat1 / 2000.0 * ibat1 / 7200.0;
      vbat_1_1 = vbat1 / 2.0;
      ibat_1_1 = ibat1 / 2.0;    
      
      adcval3 = analogRead(AC1T);
      //Code for 2.495V sourcing 10k NTC divider
      if (adcval3 > 1269) //0-35C, use linear estimation
      {
        temp1 = ((float)adcval3) / -28.82 + 78.816 + TOFFS1;
      }
      else //35C+, use polynomial estimation
      {
        temp1 = (((float)adcval3) * ((float)adcval3) * ((float)adcval3) / -20005929.2 + ((float)adcval3) * ((float)adcval3) / 6379.75 + ((float)adcval3) / -4.866 + 144.9107) + TOFFS1;
      }
      //Code for 3.3V sourcing 10k NTC divider
      /*if (adcval3 > 1900) //0-28C, use linear estimation
      {
        temp1 = ((float)adcval3) / -38.61 + 78.055 + TOFFS1;
      }
      else //35C+, use polynomial estimation
      {
        temp1 = (((float)adcval3)*((float)adcval3)*((float)adcval3)/-181172413.8+((float)adcval3)*((float)adcval3)/31672.3+((float)adcval3)/-11.3881+119.565) + TOFFS1;
      }*/
      if(((int)temp1 > OVT_THRESH) && (state1 != 8))
      {
        setChg1(DISCONNECT);
        setLED1(RED);
        state1 = 8;
        settle1 = 0;
        mode1 = 5;
        //Timer1.pause();
        Serial.println("> Cell 1 OVT, stopping");
        Serial.print("> Cell 1 Temp: ");
        Serial.println(temp1);
      }
      
      //78.125us/char at 115200
      if (state1 != 8)
      {
        //Msg type 0 (Periodic Status): 0,3754.12,mV,-1453.98,mA,-750.19,mAH,-2810.34,mWH,23.5,C,4,358
        //(Periodic Status,0,Vbat,mV,Ibat,mA,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
        Serial.print("0,");
        Serial.print(vbat_1_1);
        Serial.print(",");
        Serial.print(ibat_1_1);
        Serial.print(",");
        Serial.print(mah1);
        Serial.print(",");
        Serial.print(mwh1);
        Serial.print(",");
        Serial.print(temp1);
        Serial.print(",");
        Serial.println(state1);
        //Serial.print(",");
        //Serial.println(settle1);
      }
      /* State machine:
          0. Battery Disconnected (Unused)
          1. Battery Discharge
          Voltage < 2.75V?
          2. Battery Disconnected
          Wait 5 minutes
          3. Battery Charge
          Current < 50mA?
          4. Battery Disconnect
          5. Wait 1 minute
          6. Measure IR
          7. Measure IR
          Wait 10 seconds
          8. Parking
          1. Battery Discharge (goto top)
      */
      /* Mode List:
          0 = c = Charge
          1 = d = Discharge
          2 = y = Cycle
          3 = p = Power Supply
          4 = t = Test
      */
      switch (state1)
      {
        case 0: //Default state; determine to charge or discharge based on current SoC
          //Serial.println("Case 0");
          setChg1(DISCONNECT);
          vbati1 = (int)(vbat_1_1);
          if (vbati1 >= 3700) //Above mid-charge state
          {
            state1 = 1;
            setChg1(DISCHARGE);
          }
          else
            state1 = 3;
          break;
        case 1: //Discharge state; check for LVC, goto disconnect state if triggered
          //Serial.println("Case 1");
          vbati1 = (int)(vbat_1_1);
          if ((vbati1 <= discharge_voltage_1) && (mode1 != 3)) //Discharged at 2.75V
          {
            state1 = 2;
            settle1 = 0;
            setLED1(PURPLE);
          }
          else if ((ibat_1_1 <= ccc_1) && (mode1 == 3) && (settle1 > 10)) //Full battery = <50mA CC
          {
            state1 = 2;
            settle1 = 0;
            setLED1(PURPLE);
          }
          break;
        case 2: //Disconnect/settling state after discharge, wait 5 minutes before charging
          //Serial.println("Case 2");
          setChg1(DISCONNECT);
          if (settle1 > AFTERDISWAIT)
          {
            if(mode1 == 1)
            {
              state1 = 8;
              settle1 = 0;
              mode1 = 5;
              setLED1(OFF);
            }
            else
            {
              state1 = 3;
              settle1 = 0;
              setChg1(CHARGE);
              setLED1(CYAN);
            }
            //Msg type 1 (Discharged): 0,2754.12,mV,32.57,mOhms,893.21,mAH,3295.12,mWH,25.1,C,3,1
            //(Periodic Status,0,Vbat,mV,IR,mOhms,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
            Serial.print("1,");
            Serial.print(vbat_1_1);
            Serial.print(",");
            Serial.print(ir1);
            Serial.print(",");
            Serial.print(mah1);
            Serial.print(",");
            Serial.print(mwh1);
            Serial.print(",");
            Serial.print(temp1);
            Serial.print(",");
            Serial.println(state1);
            //Serial.print(",");
            //Serial.println(settle1);
            mah1 = 0;
            mwh1 = 0;
          }
          break;
        case 3: //Charging state (CC)
          //Serial.println("Case 3");
          if (settle1 > 30) //Wait 30s for current to settle
          {
            state1 = 4;
            settle1 = 0;
          }
          break;
        case 4: //Charging state (CC/CV)
          //Serial.println("Case 4");
          ibati1 = (int)(ibat_1_1 * -1.0);
          if ((ibati1 <= ccc_1) && (mode1 != 3)) //Full battery = <50mA CC
          {
            setChg1(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
            state1 = 5;
            settle1 = 0;
            setLED1(PURPLE);
          }
          break;
        case 5:
          //Serial.println("Case 5");
          if (settle1 > AFTERCHGWAIT) //1 Minutes to settle after charging
          {
            state1 = 6;
            settle1 = 0;
          }
          break;
        case 6: //IR measure state
          //Serial.println("Case 6");
          irstate1 = 1;
          state1 = 7;
          break;
        case 7: //IR measure state
          //Serial.println("Case 7");
          if (settle1 > 2)
          {
            if(mode1 == 0 || mode1 == 6) //Finished charge cycle, reset+park state
            {
              setChg1(DISCONNECT);
              state1 = 8;
              settle1 = 0;
              mode1 = 5;
              setLED1(OFF);
            }
            else
            {
              cycle_count_1++;
              if(cycle_count_1 == (num_cycles_1 + 1))//1 more cycle for initial charge
              {
                setChg1(DISCONNECT);
                state1 = 8;
                settle1 = 0;
                mode1 = 5;
                setLED1(OFF);
              }
              else
              {
                state1 = 1;
                settle1 = 0;
                setLED1(YELLOW);
                setChg1(DISCHARGE);
              }
            }
            //Msg type 2 (Charged): 0,4126.45,mV,32.57,mOhms,-750.19,mAH,-2810.34,mWH,23.5,C,6,1
            //(Charged,0,Vbat,mV,IR,mOhms,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
            Serial.print("2,");
            Serial.print(vbat_1_1);
            Serial.print(",");
            Serial.print(ir1);
            Serial.print(",");
            Serial.print(mah1);
            Serial.print(",");
            Serial.print(mwh1);
            Serial.print(",");
            Serial.print(temp1);
            Serial.print(",");
            Serial.println(state1);
            //Serial.print(",");
            //Serial.println(settle1);
            mah1 = 0;
            mwh1 = 0;
          }
          break;
        case 8: //Parking state
          setChg1(DISCONNECT);
          break;
        case 9: //Test state
          //setChg1(DISCONNECT);
          break;
        default:
          setChg1(DISCONNECT);
          state1 = 8;
          settle1 = 0;
          break;
      }
      //display.display();
      /*Serial.print("State: ");
        Serial.println(state);
        Serial.print("Settle: ");
        Serial.println(settle);*/
      settle1++;
      if (settle1 > 32000)
        settle1 = 32000;
      ibat1 = 0;
      vbat1 = 0;
      loopcnt = 0;
    }
    //digitalWrite(PB12, LOW);
  }
  //500Hz loop - Cell 2 IR
  if ((loop2 % 4) == 3)
  {
    //digitalWrite(PB12, HIGH);
    if (irstate2 == 2)
    {
      //Iload in mA/8 (mA*2 * (500/2000) * 0.25)
      iload2 = iload2 + ((float)adcval4 - adciref) / ADC2I2; //Accumulate current samples from 250-500ms
      //Vload in mV/8 (mV*2 * (500/2000) * 0.25)
      vload2 = vload2 + ((float)adcval5) / ADC2V2; //Accumulate voltage samples from 250-500ms
    }
    else if (irstate2 == 3)
    {
      //Voc2 in mV/8 (mV*2 * (500/2000) * 0.25)
      voc22 = voc22 + ((float)adcval5) / ADC2V2;
    }

    if ((irstate2 == 1) && (loop2 == 503)) //Get Voc1 from first 250ms of sampling, then turn on load at 250ms
    {
      voc12 = vbat2; //Vbat in mV/2 (mV*2 * (500/2000))
      iload2 = 0;
      vload2 = 0;
      setChg2(DISCHARGE);
      irstate2 = 2;
    }
    if ((irstate2 == 2) && (loop2 == 1003)) //Get Vload, Iload from 250-500ms of sampling, then turn off load at 500ms
    {
      voc22 = 0;
      setChg2(DISCONNECT);
      irstate2 = 3;
    }
    if ((irstate2 == 3) && (loop2 == 1503)) //Get Voc2 from 500-750ms of sampling
    {
      //     mV/2 + mV/8*4    -  mV/8*4*2    /  mA/8  / 125
      //         mV           -  mV          /  A = mOhms
      ir2 = ((voc12 + voc22 * 4.0) - (vload2 * 8.0)) / (iload2 / 125.0);
      //Msg type 3 (IR): 3,4126.45,mV,4123.15,mOhms,1259.21,mA,4053.12,mV,56.92,mOhms, 7,1
      //(IR Complete,3,Voc1,mV,Voc2,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
      Serial.print("8,");
      Serial.print(voc12 * 2.0);
      Serial.print(",");
      Serial.print(voc22 * 8.0);
      Serial.print(",");
      Serial.print(iload2 * 8.0);
      Serial.print(",");
      Serial.print(vload2 * 8.0);
      Serial.print(",");
      Serial.print(ir2);
      Serial.print(",");
      Serial.println(state2);
      //Serial.print(",");
      //Serial.println(settle2);
      irstate2 = 0;
    }

    //1Hz loop: Periodic status printing, temp measurement
    if (loop2 == 2003)
    {
      //Serial.print("Int cnt: ");
      //Serial.println(interruptcnt);
      //interruptcnt++;
      //digitalWrite(PB12,HIGH);
      loop2 = 4;

      //mA*2 /2 /3600 = mAH
      mah2 = mah2 + ibat2 / 7200.0;
      mwh2 = mwh2 + vbat2 / 2000.0 * ibat2 / 7200.0;
      vbat_1_2 = vbat2 / 2.0;
      ibat_1_2 = ibat2 / 2.0;
      adciref = analogRead(AIREF); //Iref voltage

      adcval6 = analogRead(AC2T);
      if (adcval6 > 1269) //0-35C, use linear estimation
      {
        temp2 = ((float)adcval6) / -28.82 + 78.816 + TOFFS2;
      }
      else //35C+, use polynomial estimation
      {
        temp2 = (((float)adcval6) * ((float)adcval6) * ((float)adcval6) / -20005929.2 + ((float)adcval6) * ((float)adcval6) / 6379.75 + ((float)adcval6) / -4.866 + 144.9107) + TOFFS2;
      }
      if(((int)temp2 > OVT_THRESH) && (state2 != 8))
      {
        setChg2(DISCONNECT);
        setLED2(RED);
        state2 = 8;
        settle2 = 0;
        mode2 = 5;
        //Timer1.pause();
        Serial.println("> Cell 2 OVT, stopping");
        Serial.print("> Cell 2 Temp: ");
        Serial.println(temp2);
      }
      
      //78.125us/char at 115200
      if (state2 != 8)
      {
        //Msg type 0 (Periodic Status): 0,3754.12,mV,-1453.98,mA,-750.19,mAH,-2810.34,mWH,23.5,C,4,358
        //(Periodic Status,0,Vbat,mV,Ibat,mA,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
        Serial.print("5,");
        Serial.print(vbat_1_2);
        Serial.print(",");
        Serial.print(ibat_1_2);
        Serial.print(",");
        Serial.print(mah2);
        Serial.print(",");
        Serial.print(mwh2);
        Serial.print(",");
        Serial.print(temp2);
        Serial.print(",");
        Serial.println(state2);
        //Serial.print(",");
        //Serial.println(settle2);
      }
      /* State machine:
          0. Battery Disconnected (Unused)
          1. Battery Discharge
          Voltage < 2.75V?
          2. Battery Disconnected
          Wait 5 minutes
          3. Battery Charge
          Current < 50mA?
          4. Battery Disconnect
          5. Wait 1 minute
          6. Measure IR
          7. Measure IR
          Wait 10 seconds
          8. Parking
          1. Battery Discharge (goto top)
      */
      /* Mode List:
          0 = c = Charge
          1 = d = Discharge
          2 = y = Cycle
          3 = p = Power Supply
          4 = t = Test
      */
      switch (state2)
      {
        case 0: //Default state; determine to charge or discharge based on current SoC
          //Serial.println("Case 0");
          setChg2(DISCONNECT);
          vbati2 = (int)(vbat_1_2);
          if (vbati1 >= 3700) //Above mid-charge state
          {
            state2 = 1;
            setChg2(DISCHARGE);
          }
          else
            state2 = 3;
          break;
        case 1: //Discharge state; check for LVC, goto disconnect state if triggered
          //Serial.println("Case 1");
          vbati2 = (int)(vbat_1_2);
          if ((vbati2 <= discharge_voltage_2) && (mode2 != 3)) //Discharged at 2.75V
          {
            state2 = 2;
            settle2 = 0;
            setLED2(PURPLE);
          }
          else if ((ibat_1_2 <= ccc_2) && (mode2 == 3) && (settle2 > 10)) //Full battery = <50mA CC
          {
            state2 = 2;
            settle2 = 0;
            setLED2(PURPLE);
          }
          break;
        case 2: //Disconnect/settling state after discharge, wait 5 minutes before charging
          //Serial.println("Case 2");
          setChg2(DISCONNECT);
          if (settle2 > AFTERDISWAIT)
          {
            if(mode2 == 1)
            {
              state2 = 8;
              settle2 = 0;
              mode2 = 5;
              setLED2(OFF);
            }
            else
            {
              state2 = 3;
              settle2 = 0;
              setChg2(CHARGE);
              setLED2(CYAN);
            }
            //Msg type 1 (Discharged): 0,2754.12,mV,32.57,mOhms,893.21,mAH,3295.12,mWH,25.1,C,3,1
            //(Periodic Status,0,Vbat,mV,IR,mOhms,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
            Serial.print("6,");
            Serial.print(vbat_1_2);
            Serial.print(",");
            Serial.print(ir2);
            Serial.print(",");
            Serial.print(mah2);
            Serial.print(",");
            Serial.print(mwh2);
            Serial.print(",");
            Serial.print(temp2);
            Serial.print(",");
            Serial.println(state2);
            //Serial.print(",");
            //Serial.println(settle2);
            mah2 = 0;
            mwh2 = 0;
          }
          break;
        case 3: //Charging state (CC)
          //Serial.println("Case 3");
          if (settle2 > 30) //Wait 30s for current to settle
          {
            state2 = 4;
            settle2 = 0;
          }
          break;
        case 4: //Charging state (CC/CV)
          //Serial.println("Case 4");
          ibati2 = (int)(ibat_1_2 * -1.0);
          if ((ibati2 <= ccc_1) && (mode2 != 3)) //Full battery = <50mA CC
          {
            setChg2(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
            state2 = 5;
            settle2 = 0;
            setLED2(PURPLE);
          }
          break;
        case 5:
          //Serial.println("Case 5");
          if (settle2 > AFTERCHGWAIT) //1 Minutes to settle after charging
          {
            state2 = 6;
            settle2 = 0;
          }
          break;
        case 6: //IR measure state
          //Serial.println("Case 6");
          irstate2 = 1;
          state2 = 7;
          break;
        case 7: //IR measure state
          //Serial.println("Case 7");
          if (settle2 > 2)
          {
            if(mode2 == 0 || mode2 == 6) //Finished charge cycle, reset+park state
            {
              setChg2(DISCONNECT);
              state2 = 8;
              settle2 = 0;
              mode2 = 5;
              setLED2(OFF);
            }
            else
            {
              cycle_count_2++;
              if(cycle_count_2 == (num_cycles_2 + 1))//1 more cycle for initial charge
              {
                setChg2(DISCONNECT);
                state2 = 8;
                settle2 = 0;
                mode2 = 5;
                setLED2(OFF);
              }
              else
              {
                state2 = 1;
                settle2 = 0;
                setLED2(YELLOW);
                setChg2(DISCHARGE);
              }
            }
            //Msg type 2 (Charged): 0,4126.45,mV,32.57,mOhms,-750.19,mAH,-2810.34,mWH,23.5,C,6,1
            //(Charged,0,Vbat,mV,IR,mOhms,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
            Serial.print("7,");
            Serial.print(vbat_1_2);
            Serial.print(",");
            Serial.print(ir2);
            Serial.print(",");
            Serial.print(mah2);
            Serial.print(",");
            Serial.print(mwh2);
            Serial.print(",");
            Serial.print(temp2);
            Serial.print(",");
            Serial.println(state2);
            //Serial.print(",");
            //Serial.println(settle2);
            mah2 = 0;
            mwh2 = 0;
          }
          break;
        case 8: //Parking state
          setChg2(DISCONNECT);
          break;
        case 9: //Test state
          //setChg2(DISCONNECT);
          break;
        default:
          setChg2(DISCONNECT);
          state2 = 8;
          settle2 = 0;
          break;
      }
      if((state1 != 8) || (state2 != 8))
      {
        adciref = analogRead(AIREF); //Iref voltage
        vbuf_i = (int)(((float)getAuxADC(BUFV)) * BUF2V);
        //Msg type 4 (Debug):
        //(Debug,4,Vbuf,mV,Vrev,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
        Serial.print("4,");
        Serial.print(vbuf_i); //2.417);
        adctemp = getAuxADC(HSTH1);
        if (adctemp > 1269) //0-35C, use linear estimation
        {
          temp_t = ((float)adctemp) / -28.82 + 78.816 + TOFFS2;
        }
        else //35C+, use polynomial estimation
        {
          temp_t = (((float)adctemp) * ((float)adctemp) * ((float)adctemp) / -20005929.2 + ((float)adctemp) * ((float)adctemp) / 6379.75 + ((float)adctemp) / -4.866 + 144.9107) + TOFFS2;
        }
        Serial.print(",");
        Serial.println(temp_t);//2.417);
        //Serial.print(",");
        //Serial.print(ibat_now);//2.417);
        //Serial.print(",mA,");
        //Serial.print(vbat_now);//2.417);
        //Serial.print(",mV,");
        //Serial.print(duty1);//2.417);
        //Serial.print(",");
        //Serial.println(duty2);//2.417);
        //Serial.println(",Duty2");
      }
      if((state1 == 1) || (state2 == 1) || (state1 == 2) || (state2 == 2) || (state1 == 3) || (state2 == 3) || (state1 == 4) || (state2 == 4))
      {
        pinMode(FANON, OUTPUT);
        digitalWrite(FANON, HIGH); //Non-inverting mode
      }
      else
      {
        pinMode(FANON, OUTPUT);
        digitalWrite(FANON, LOW); //Non-inverting mode
      }
      //display.display();
      /*Serial.print("State: ");
        Serial.println(state);
        Serial.print("Settle: ");
        Serial.println(settle);*/
      settle2++;
      if (settle2 > 32000)
        settle2 = 32000;
      ibat2 = 0;
      vbat2 = 0;
    }
    //digitalWrite(PB12, LOW);
  }
}

void loop() {

  /* Mode List:
      0 = c = Charge
      1 = d = Discharge
      2 = y = Cycle
      3 = p = Power Supply
      4 = t = Test
          v = Version
          ? = Help
          s = Status
  */
  uint8 i = 0;
  uint16 estatus = 0, wAddress = 0, wData = 0;
  char *args[8];
  recvWithStartEndMarkers();
  if (newData == true)
  {
    //charge_current = -1500;
    //charge_voltage = 4200;
    //discharge_current = 1500;
    //discharge_voltage = 2700;
    //ccc = 50; //Charge CC cutoff in mA (50mA)
    //Charge
    // c[1-2] i[charge current, mA] v[charge voltage, mV] o[cutoff current, mA]
    //     default = 1500       default = 4200       default = 50
    //Discharge
    // d[1-2] i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
    //     default = 1500          default = 2700       default = 0
    //Cycle
    // y[1-2] i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
    //     default = 1500          default = 2700       default = 0
    //   k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]
    //     default = 1500       default = 4200       default = 50         default = 1
    //Power Supply
    // p[1-2] r[direction: 0 = boost, 1 = buck] v[voltage setting, mV] i[current limit, mA]
    //     default = 1                      default = 4200        default = 1500
    //Test Mode
    // t[1-2] r[direction: 0 = boost, 1 = buck] l[duty cycle (0-199)]
    //     default = 1                      default = 0 (boost), 199 (buck)
    //IR Test
    // r[1-2] i[test current, mA]
    //          default = 1500
    //Help (prints this menu)
    // ?
    args[i] = strtok(receivedChars, " ");
    while (args[i] != NULL) //i = number of arguments in received string
    {
      args[++i] = strtok(NULL, " ");
    }
    switch (args[0][0]) {
      case 'c':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(RED);
            Serial.println("> Detected reverse polarity cell, cancelling charge");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            mode1 = 0;
            parseCharge1(i, args);
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(CYAN);
            printMenu(mode1);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(RED);
            Serial.println("> Detected reverse polarity cell, cancelling charge");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            mode2 = 0;
            parseCharge2(i, args);
            state2 = 3;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(CHARGE);
            setLED2(CYAN);
            printMenu(mode2);
          }
        }
        break;
      case 'd':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(RED);
            Serial.println("> Detected reverse polarity cell, cancelling discharge");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            mode1 = 1;
            parseDischarge1(i, args);
            state1 = 1;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(DISCHARGE);
            setLED1(YELLOW);
            printMenu(mode1);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(RED);
            Serial.println("> Detected reverse polarity cell, cancelling discharge");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            mode2 = 1;
            parseDischarge2(i, args);
            state2 = 1;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(DISCHARGE);
            setLED2(YELLOW);
            printMenu(mode2);
          }
        }
        break;
      case 'r':
        if(args[0][1] == '1')
        {
          mode1 = 6;
          parseIR1(i, args);
          state1 = 6;
          settle1 = 0;
          ir1 = 0;
          //Timer2.resume(); //Start the timer counting
          setLED1(PURPLE);
          printMenu(mode1);
        }
        else if(args[0][1] == '2')
        {
          mode2 = 6;
          parseIR2(i, args);
          state2 = 6;
          settle2 = 0;
          ir2 = 0;
          //Timer2.resume(); //Start the timer counting
          setLED2(PURPLE);
          printMenu(mode2);
        }
        break;
      case 'y':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(RED);
            Serial.println("> Detected reverse polarity cell, cancelling cycle");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            mode1 = 2;
            parseCycle1(i, args);
            cycle_count_1 = 0;
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(CYAN);
            printMenu(mode1);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(RED);
            Serial.println("> Detected reverse polarity cell, cancelling charge");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            mode2 = 2;
            parseCycle2(i, args);
            cycle_count_2 = 0;
            state2 = 3;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(CHARGE);
            setLED2(CYAN);
            printMenu(mode2);
          }
        }
        break;
      case 'p':
        if(args[0][1] == '1')
        {
          mode1 = 3;
          parsePSU1(i, args);
          if(psu_dir_1 == 1) //Buck aka charge mode
          {
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(CYAN);
          }
          else
          {
            state1 = 1;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(DISCHARGE);
            setLED1(YELLOW);
          }
          printMenu(mode1);
        }
        else if(args[0][1] == '2')
        {
          mode2 = 3;
          parsePSU2(i, args);
          if(psu_dir_2 == 1) //Buck aka charge mode
          {
            state2 = 3;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(CHARGE);
            setLED2(CYAN);
          }
          else
          {
            state2 = 1;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(DISCHARGE);
            setLED2(YELLOW);
          }
          printMenu(mode2);
        }
        break;
      case 't':
        if(args[0][1] == 'w')
        {
          mode1 = 6;
          Serial.println("\r\n> Writing EEPROM: ");
          //aXXX = address in dec.
          wAddress = fast_atoi_leading_pos(args[1]);
          //dXXX = data in dec.
          wData = fast_atoi_leading_pos(args[2]);
          Serial.print("> Address 0x");
          Serial.print(wAddress, HEX);
          Serial.print(", Data = dec. ");
          Serial.println(wData);
          estatus = EEPROM.write(wAddress, wData);
          if(estatus == 0)
          {
            Serial.print("> Write finished, Status: ");
            Serial.println(estatus, HEX);
          }
          else
          {
            Serial.print("> EEPROM error, code: ");
            Serial.println(estatus, HEX);
          }
        }
        else if(args[0][1] == 'r')
        {
          mode1 = 6;
          Serial.println("\r\n> Reading EEPROM: ");
          wAddress = fast_atoi_leading_pos(args[1]);
          Serial.print("> Address 0x");
          Serial.print(wAddress, HEX);
          estatus = EEPROM.read(wAddress, &wData);
          Serial.print(", Data = dec. ");
          Serial.println(wData);
          if(estatus == 0)
          {
            Serial.print("> Read finished, Status: ");
            Serial.println(estatus, HEX);
          }
          else
          {
            Serial.print("> EEPROM error, code: ");
            Serial.println(estatus, HEX);
          }
        }
        break;
      case 'l': //Calibration mode/update
      //Currently broken
        updateADCRef();
        ADC2V1 = ADC2V1INIT;
        ADC2V2 = ADC2V2INIT;
        ADC2I1 = ADC2I1INIT;
        ADC2I2 = ADC2I2INIT;
        mode1 = 3;
        charge_voltage_1 = DEF_CHG_VOL;
        charge_current_1 = -1000;
        psu_dir_1 = DEF_PSU_MODE;
        state1 = 3;
        settle1 = 0;
        mah1 = 0;
        mwh1 = 0;
        //Timer2.resume(); //Start the timer counting
        setChg1(CHARGE);
        setLED1(CYAN);
        mode2 = 3;
        charge_voltage_2 = DEF_CHG_VOL;
        charge_current_2 = -1000;
        psu_dir_2 = DEF_PSU_MODE;
        state2 = 3;
        settle2 = 0;
        mah2 = 0;
        mwh2 = 0;
        //Timer2.resume(); //Start the timer counting
        setChg2(CHARGE);
        setLED2(CYAN);
        Serial.print("\r\n");
        Serial.println("> Currently in Calibration Mode");
        Serial.println("> Slot 1/2 corrections reset to default");
        Serial.print(">  ADC correction factor: ");
        Serial.print(corr_factor*100);
        Serial.println(" Slot 1+2 set to 4.20V, 1.0A");
        Serial.println(">  Press n1, n2 to end");
        Serial.print("\r\n");
        Serial.print("> ");
        break;
      case 'n':
        if(args[0][1] == '1')
        {
          mode1 = 6;
          state1 = 8;
          setChg1(DISCONNECT);
          //Timer2.pause(); //Start the timer counting
          setLED1(OFF);
          Serial.println("\r\n");
          //printMenu(mode1);
        }
        else if(args[0][1] == '2')
        {
          mode2 = 6;
          state2 = 8;
          setChg2(DISCONNECT);
          //Timer2.pause(); //Start the timer counting
          setLED2(OFF);
          Serial.println("\r\n");
          //printMenu(mode2);
        }
        break;
      case 'v':        
        Serial.print("\r\n> Software version: ");
        Serial.println(vers);
        Serial.println("\r\n>");
        break;
      case 'q':
        if(args[0][1] == '1')
        {
          mode1 = 3;
          state1 = 9;
          settle1 = 0;
          parseTM1(i, args);
          setChg1(CHARGE);
          pwmWrite(OC1PF, tm_duty_1);
          //printMenu(mode1);
        }
        else if(args[0][1] == '2')
        {
          mode2 = 3;
          state2 = 9;
          settle2 = 0;
          parseTM2(i, args);
          setChg2(CHARGE);
          pwmWrite(OC2PF, tm_duty_2);
          //printMenu(mode2);
        }
        break;
      case 's':
        Serial.println("\r\n");
        if((state1 == 8) || (state2 == 8))
        {
          //adciref = analogRead(AIREF); //Iref voltage
          vbuf_i = (int)(((float)getAuxADC(BUFV)) * BUF2V);
        }
        //ToDo - restore state or disable while running
        //settle1 = 0;
        //settle2 = 0;
        //Timer2.resume(); //Start the timer counting
        //delay(1200);
        //Msg type 4 (Debug):
        //(Debug,4,Vbuf,mV,Vrev,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
        Serial.print("4,");
        Serial.print(vbuf_i); //2.417);
        Serial.print(",");
        //Serial.print(ibat_now);//2.417);
        //Serial.print(",mA,");
        //Serial.print(vbat_now);//2.417);
        //Serial.print(",mV,");
        Serial.print(getCell1RV());
        Serial.print(",");
        Serial.print(getCell2RV());
        Serial.print(",");
        Serial.print(adciref);//2.417);
        adctemp = getAuxADC(HSTH1);
        if (adctemp > 939) //0-40C, use linear estimation
        {
          temp_t = ((float)adctemp) / -32.56 + 72.980 + TOFFSHS1;
        }
        else //40C+, use polynomial estimation
        {
          temp_t = (((float)adctemp) * ((float)adctemp) * ((float)adctemp) / -7047334.3 + ((float)adctemp) * ((float)adctemp) / 3173.10 + ((float)adctemp) / -3.6194 + 143.561) + TOFFSHS1;
        }
        Serial.print(",");
        Serial.print(temp_t);//2.417);
        adctemp = getAuxADC(HSTH2);
        if (adctemp > 939) //0-40C, use linear estimation
        {
          temp_t = ((float)adctemp) / -32.56 + 72.980 + TOFFSHS2;
        }
        else //40C+, use polynomial estimation
        {
          temp_t = (((float)adctemp) * ((float)adctemp) * ((float)adctemp) / -7047334.3 + ((float)adctemp) * ((float)adctemp) / 3173.10 + ((float)adctemp) / -3.6194 + 143.561) + TOFFSHS2;
        }
        Serial.print(",");
        Serial.println(temp_t);//2.417);
        Serial.print("9,");
        Serial.print(vbat_1_1);
        Serial.print(",");
        Serial.print(ibat_1_1);
        Serial.print(",");
        Serial.print(mah1);
        Serial.print(",");
        Serial.print(mwh1);
        Serial.print(",");
        Serial.print(temp1);
        Serial.print(",");
        Serial.println(ir1);
        Serial.print("10,");
        Serial.print(vbat_1_2);
        Serial.print(",");
        Serial.print(ibat_1_2);
        Serial.print(",");
        Serial.print(mah2);
        Serial.print(",");
        Serial.print(mwh2);
        Serial.print(",");
        Serial.print(temp2);
        Serial.print(",");
        Serial.println(ir2);
        Serial.print("\r\n> ");
        //state1 = 9; //Set to invalid state to print status
        //state2 = 9; //Set to invalid state to print status
        //Serial.println(",Duty2");
        //Timer2.pause(); //Start the timer counting
        //state1 = 8; //Set to invalid state to print status
        //state2 = 8; //Set to invalid state to print status
        //printMenu(mode1);
        break;
      case '?':
        mode1 = 6;
        printMenu(mode1);
        //Timer2.pause(); //Start the timer counting
        break;
      case 'z':
        //Soft reset
        nvic_sys_reset();
        break;
      default:
        mode1 = 6;
        printMenu(mode1);
        //Timer2.pause(); //Start the timer counting
        break;
    }

    newData = false;
  }
  /*display.clearDisplay();
    display.setCursor(0,0);
    display.print("C1 V: ");
    display.println((int)vbat_1);
    display.print("C1 A: ");
    display.println((int)ibat_1);
    display.print("C1 T: ");
    display.println(temp);
    display.print("Vbuf: ");
    display.println(((float)analogRead(ABUFV))*2.447);
    display.setCursor(64,0);
    display.print("Stat: ");
    switch(state)
    {
    case 0: //Default state; determine to charge or discharge based on current SoC
      display.println("Init");
      break;
    case 1: //Discharge state; check for LVC, goto disconnect state if triggered
      display.println("DISc");
      break;
    case 2: //Disconnect/settling state after discharge, wait 5 minutes before charging
      display.println("DISf");
      break;
    case 3: //Charging state (CC)
      display.println("CHGi");
      break;
    case 4: //Charging state (CC/CV)
      display.println("CHGc");
      break;
    case 5:
      display.println("CHGf");
      break;
    case 6: //IR measure state
      display.println("IRm1");
      break;
    case 7: //IR measure state
      display.println("IRm2");
      break;
    case 8: //Parking state
      display.println("Wait");
      break;
    default:
      display.println("Err");
      break;
    }
    display.display();*/

  //delay(100);
  /*loopcnt++;
  if(loopcnt > 9)
    loopcnt = 0;*/
  if (interruptCounter > 16) {
    setChg1(DISCONNECT);
    setChg2(DISCONNECT);
    mode1 = 5;
    state1 = 8;
    mode2 = 5;
    state2 = 8;
    setLED1(RED);
    setLED2(RED);
    Serial.println("> Interrupt Overflow Error, Stopping Test!");
    Serial.print("> Interrupts Pending: ");
    Serial.println(interruptCounter);
    interruptCounter--;
    runStateMachine();
  }
  else if (interruptCounter > 0) {
    interruptCounter--;
    runStateMachine();
  }
  if (slot1_startup > 0)
  {
    slot1_startup--;
    if(slot1_startup == 0)
      digitalWrite(C1ON, LOW); //synchronous buck started, reconnect cell 
  }
  if (slot2_startup > 0)
  {
    slot2_startup--;
    if(slot2_startup == 0)
      digitalWrite(C2ON, LOW); //synchronous buck started, reconnect cell 
  }
}

//2kHz interrupt
void handler_loop(void) {
  interruptCounter++;
}


