/*
    Main code for the CCR v2.4
    Details here: http://rev0.net/index.php?title=CCR

    2019 Justin Kenny
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel-ANDnXOR.h>
#include <EEPROM.h>
#include "serial_support.h"

// Hardware Configurations:
// HW 2.0 - First CCR v2 HW using individual N- and P- gate drivers, max current 3A from 12V, 3.5A from 5V
// HW 2.4 - Second CCR v2 HW using synchronous gate drivers, max current +/-6A from 7.5-12V
// 1S - Using 220k/150k divider for 0-4.84V measurement
// 2S - Using 430k/150k divider for 0-9.46V measurement
// OLED - Using OLED display or not (not currently compatible with LEDs)
//#define OLED_ENABLED
//#define HW_1_0
//#define HW_2_0
#define HW_2_4
//#define REGEN_ENABLED
//#define MON_SHUNT
//#define V_1S //220k/150k 0-4.84V
#define V_2S //430k/150k 0-9.46V, 2s charging allowed
//#define V_2S1 //430k/150k 0-9.46V, only 1s charging allowed
#define LP //Low power, 4A limit, 1.65A shunt, 90kHz Fsw

#ifdef OLED_ENABLED
  #include <Adafruit_GFX_AS.h>
  #include <Adafruit_SSD1306_STM32.h>
  
  Adafruit_SSD1306 display(-1);

  #if (SSD1306_LCDHEIGHT != 32)
    #error("Height incorrect, please fix Adafruit_SSD1306.h!");
  #endif
#endif

volatile int interruptCounter;

const char vers[] = "2.0-02252020"; 

#define AFTERDISWAIT 300//300 //300s after charging wait time
#define AFTERCHGWAIT 60//60 //60s after charging wait time
#define LOOP_PERIOD 500 //In microseconds
#define CHARGE 1
#define DISCHARGE 2
#define DISCONNECT 0
#define REGEN 3
#define LED_OFF 0
#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3
#define LED_CYAN 4
#define LED_BLUE 5
#define LED_PURPLE 6
#define LED_WHITE 7
//Pin definitions - Analog inputs
#define AC1V PB0 //Cell 1 voltage input
#define AC1A PB1 //Cell 1 current input
#ifndef HW_1_0
  #define AC1T PA3 //Cell 1 temperature input
  #define AC1R PA1 //Cell 1 reverse voltage input
#endif
#define AC2V PA6 //Cell 2 voltage input
#define AC2A PA7 //Cell 2 current input
#ifndef HW_1_0
  #define AC2T PA2 //Cell 2 temperature input
  #define AC2R PA0 //Cell 2 reverse voltage input
  #define ABUFV PA4 //Buffer/input voltage input
#endif
#define AIREF PA5 //Reference voltage input
//Pin definitions - Digital outputs
#define OC1NF2 PB6 //CC 1 setting output
#define OC2NF2 PB7 //CC 2 setting output
#define C1DOFF PB15 //CC 1 disable output
#define C2DOFF PA15 //CC 2 disable output
#define C1ON PB14 //Cell 1 protection bypass
#define C2ON PB3 //Cell 2 protection bypass
#define FANON PB4 //Fan control outputc1
#define WSDI PB13 //LED control pin, inverted logic
#define HSTH1 0
#define HSTH2 1
#define BUFV 2
//Pin definitions - Digital inputs
#define S1 PB9 //Switch 1
#define S2 PC13 //Switch 2
#ifdef HW_1_0
  #define MAX_CHG_CUR -1500
  #define MAX_CHG_VOL 4500
  #define MIN_CHG_VOL 1000
  #define MIN_CCC 10
  #define MAX_DIS_CUR 1500
  #define MIN_DIS_VOL 700
  #define MAX_DIS_VOL 4300
  #define MINBUCKDUTY 0
  #define MAXBUCKDUTY 400
  #define MAX_VBUF 12600
  #define DEF_CHG_CUR -1500
  #define DEF_CHG_VOL 4200
  #define DEF_CCC 50
  #define DEF_DIS_CUR 1500
  #define DEF_DIS_VOL 2700
  //CCR v1.0 HW configuration: (ToDo: Update)
  #define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
  #define AUXSEL1 PB2 //Lo = ABUFV, Hi = HS thermistor #1
  #define OC1NF1 PA8 //Boost NFET output
  #define OC1PF PA9 //Buck PFET output, negative logic; lower PWM duty = higher output voltage
  #define OC2NF1 PA10 //Boost NFET output
  #define OC2PF PB8 //Buck PFET output, negative logic; lower PWM duty = higher output voltage
  #define AC1R PA2
  #define AC2R PA1
  #define ABUFV PA0
  #define AC1T PA4
  #define AC2T PA3
  #ifdef V_1S
    #define OVV_THRESH 4370
  #endif
  #ifdef V_2S1
    #define OVV_THRESH 4370
  #endif
  #ifdef V_2S
    #define OVV_THRESH 8770
  #endif
#endif
#ifdef HW_2_0
  #define MAX_CHG_CUR -3500
  #define MIN_CHG_VOL 1000
  #define MIN_CCC 10
  #define MAX_DIS_CUR 5000
  #define MIN_DIS_VOL 700
  #define MINBUCKDUTY 0
  #define MAXBUCKDUTY 400
  #define MAX_VBUF 12600
  #define DEF_CHG_CUR -1500
  #define DEF_CHG_VOL 4200
  #define DEF_CCC 50
  #define DEF_DIS_CUR 1500
  #define DEF_DIS_VOL 2700
  //CCR v2.0 HW configuration:
  #define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
  #define AUXSEL1 PB2 //Lo = ABUFV, Hi = HS thermistor #1
  #define OC1NF1 PA8 //Boost NFET output
  #define OC1PF PA9 //Buck PFET output, negative logic; lower PWM duty = higher output voltage
  #define OC2NF1 PA10 //Boost NFET output
  #define OC2PF PB8 //Buck PFET output, negative logic; lower PWM duty = higher output voltage
  #ifdef V_1S
    #define OVV_THRESH 4370
    #define MAX_CHG_VOL 4500
    #define MAX_DIS_VOL 4300
  #endif
  #ifdef V_2S1
    #define OVV_THRESH 4370
    #define MAX_CHG_VOL 4500
    #define MAX_DIS_VOL 4300
  #endif
  #ifdef V_2S
    #define OVV_THRESH 8770
    #define MAX_CHG_VOL 8700
    #define MAX_DIS_VOL 8400
  #endif
#endif
#ifdef HW_2_4
  #ifdef LP
    #define MAX_CHG_CUR -1500
  #else
    #define MAX_CHG_CUR -6500
  #endif
  #define MIN_CHG_VOL 1000
  #define MIN_CCC 10
  #ifdef REGEN_ENABLED
    #ifdef LP
      #define MAX_DIS_CUR 1500
    #else
      #define MAX_DIS_CUR 6500
    #endif
  #else
    #ifdef LP
      #define MAX_DIS_CUR 1500
    #else
      #define MAX_DIS_CUR 5000
    #endif
  #endif
  #define MIN_DIS_VOL 700
  #define MINBUCKDUTY 1
  #ifdef LP
    #define MAXBUCKDUTY 794
  #else
    #define MAXBUCKDUTY 397
  #endif
  #define MAX_VBUF 12600
  #define DEF_CHG_CUR -1500
  #define DEF_CHG_VOL 4200
  #define DEF_CCC 50
  #define DEF_DIS_CUR 1500
  #define DEF_DIS_VOL 2700
  //CCR v2.2 HW configuration:
  #define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
  #define AUXSEL1 PB12 //Lo = ABUFV, Hi = HS thermistor #1
  #define OC1PF PA8 //Buck FET control output, positive logic; higher PWM duty = higher output voltage
  #define OC1ON PA9 //Buck disable, active low (0 = buck disabled)
  #define OC2PF PA10 //Buck FET control output, positive logic; higher PWM duty = higher output voltage
  #define OC2ON PB8 //Buck disable, active low (0 = buck disabled)
  #ifdef V_1S
    #define OVV_THRESH 4370
    #define MAX_CHG_VOL 4500
    #define MAX_DIS_VOL 4300
  #endif
  #ifdef V_2S1
    #define OVV_THRESH 4370
    #define MAX_CHG_VOL 4500
    #define MAX_DIS_VOL 4300
  #endif
  #ifdef V_2S
    #define OVV_THRESH 8770
    #define MAX_CHG_VOL 8700
    #define MAX_DIS_VOL 8400
  #endif
#endif
#define DEF_DIS_MODE 0
#define DEF_PSU_MODE 1
#define DEF_CYCLES 1
#define OVT_THRESH 45
#define LED_BRIGHTNESS 80 //1-255 for LED brightness
#define NIMH_DV 10
#define DEF_CELL_TYPE 0
#define STARTUP_CYCLES 5 //number of cycles-1 (0.25ms each) to delay before turning on cell

volatile int16 charge_current_1 = -1500;
volatile uint16 charge_voltage_1 = 4200;
volatile uint16 discharge_current_1 = 1500;
volatile uint16 discharge_voltage_1 = 2700;
//volatile int16 charge_power_1 = -1500;
//volatile uint16 discharge_power_1 = 1500;
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
volatile uint16 cell1_vmax = 0;
volatile uint16 cell2_vmax = 0;
volatile uint8 cell1_type = 0; //0 = Li-Ion, 1 = NiMH
volatile uint8 cell2_type = 0;
volatile uint8 displayEnabled = 0;
volatile uint8 fanSpeed = 0; //0 = off, 1 = pwm, 2 = on

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
//EEPROM address for slot 2 voltage value: 2
#define ADC2V2ADDR 2
#ifdef V_1S //220k/150k 0-4.84V
  #define ADC2V1INIT 846.281f
  #define ADC2V2INIT 846.281f
#endif
#ifdef V_2S //430k/150k 0-9.46V
  #define ADC2V1INIT 432.981f
  #define ADC2V2INIT 432.981f
#endif
#ifdef V_2S1 //430k/150k 0-9.46V
  #define ADC2V1INIT 432.981f
  #define ADC2V2INIT 432.981f
#endif
//todo: reset adc2v2init during "l" calibration for storing correction factors
//EEPROM address for slot 1 current value: 3
#define ADC2I1ADDR 3
#ifndef MON_SHUNT
  #ifdef LP
    #define ADC2I1INIT 1241.212f //20m shunt
  #else
    #define ADC2I1INIT 310.303f //5m shunt
  #endif
#endif
#ifdef MON_SHUNT
  #define ADC2I1INIT 124.121f //2m shunt
#endif
//EEPROM address for slot 2 current value: 4
#define ADC2I2ADDR 4
#ifdef LP
  #define ADC2I2INIT 1241.212f
#else
  #define ADC2I2INIT 310.303f
#endif
//EEPROM address for buffer current value: 5
#define BUF2VADDR 5
#define BUF2VINIT 3.22266f //x/4096*3.3*1000*4
volatile float BUF2V = BUF2VINIT;
#define VR12V 2.41699f
#define VR22V 2.41699f
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

Adafruit_NeoPixel leds = Adafruit_NeoPixel(2, WSDI, NEO_GRB + NEO_KHZ800);

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
volatile uint32 tick = 0;
volatile uint32 last_tick = 0;
volatile uint32 last_tick_fan = 0;
//HardwareTimer timer1(1);
//HardwareTimer timer2(2);

boolean fantoggle = false;

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

unsigned short getChgPwr(){
  unsigned long power = 0;
  
  if ((state1 == 3) || (state1 == 4)) //Charging states
  {
    power = (charge_current_1*charge_voltage_1*-1)/1000;
  }
  if ((state2 == 3) || (state2 == 4)) //Charging states
  {
    power += (charge_current_2*charge_voltage_2*-1)/1000;
  }
  
  return (unsigned short)power; //Returns power in mW
}

unsigned short getDisPwr(){
  unsigned long power = 0;
  
  if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
  {
    power = discharge_current_1*charge_voltage_1/1000;
  }
  if ((state2 == 1) || (state2 == 6) || (state2 == 7)) //Discharging states
  {
    power += discharge_current_2*charge_voltage_2/1000;
  }

  return (unsigned short)power; //Returns power in mW
}

void setChg1(unsigned char state) {
  switch (state)
  {
    /*#define OC1NF1 PA8 //Boost NFET
      #define OC1PF PA9 //Buck PFET
      #define OC1NF2 PB6 //CC setting
      #define C1DOFF PB15*/
    case DISCONNECT:
      #ifdef HW_2_0
        pinMode(OC1NF1, OUTPUT);
        digitalWrite(OC1NF1, LOW);
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, HIGH);
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, HIGH); //Non-inverting mode
      #endif
      #ifdef HW_2_4
        digitalWrite(C1ON, LOW); //disable slot cell protection FET
        pinMode(OC1ON, OUTPUT);
        digitalWrite(OC1ON, LOW);
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, LOW);
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, HIGH);
      #endif
      break;
    case CHARGE:
      #ifdef HW_2_0
        pinMode(OC1NF1, OUTPUT);
        digitalWrite(OC1NF1, LOW);
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, HIGH); //Inverting
        duty1 = MAXBUCKDUTY;
        pinMode(OC1PF, PWM);
        pwmWrite(OC1PF, duty1);
      #endif
      #ifdef HW_2_4
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
        digitalWrite(C1ON, LOW); //override cell protection FET off until synchronous buck is started
      #endif
      break;
    case DISCHARGE:
      #ifdef HW_2_0
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, HIGH); //Non-inverting mode
        pinMode(OC1NF1, OUTPUT);
        digitalWrite(OC1NF1, LOW);
        duty1 = 0;
        pinMode(OC1NF2, PWM);
        pwmWrite(OC1NF2, duty1);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, LOW); //Non-inverting mode
      #endif
      #ifdef HW_2_4
        pinMode(OC1ON, OUTPUT);
        digitalWrite(OC1ON, LOW);
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, LOW); //Non-inverting mode
        duty1 = 0;
        pinMode(OC1NF2, PWM);
        pwmWrite(OC1NF2, duty1);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, LOW); //Non-inverting mode
        digitalWrite(C1ON, HIGH); //enable slot cell protection FET
      #endif
      break;
    default:
      #ifdef HW_2_0
        pinMode(OC1NF1, OUTPUT);
        digitalWrite(OC1NF1, LOW);
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, HIGH);
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, HIGH); //Non-inverting mode
      #endif
      #ifdef HW_2_4
        digitalWrite(C1ON, LOW); //disable slot cell protection FET
        pinMode(OC1ON, OUTPUT);
        digitalWrite(OC1ON, LOW);
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, LOW);
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW);
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, HIGH);
      #endif
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
      #ifdef HW_2_0
        pinMode(OC2NF1, OUTPUT);
        digitalWrite(OC2NF1, LOW);
        pinMode(OC2NF2, OUTPUT);
        digitalWrite(OC2NF2, LOW);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, HIGH);
        pinMode(OC2PF, OUTPUT);
        digitalWrite(OC2PF, HIGH); //Non-inverting mode
      #endif
      #ifdef HW_2_4
        digitalWrite(C2ON, LOW); //disable slot cell protection FET
        pinMode(OC2ON, OUTPUT);
        digitalWrite(OC2ON, LOW);
        pinMode(OC2PF, OUTPUT);
        digitalWrite(OC2PF, LOW);
        pinMode(OC2NF2, OUTPUT);
        digitalWrite(OC2NF2, LOW);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, HIGH);
      #endif
      break;
    case CHARGE:
      #ifdef HW_2_0
        pinMode(OC2NF1, OUTPUT);
        digitalWrite(OC2NF1, LOW);
        pinMode(OC2NF2, OUTPUT);
        digitalWrite(OC2NF2, LOW);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, HIGH); //Inverting
        duty2 = MAXBUCKDUTY;
        pinMode(OC2PF, PWM);
        pwmWrite(OC2PF, duty2);
      #endif
      #ifdef HW_2_4
        pinMode(OC2NF2, OUTPUT);
        digitalWrite(OC2NF2, LOW);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, HIGH); //Inverting
        presetDuty2();
        duty2 = initbuckduty2; //Assume 3.7V initial cell voltage - optimise later
        pinMode(OC2PF, PWM);
        pwmWrite(OC2PF, duty2);
        pinMode(OC2ON, OUTPUT);
        digitalWrite(OC2ON, HIGH);
        slot2_startup = STARTUP_CYCLES;
        digitalWrite(C2ON, LOW); //override cell protection FET off until synchronous buck is started
      #endif
      break;
    case DISCHARGE:
      #ifdef HW_2_0
        pinMode(OC2PF, OUTPUT);
        digitalWrite(OC2PF, HIGH); //Non-inverting mode
        pinMode(OC2NF1, OUTPUT);
        digitalWrite(OC2NF1, LOW);
        duty2 = 0;
        pinMode(OC2NF2, PWM);
        pwmWrite(OC2NF2, duty2);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, LOW); //Non-inverting mode
      #endif
      #ifdef HW_2_4
        pinMode(OC2ON, OUTPUT);
        digitalWrite(OC2ON, LOW);
        pinMode(OC2PF, OUTPUT);
        digitalWrite(OC2PF, LOW); //Non-inverting mode
        duty2 = 0;
        pinMode(OC2NF2, PWM);
        pwmWrite(OC2NF2, duty2);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, LOW); //Non-inverting mode
        digitalWrite(C2ON, HIGH); //enable slot cell protection FET
      #endif
      break;
    default:
      #ifdef HW_2_0
        pinMode(OC2NF1, OUTPUT);
        digitalWrite(OC2NF1, LOW);
        pinMode(OC2NF2, OUTPUT);
        digitalWrite(OC2NF2, LOW);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, HIGH);
        pinMode(OC2PF, OUTPUT);
        digitalWrite(OC2PF, HIGH); //Non-inverting mode
      #endif
      #ifdef HW_2_4
        digitalWrite(C2ON, LOW); //disable slot cell protection FET
        pinMode(OC2ON, OUTPUT);
        digitalWrite(OC2ON, LOW);
        pinMode(OC2PF, OUTPUT);
        digitalWrite(OC2PF, LOW);
        pinMode(OC2NF2, OUTPUT);
        digitalWrite(OC2NF2, LOW);
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, HIGH);
      #endif
      break;
  }
}

void presetDuty1(void) {
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

void presetDuty2(void) {
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

void pauseInts(void) {
  Timer1.pause();
  Timer4.pause();
  Timer2.pause();
}

void resumeInts(void) {
  Timer1.resume();
  Timer4.resume();
  Timer2.resume();
}

void setLED1(unsigned char color) {
  switch (color)
  {
    case LED_OFF:
      leds.setPixelColor(0,0,0,0);
      leds.show();
      break;
    case LED_RED:
      leds.setPixelColor(0,LED_BRIGHTNESS,0,0);
      leds.show();
      break;
    case LED_YELLOW:
      leds.setPixelColor(0,LED_BRIGHTNESS,LED_BRIGHTNESS,0);
      leds.show();
      break;
    case LED_GREEN:
      leds.setPixelColor(0,0,LED_BRIGHTNESS,0);
      leds.show();
      break;
    case LED_CYAN:
      leds.setPixelColor(0,0,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      break;
    case LED_BLUE:
      leds.setPixelColor(0,0,0,LED_BRIGHTNESS);
      leds.show();
      break;
    case LED_PURPLE:
      leds.setPixelColor(0,LED_BRIGHTNESS,0,LED_BRIGHTNESS);
      leds.show();
      break;
    case LED_WHITE:
      leds.setPixelColor(0,LED_BRIGHTNESS,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      break;
    default:
      leds.setPixelColor(0,0,0,0);
      leds.show();
      break;
  }
}

void setLED2(unsigned char color) {
  switch (color)
  {
    case LED_OFF:
      leds.setPixelColor(1,0,0,0);
      leds.show();
      break;
    case LED_RED:
      leds.setPixelColor(1,LED_BRIGHTNESS,0,0);
      leds.show();
      break;
    case LED_YELLOW:
      leds.setPixelColor(1,LED_BRIGHTNESS,LED_BRIGHTNESS,0);
      leds.show();
      break;
    case LED_GREEN:
      leds.setPixelColor(1,0,LED_BRIGHTNESS,0);
      leds.show();
      break;
    case LED_CYAN:
      leds.setPixelColor(1,0,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      break;
    case LED_BLUE:
      leds.setPixelColor(1,0,0,LED_BRIGHTNESS);
      leds.show();
      break;
    case LED_PURPLE:
      leds.setPixelColor(1,LED_BRIGHTNESS,0,LED_BRIGHTNESS);
      leds.show();
      break;
    case LED_WHITE:
      leds.setPixelColor(1,LED_BRIGHTNESS,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      break;
    default:
      leds.setPixelColor(1,0,0,0);
      leds.show();
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
  pinMode(FANON, OUTPUT);
  digitalWrite(FANON, LOW);
  adciref = analogRead(AIREF); //Iref voltage

  leds.begin();

  setLED1(LED_OFF);
  setLED2(LED_OFF);

  //Timer1 = A8, (A9, )A10 used for buck PWM
  Timer1.pause();
  Timer1.setPrescaleFactor(1);
  #ifdef LP
    Timer1.setOverflow(800); //180kHz = 400, 90kHz = 800
  #else
    Timer1.setOverflow(400); //180kHz = 400, 90kHz = 800
  #endif
  Timer1.refresh();
  Timer1.resume();

  //Timer4 = B6, B7(, B8) used for CC load
  Timer4.pause();
  Timer4.setPrescaleFactor(1);
  #ifdef LP
    Timer4.setOverflow(800); //180kHz = 400, 90kHz = 800
  #else
    Timer4.setOverflow(400); //180kHz = 400, 90kHz = 800
  #endif
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

  #ifdef OLED_ENABLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
  #endif

  #ifdef HW_1_0
  Serial.print("> HW Cfg: 1.0");
  #endif
  #ifdef HW_2_0
  Serial.print("> HW Cfg: 2.0");
  #endif
  #ifdef HW_2_4
  Serial.print("> HW Cfg: 2.4");
  #endif
  #ifdef MON_SHUNT
  Serial.print("_Mon");
  #endif
  #ifdef V_1S
  Serial.print("_1s");
  #endif
  #ifdef V_2S
  Serial.print("_2s");
  #endif
  #ifdef V_2S1
  Serial.print("_2s1");
  #endif
  #ifdef LP
  Serial.print("_LP");
  #endif
  #ifdef OLED_ENABLED
  Serial.println(" OLED Enabled");
  #endif
  Serial.println("");

  estatus = EEPROM.init();
  if(estatus != 0)
  {
    Serial.print("> EEPROM init err: ");
    Serial.println(estatus, HEX);
  }
  
  Serial.println("> Init cal from EEPROM");
  estatus = EEPROM.read(REFAVALADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 6800 || wData < 6200)
    {
      Serial.println("> Ref cal out of range, using default.");
    }
    else
    {
      Serial.print("> Ref cal value: 1.");
      Serial.print(wData);
      Serial.println("V");
      REFAVAL = (10000.0+(float)wData)*124.1212;
    }
  }
  else
  {
    Serial.println("> Ref cal not found, using default.");
  }
  estatus = EEPROM.read(ADC2V1ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4600 || wData < 3800)
    {
      Serial.println("> S1 Vcal out of range, using default.");
    }
    else
    {
      Serial.print("> S1 Vcal value: ");
      Serial.print(wData);
      Serial.println("mV");
      //Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
      ADC2V1 = ADC2V1 * 4200.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> S1 Vcal not found, using default.");
  }
  estatus = EEPROM.read(ADC2V2ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4600 || wData < 3800)
    {
      Serial.println("> S2 Vcal out of range, using default.");
    }
    else
    {
      Serial.print("> S2 Vcal value: ");
      Serial.print(wData);
      Serial.println("mV");
      //Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
      ADC2V2 = ADC2V2 * 4200.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> S2 Vcal not found, using default.");
  }
  estatus = EEPROM.read(ADC2I1ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 1200 || wData < 800)
    {
      Serial.println("> S1 Ical out of range, using default.");
    }
    else
    {
      Serial.print("> S1 Ical value: ");
      Serial.print(wData);
      Serial.println("mA");
      //Current reported / Current real * ADC2IxINIT orig = ADC2IxINIT new
      ADC2I1 = ADC2I1 * 1000.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> S1 Ical not found, using default.");
  }
  estatus = EEPROM.read(ADC2I2ADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 1200 || wData < 800)
    {
      Serial.println("> S2 Ical out of range, using default.");
    }
    else
    {
      Serial.print("> S2 Ical value: ");
      Serial.print(wData);
      Serial.println("mA");
      //Current reported / Current real * ADC2IxINIT orig = ADC2IxINIT new
      ADC2I2 = ADC2I2 * 1000.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> S2 Ical not found, using default.");
  }
  estatus = EEPROM.read(BUF2VADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 6000 || wData < 4000)
    {
      Serial.println("> Input cal out of range, using default.");
    }
    else
    {
      Serial.print("> Input cal value: ");
      Serial.print(wData);
      Serial.println("mV");
      BUF2V = (5000.0/(float)wData)*BUF2VINIT;
    }
  }
  else
  {
    Serial.println("> Input cal not found, using default.");
  }

  Serial.println("> Init ref...");
  adciref = 0;
  for(i=0;i<1000;i++)
  {
    adciref += analogRead(AIREF); //Iref voltage
    //Serial.println(adciref);
  }
  Serial.print("> ADC corr: ");
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
  
  Serial.print("> SW ver: ");
  Serial.println(vers);
  adciref = (uint32)((float)adciref/1000.0); //Iref voltage
  
  printMenu(mode1, Serial);
  
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
  char startMarkers[] = {'c', 'd', 'y', 'p', 't', 'n', '?', 'v', 's', 'l', 'r', 'z', 'q', 'a'};
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
          //receivedChars[ndx] = 0;
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

    else if (rc == startMarkers[0] || rc == startMarkers[1] || rc == startMarkers[2] || rc == startMarkers[3] || rc == startMarkers[9] || rc == startMarkers[11] || rc == startMarkers[13]
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

void parseCharge1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default charge current/voltage/cutoff
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  ccc_1 = DEF_CCC;
  cell1_type = DEF_CELL_TYPE;

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
          if(strVal<MIN_CHG_VOL)
            strVal = MIN_CHG_VOL;
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
        case 'n':
          Serial.print("> Using cell type: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<0)
            strVal = 0;
          else if(strVal>1)
            strVal = 1;
          cell1_type = strVal;
          if(cell1_type == 0)
            Serial.println("Li*");
          else
            Serial.println("Ni*");
          break;
        default:
          break;
      }
    }
  }
  
  if(charge_voltage_1 == DEF_CHG_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(charge_voltage_1);
    Serial.println("mV");
  }
  if(charge_current_1 == DEF_CHG_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(charge_current_1);
    Serial.println("mA");
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
  if(cell1_type == DEF_CELL_TYPE)
  {
    Serial.print("> Using def cell type: ");
    if(cell1_type == 0)
      Serial.println("Li*");
    else
      Serial.println("Ni*");
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
  psu_dir_1 = DEF_DIS_MODE;

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
          else if(strVal<30)
            strVal = 30;
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
            Serial.println("Constant");
          else
            Serial.println("Stepped");
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else if(psu_dir_1 == 2)
            Serial.println("Regen");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_1 == DEF_DIS_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(discharge_voltage_1);
    Serial.println("mV");
  }
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(discharge_mode_1 == DEF_DIS_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
      Serial.println("Constant");
    else
      Serial.println("Stepped");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_1 == 0)
    {
      Serial.println("Resistive");
    }
    else if(psu_dir_1 == 2)
    {
      Serial.println("Regen");
    }
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
          else if(strVal<30)
            strVal = 30;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else
            Serial.println("Regenerative");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
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
          Serial.print("> Using dir: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Discharge");
          else if(psu_dir_1 == 1)
            Serial.println("Buck");
          else if(psu_dir_1 == 2)
            Serial.println("Boost");
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
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    Serial.println("Buck");
    if(charge_voltage_1 == DEF_CHG_VOL)
    {
      Serial.print("> Using def voltage: ");
      Serial.print(charge_voltage_1);
      Serial.println("mV");
    }
  }
  else
  {
    if(discharge_voltage_1 == DEF_DIS_VOL)
    {
      Serial.print("> Using def voltage: ");
      Serial.print(discharge_voltage_1);
      Serial.println("mV");
    }
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
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
          Serial.print("> Using dir: ");
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
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
    {
      Serial.println("Boost");
      if(discharge_voltage_1 == DEF_DIS_VOL)
      {
        Serial.print("> Using def voltage: ");
        Serial.print(discharge_voltage_1);
        Serial.println("mV");
      }
    }
    else
    {
      Serial.println("Buck");
      if(charge_voltage_1 == DEF_CHG_VOL)
      {
        Serial.print("> Using def voltage: ");
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
  psu_dir_1 = DEF_PSU_MODE;
  //Set default charge current/voltage/cutoff
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  ccc_1 = DEF_CCC;
  num_cycles_1 = DEF_CYCLES;
  cell1_type = DEF_CELL_TYPE;
  
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
          else if(strVal<30)
            strVal = 30;
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
            Serial.println("Constant");
          else
            Serial.println("Stepped");
          break;
        case 'k':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-30)
            strVal = -30;
          charge_current_1 = strVal;
          Serial.print(charge_current_1);
          Serial.println("mA");
          break;
        case 'u':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CHG_VOL)
            strVal = MIN_CHG_VOL;
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
          else if(strVal>2000)
            strVal = 2000;
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
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else if(psu_dir_1 == 2)
            Serial.println("Regen");
          break;
        case 'n':
          Serial.print("> Using cell type: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<0)
            strVal = 0;
          else if(strVal>1)
            strVal = 1;
          cell1_type = strVal;
          if(cell1_type == 0)
            Serial.println("Li*");
          else
            Serial.println("Ni*");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_1 == DEF_DIS_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(discharge_voltage_1);
    Serial.println("mV");
  }
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(discharge_mode_1 == DEF_DIS_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
      Serial.println("Constant");
    else
      Serial.println("Stepped");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_1 == 0)
    {
      Serial.println("Resistive");
    }
    else if(psu_dir_1 == 2)
    {
      Serial.println("Regen");
    }
  }
  if(charge_voltage_1 == DEF_CHG_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(charge_voltage_1);
    Serial.println("mV");
  }
  if(charge_current_1 == DEF_CHG_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(charge_current_1);
    Serial.println("mA");
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
  if(num_cycles_1 == DEF_CYCLES)
  {
    Serial.print("> Using def cycles: ");
    Serial.println(num_cycles_1);
  }
  if(cell1_type == DEF_CELL_TYPE)
  {
    Serial.print("> Using def cell type: ");
    if(cell1_type == 0)
      Serial.println("Li*");
    else
      Serial.println("Ni*");
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
  cell2_type = DEF_CELL_TYPE;

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
          else if(strVal>-30)
            strVal = -30;
          charge_current_2 = strVal;
          Serial.print(charge_current_2);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CHG_VOL)
            strVal = MIN_CHG_VOL;
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
          else if(strVal>2000)
            strVal = 2000;
          ccc_2 = strVal;
          Serial.print(ccc_2);
          Serial.println("mA");
          break;
        case 'n':
          Serial.print("> Using cell type: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<0)
            strVal = 0;
          else if(strVal>1)
            strVal = 1;
          cell2_type = strVal;
          if(cell2_type == 0)
            Serial.println("Li*");
          else
            Serial.println("Ni*");
          break;
        default:
          break;
      }
    }
  }
  
  if(charge_voltage_2 == DEF_CHG_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(charge_voltage_2);
    Serial.println("mV");
  }
  if(charge_current_2 == DEF_CHG_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(charge_current_2);
    Serial.println("mA");
  }
  if(ccc_2 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_2);
    Serial.println("mA");
  }
  if(cell2_type == DEF_CELL_TYPE)
  {
    Serial.print("> Using def cell type: ");
    if(cell2_type == 0)
      Serial.println("Li*");
    else
      Serial.println("Ni*");
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
  psu_dir_2 = DEF_DIS_MODE;

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
          else if(strVal<30)
            strVal = 30;
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
            Serial.println("Constant");
          else
            Serial.println("Stepped");
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_2 = strVal;
          if(psu_dir_2 == 0)
            Serial.println("Resistive");
          else if(psu_dir_1 == 2)
            Serial.println("Regen");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_2 == DEF_DIS_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(discharge_voltage_2);
    Serial.println("mV");
  }
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(discharge_mode_2 == DEF_DIS_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_2 == 0)
      Serial.println("Constant");
    else
      Serial.println("Stepped");
  }
  if(psu_dir_2 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_2 == 0)
    {
      Serial.println("Resistive");
    }
    else if(psu_dir_1 == 2)
    {
      Serial.println("Regen");
    }
  }
}

void parseIR2(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current
  discharge_current_2 = DEF_DIS_CUR;
  psu_dir_1 = DEF_PSU_MODE;

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
          else if(strVal<30)
            strVal = 30;
          discharge_current_2 = strVal;
          Serial.print(discharge_current_2);
          Serial.println("mA");
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_2 = strVal;
          if(psu_dir_2 == 0)
            Serial.println("Resistive");
          else
            Serial.println("Regenerative");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_1 == 0)
    {
      Serial.println("Resistive");
    }
    else
    {
      Serial.println("Regen");
    }
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
          else if(strVal<30)
            strVal = 30;
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
          Serial.print("> Using dir: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
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
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    Serial.println("Buck");
    if(charge_voltage_2 == DEF_CHG_VOL)
    {
      Serial.print("> Using def voltage: ");
      Serial.print(charge_voltage_2);
      Serial.println("mV");
    }
  }
  else
  {
    if(discharge_voltage_2 == DEF_DIS_VOL)
    {
      Serial.print("> Using def voltage: ");
      Serial.print(discharge_voltage_2);
      Serial.println("mV");
    }
  }
  if(ccc_2 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
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
          Serial.print("> Using dir: ");
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
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
    {
      Serial.println("Boost");
      if(discharge_voltage_1 == DEF_DIS_VOL)
      {
        Serial.print("> Using def voltage: ");
        Serial.print(discharge_voltage_1);
        Serial.println("mV");
      }
    }
    else
    {
      Serial.println("Buck");
      if(charge_voltage_1 == DEF_CHG_VOL)
      {
        Serial.print("> Using def voltage: ");
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
  psu_dir_1 = DEF_PSU_MODE;
  //Set default charge current/voltage/cutoff
  charge_voltage_2 = DEF_CHG_VOL;
  charge_current_2 = DEF_CHG_CUR;
  ccc_2 = DEF_CCC;
  num_cycles_2 = DEF_CYCLES;
  cell2_type = DEF_CELL_TYPE;
  
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
          else if(strVal<30)
            strVal = 30;
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
            Serial.println("Constant");
          else
            Serial.println("Stepped");
          break;
        case 'k':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-30)
            strVal = -30;
          charge_current_2 = strVal;
          Serial.print(charge_current_2);
          Serial.println("mA");
          break;
        case 'u':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CHG_VOL)
            strVal = MIN_CHG_VOL;
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
          else if(strVal>2000)
            strVal = 2000;
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
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else if(psu_dir_1 == 2)
            Serial.println("Regen");
          break;
        case 'n':
          Serial.print("> Using cell type: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<0)
            strVal = 0;
          else if(strVal>1)
            strVal = 1;
          cell2_type = strVal;
          if(cell2_type == 0)
            Serial.println("Li*");
          else
            Serial.println("Ni*");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_2 == DEF_DIS_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(discharge_voltage_2);
    Serial.println("mV");
  }
  if(discharge_current_2 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_2);
    Serial.println("mA");
  }
  if(discharge_mode_2 == DEF_DIS_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_2 == 0)
      Serial.println("Constant");
    else
      Serial.println("Stepped");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_1 == 0)
    {
      Serial.println("Resistive");
    }
    else if(psu_dir_1 == 2)
    {
      Serial.println("Regen");
    }
  }
  if(charge_voltage_2 == DEF_CHG_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(charge_voltage_2);
    Serial.println("mV");
  }
  if(charge_current_2 == DEF_CHG_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(charge_current_2);
    Serial.println("mA");
  }
  if(ccc_2 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_2);
    Serial.println("mA");
  }
  if(num_cycles_2 == DEF_CYCLES)
  {
    Serial.print("> Using def cycles: ");
    Serial.println(num_cycles_2);
  }
  if(cell2_type == DEF_CELL_TYPE)
  {
    Serial.print("> Using def cell type: ");
    if(cell2_type == 0)
      Serial.println("Li*");
    else
      Serial.println("Ni*");
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
        #ifdef HW_2_4
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
        #endif
        #ifdef HW_2_0
          duty1--;
          if (duty1 < MINBUCKDUTY)
            duty1 = MINBUCKDUTY;
        #endif
      }
      else //Vbat >= 4.2V, Ibat < 1.5A
      {
        #ifdef HW_2_4
          duty1--;
          if (duty1 < MINBUCKDUTY)
            duty1 = MINBUCKDUTY;
        #endif
        #ifdef HW_2_0
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
        #endif
      }
    }
    else //Ibat >= 1.5A
    {
      #ifdef HW_2_4
        duty1--;
        if (duty1 < MINBUCKDUTY)
          duty1 = MINBUCKDUTY;
      #endif
      #ifdef HW_2_0
        duty1++;
        if (duty1 > MAXBUCKDUTY)
          duty1 = MAXBUCKDUTY;
      #endif
    }
    pwmWrite(OC1PF, duty1);
  }
  if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
  {
    #ifdef HW_2_4
      if(psu_dir_1 < 2)
      {
        if (ibat_now1 < discharge_current_1) //Ibat < 1.5A
        {
          if (vbat_now1 > (discharge_voltage_1 - 100)) //Vbat > 2.7V, Ibat < 1.5A
          {
            duty1++;
            if (duty1 > MAXBUCKDUTY)
              duty1 = MAXBUCKDUTY;
          }
          else //Vbat <= 2.7V, Ibat < 1.5A
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
        pwmWrite(OC1NF2, duty1); //CC Load
      }
      else
      {
        if (ibat_now1 < discharge_current_1) //Ibat < 1.5A
        {
          if (vbat_now1 > (discharge_voltage_1 - 100)) //Vbat > 2.7V, Ibat < 1.5A
          { 
            duty1--;
            if (duty1 < MINBUCKDUTY)
              duty1 = MINBUCKDUTY;
          }
          else //Vbat <= 2.7V, Ibat < 1.5A
          {
            duty1++;
            if (duty1 > MAXBUCKDUTY)
              duty1 = MAXBUCKDUTY;
          }
        }
        else //Ibat >= 1.5A
        {
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
        }
        pwmWrite(OC1PF, duty1);
      }
    #endif
    #ifdef HW_2_0
      if (ibat_now1 < discharge_current_1) //Ibat < 1.5A
      {
        if (vbat_now1 > discharge_voltage_1) //Vbat > 2.7V, Ibat < 1.5A
        {
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
        }
        else //Vbat <= 2.7V, Ibat < 1.5A
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
      pwmWrite(OC1NF2, duty1); //CC Load
    #endif
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
        #ifdef HW_2_4
          duty2++;
          if (duty2 > MAXBUCKDUTY)
            duty2 = MAXBUCKDUTY;
        #endif
        #ifdef HW_2_0
          duty2--;
          if (duty2 < MINBUCKDUTY)
            duty2 = MINBUCKDUTY;
        #endif
      }
      else //Vbat >= 4.2V, Ibat < 1.5A
      {
        #ifdef HW_2_4
          duty2--;
          if (duty2 < MINBUCKDUTY)
            duty2 = MINBUCKDUTY;
        #endif
        #ifdef HW_2_0
          duty2++;
          if (duty2 > MAXBUCKDUTY)
            duty2 = MAXBUCKDUTY;
        #endif
      }
    }
    else //Ibat >= 1.5A
    {
      #ifdef HW_2_4
        duty2--;
        if (duty2 < MINBUCKDUTY)
          duty2 = MINBUCKDUTY;
      #endif
      #ifdef HW_2_0
        duty2++;
        if (duty2 > MAXBUCKDUTY)
          duty2 = MAXBUCKDUTY;
      #endif
    }
    pwmWrite(OC2PF, duty2);
  }
  if ((state2 == 1) || (state2 == 6) || (state2 == 7)) //Discharging states
  {
    #ifdef HW_2_4
      if(psu_dir_2 < 2)
      {
        if (ibat_now2 < discharge_current_2) //Ibat < 1.5A
        {
          if (vbat_now2 > (discharge_voltage_2 - 100)) //Vbat > 2.7V, Ibat < 1.5A
          {
            duty2++;
            if (duty2 > MAXBUCKDUTY)
              duty2 = MAXBUCKDUTY;
          }
          else //Vbat <= 2.7V, Ibat < 1.5A
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
        pwmWrite(OC2NF2, duty2); //CC Load
      }
      else
      {
        if (ibat_now2 < discharge_current_2) //Ibat < 1.5A
        {
          if (vbat_now2 > (discharge_voltage_2 - 100)) //Vbat > 2.7V, Ibat < 1.5A
          { 
            duty2--;
            if (duty2 < MINBUCKDUTY)
              duty2 = MINBUCKDUTY;
          }
          else //Vbat <= 2.7V, Ibat < 1.5A
          {
            duty2++;
            if (duty2 > MAXBUCKDUTY)
              duty2 = MAXBUCKDUTY;
          }
        }
        else //Ibat >= 1.5A
        {
          duty2++;
          if (duty2 > MAXBUCKDUTY)
            duty2 = MAXBUCKDUTY;
        }
        pwmWrite(OC2PF, duty2);
        //pwmWrite(OC2PF, duty1);
      }
    #endif
    #ifdef HW_2_0
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
      pwmWrite(OC2NF2, duty2);
    #endif
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
      #ifdef HW_2_4
        if(psu_dir_1 < 2)
        {
          setChg1(DISCHARGE);
        }
        #ifdef REGEN_ENABLED
        else
        {
          setChg1(CHARGE);
        }
        #endif
      #endif
      #ifdef HW_2_0
        setChg1(DISCHARGE);
      #endif
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
        setLED1(LED_RED);
        state1 = 8;
        settle1 = 0;
        mode1 = 5;
        //Timer1.pause();
        Serial.println("> Cell 1 OVT, stopping");
        Serial.print("> Cell 1 Temp: ");
        Serial.println(temp1);
      }
      if(((int)vbat_1_1 > OVV_THRESH) && (state1 != 8))
      {
        setChg1(DISCONNECT);
        setLED1(LED_RED);
        state1 = 8;
        settle1 = 0;
        mode1 = 5;
        //Timer1.pause();
        Serial.println("> Cell 1 OVV, stopping");
        Serial.print("> Cell 1 Voltage: ");
        Serial.println(vbat_1_1);
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
        //vbuf_i = (int)(((float)getAuxADC(BUFV)) * BUF2V);
        //Serial.print(vbuf_i); //2.417);
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
            setLED1(LED_PURPLE);
          }
          else if ((ibat_1_1 <= ccc_1) && (mode1 == 3) && (settle1 > 10)) //Full battery = <50mA CC
          {
            state1 = 2;
            settle1 = 0;
            setLED1(LED_PURPLE);
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
              setLED1(LED_OFF);
            }
            else
            {
              state1 = 3;
              settle1 = 0;
              setChg1(CHARGE);
              setLED1(LED_CYAN);
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
            cell1_vmax = 0;
          }
          break;
        case 4: //Charging state (CC/CV)
          //Serial.println("Case 4");
          ibati1 = (int)(ibat_1_1 * -1.0);
          vbati1 = (int)(vbat_1_1);
          if(vbati1 > cell1_vmax)
            cell1_vmax = vbati1;
          if(cell1_type == 0)
          {
            if ((ibati1 <= ccc_1) && (mode1 != 3)) //Full battery = <50mA CC
            {
              setChg1(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
              state1 = 5;
              settle1 = 0;
              setLED1(LED_PURPLE);
            }
          }
          else
          {
            if (((vbati1 <= (cell1_vmax - NIMH_DV)) && settle1 > 600) || (vbati1 >= 1800)) //Full battery = -10mV dV or >1.8V, 10 minute delay until detection
            {
              setChg1(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
              state1 = 5;
              settle1 = 0;
              setLED1(LED_PURPLE);
            }
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
              setLED1(LED_OFF);
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
                setLED1(LED_OFF);
              }
              else
              {
                #ifdef HW_2_4
                  if(psu_dir_1 == 2)
                  {
                    state1 = 1;
                    settle1 = 0;
                    #ifdef REGEN_ENABLED
                      setChg1(CHARGE);
                    #endif
                    setLED1(LED_YELLOW);
                  }
                  else
                  {
                    state1 = 1;
                    settle1 = 0;
                    setLED1(LED_YELLOW);
                    setChg1(DISCHARGE);
                  }
                #endif
                #ifdef HW_2_0
                  state1 = 1;
                  settle1 = 0;
                  setLED1(LED_YELLOW);
                  setChg1(DISCHARGE);
                #endif
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
      #ifdef HW_2_4
        if(psu_dir_2 < 2)
        {
          setChg2(DISCHARGE);
        }
        #ifdef REGEN_ENABLED
          else
          {
            setChg2(CHARGE);
          }
        #endif
      #endif
      #ifdef HW_2_0
        setChg2(DISCHARGE);
      #endif
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
        setLED2(LED_RED);
        state2 = 8;
        settle2 = 0;
        mode2 = 5;
        //Timer1.pause();
        Serial.println("> Cell 2 OVT, stopping");
        Serial.print("> Cell 2 Temp: ");
        Serial.println(temp2);
      }
      if(((int)vbat_1_2 > OVV_THRESH) && (state2 != 8))
      {
        setChg2(DISCONNECT);
        setLED2(LED_RED);
        state2 = 8;
        settle2 = 0;
        mode2 = 5;
        //Timer1.pause();
        Serial.println("> Cell 2 OVV, stopping");
        Serial.print("> Cell 2 Voltage: ");
        Serial.println(vbat_1_2);
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
            setLED2(LED_PURPLE);
          }
          else if ((ibat_1_2 <= ccc_2) && (mode2 == 3) && (settle2 > 10)) //Full battery = <50mA CC
          {
            state2 = 2;
            settle2 = 0;
            setLED2(LED_PURPLE);
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
              setLED2(LED_OFF);
            }
            else
            {
              state2 = 3;
              settle2 = 0;
              setChg2(CHARGE);
              setLED2(LED_CYAN);
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
          vbati2 = (int)(vbat_1_2);
          if(vbati2 > cell2_vmax)
            cell2_vmax = vbati2;
          if(cell2_type == 0)
          {
            if ((ibati2 <= ccc_2) && (mode2 != 3)) //Full battery = <50mA CC
            {
              setChg2(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
              state2 = 5;
              settle2 = 0;
              setLED2(LED_PURPLE);
            }
          }
          else
          {
            if (((vbati2 <= (cell2_vmax - NIMH_DV)) && settle2 > 600) || (vbati2 >= 1800)) //Full battery = -10mV dV or >1.8V, 10 minute delay until detection
            {
              setChg2(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
              state2 = 5;
              settle2 = 0;
              setLED2(LED_PURPLE);
            }
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
              setLED2(LED_OFF);
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
                setLED2(LED_OFF);
              }
              else
              {
                #ifdef HW_2_4
                  if(psu_dir_2 == 2)
                  {
                    state2 = 1;
                    settle2 = 0;
                    #ifdef REGEN_ENABLED
                      setChg2(CHARGE);
                    #endif
                    setLED2(LED_YELLOW);
                  }
                  else
                  {
                    state2 = 1;
                    settle2 = 0;
                    setLED2(LED_YELLOW);
                    setChg2(DISCHARGE);
                  }
                #endif
                #ifdef HW_2_0
                  state2 = 1;
                  settle2 = 0;
                  setLED2(LED_YELLOW);
                  setChg2(DISCHARGE);
                #endif
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
        if(vbuf_i > MAX_VBUF)
        {
          if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
          {
            if(psu_dir_1 >= 2)
            {
              setChg1(DISCONNECT);
              setLED1(LED_RED);
              state1 = 8;
              settle1 = 0;
              mode1 = 5;
              Serial.println("> Vbuf OV, stop C1 regen");
            }
          }
          if ((state2 == 1) || (state2 == 6) || (state2 == 7)) //Discharging states
          {
            if(psu_dir_2 >= 2)
            {
              setChg2(DISCONNECT);
              setLED2(LED_RED);
              state2 = 8;
              settle2 = 0;
              mode2 = 5;
              Serial.println("> Vbuf OV, stop C2 regen");
            }
          }
        }
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
        //Serial.println(duty1);//2.417);
        //Serial.print(",");
        //Serial.println(duty2);//2.417);
        //Serial.println(",Duty2");
      }
      /*if((state1 == 1) || (state2 == 1) || (state1 == 3) || (state2 == 3) || (state1 == 4) || (state2 == 4))
      {
        pinMode(FANON, OUTPUT);
        digitalWrite(FANON, HIGH); //Non-inverting mode
      }
      else
      {
        pinMode(FANON, OUTPUT);
        digitalWrite(FANON, LOW); //Non-inverting mode
      }*/
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
  uint16 cellV = 4200;
  char *args[8];
  recvWithStartEndMarkers();
  if (newData == true)
  {
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
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC1V)) / ADC2V1) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 1, cancelling charge");
              break;
            }
            mode1 = 0;
            parseCharge1(i, args);
            if(charge_voltage_1 < cellV)
            {
              Serial.print("> Cell V < charge set., increasing to ");
              charge_voltage_1 = cellV + 100;
              if(charge_voltage_1 > MAX_CHG_VOL)
                charge_voltage_1 = MAX_CHG_VOL;
              Serial.print(charge_voltage_1);
              Serial.println("mV");
            }
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(LED_CYAN);
            printMenu(mode1, Serial);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC2V)) / ADC2V2) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 2, cancelling charge");
              break;
            }
            mode2 = 0;
            parseCharge2(i, args);
            if(charge_voltage_2 < cellV)
            {
              Serial.print("> Cell V < charge set., increasing to ");
              charge_voltage_2 = cellV + 100;
              if(charge_voltage_2 > MAX_CHG_VOL)
                charge_voltage_2 = MAX_CHG_VOL;
              Serial.print(charge_voltage_2);
              Serial.println("mV");
            }
            state2 = 3;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(CHARGE);
            setLED2(LED_CYAN);
            printMenu(mode2, Serial);
          }
        }
        break;
      case 'd':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
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
            #ifdef HW_2_4
              if(psu_dir_1 == 0) //Resistive Mode
              {
                setChg1(DISCHARGE);
              }
              #ifdef REGEN_ENABLED
                else if(psu_dir_1 == 2) //Regenerative Mode
                {
                  setChg1(CHARGE);
                }
              #endif
            #endif
            #ifdef HW_2_0
                setChg1(DISCHARGE);
            #endif
            setLED1(LED_YELLOW);
            printMenu(mode1, Serial);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
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
            #ifdef HW_2_4
              if(psu_dir_2 == 0) //Resistive Mode
              {
                setChg2(DISCHARGE);
              }
              #ifdef REGEN_ENABLED
                else if(psu_dir_2 == 2) //Regenerative Mode
                {
                  setChg2(CHARGE);
                }
              #endif
            #endif
            #ifdef HW_2_0
                setChg2(DISCHARGE);
            #endif
            setLED2(LED_YELLOW);
            printMenu(mode2, Serial);
          }
        }
        break;
      case 'r':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            mode1 = 6;
            parseIR1(i, args);
            state1 = 6;
            settle1 = 0;
            ir1 = 0;
            //Timer2.resume(); //Start the timer counting
            setLED1(LED_PURPLE);
            printMenu(mode1, Serial);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            mode2 = 6;
            parseIR2(i, args);
            state2 = 6;
            settle2 = 0;
            ir2 = 0;
            //Timer2.resume(); //Start the timer counting
            setLED2(LED_PURPLE);
            printMenu(mode2, Serial);
          }
        }
        break;
      case 'y':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC1V)) / ADC2V1) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 1, cancelling charge");
              break;
            }
            mode1 = 2;
            parseCycle1(i, args);
            if(charge_voltage_1 < cellV)
            {
              Serial.print("> Cell V < charge set., increasing to ");
              charge_voltage_1 = cellV + 100;
              if(charge_voltage_1 > MAX_CHG_VOL)
                charge_voltage_1 = MAX_CHG_VOL;
              Serial.print(charge_voltage_1);
              Serial.println("mV");
            }
            cycle_count_1 = 0;
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(LED_CYAN);
            printMenu(mode1, Serial);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC2V)) / ADC2V2) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 2, cancelling charge");
              break;
            }
            mode2 = 2;
            parseCycle2(i, args);
            if(charge_voltage_2 < cellV)
            {
              Serial.print("> Cell V < charge set., increasing to ");
              charge_voltage_2 = cellV + 100;
              if(charge_voltage_2 > MAX_CHG_VOL)
                charge_voltage_2 = MAX_CHG_VOL;
              Serial.print(charge_voltage_2);
              Serial.println("mV");
            }
            cycle_count_2 = 0;
            state2 = 3;
            settle2 = 0;
            mah2 = 0;
            mwh2 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg2(CHARGE);
            setLED2(LED_CYAN);
            printMenu(mode2, Serial);
          }
        }
        break;
      case 'p':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC1V)) / ADC2V1) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 1, cancelling PSU");
              break;
            }
            mode1 = 3;
            parsePSU1(i, args);
            if(charge_voltage_1 < cellV)
            {
              Serial.print("> Cell V < PSU set., increasing to ");
              charge_voltage_1 = cellV + 100;
              if(charge_voltage_1 > MAX_CHG_VOL)
                charge_voltage_1 = MAX_CHG_VOL;
              Serial.print(charge_voltage_1);
              Serial.println("mV");
            }
            if(psu_dir_1 == 1) //Buck aka charge mode
            {
              state1 = 3;
              settle1 = 0;
              mah1 = 0;
              mwh1 = 0;
              //Timer2.resume(); //Start the timer counting
              setChg1(CHARGE);
              setLED1(LED_CYAN);
            }
            else if(psu_dir_1 == 0)
            {
              state1 = 1;
              settle1 = 0;
              mah1 = 0;
              mwh1 = 0;
              //Timer2.resume(); //Start the timer counting
              setChg1(DISCHARGE);
              setLED1(LED_YELLOW);
            }
            #ifdef REGEN_ENABLED
              else if(psu_dir_1 == 2)
              {
                mode1 = 1;
                state1 = 1;
                settle1 = 0;
                mah1 = 0;
                mwh1 = 0;
                //Timer2.resume(); //Start the timer counting
                setChg1(CHARGE);
                setLED1(LED_YELLOW);
              }
            #endif
            printMenu(mode1, Serial);
          }
        }
        else if(args[0][1] == '2')
        {
          if(getCell2RV() > 100)
          {
            setLED2(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 2 voltage: ");
            Serial.println(getCell2RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC2V)) / ADC2V2) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 2, cancelling PSU");
              break;
            }
            mode2 = 3;
            parsePSU2(i, args);
            if(charge_voltage_2 < cellV)
            {
              Serial.print("> Cell V < PSU set., increasing to ");
              charge_voltage_2 = cellV + 100;
              if(charge_voltage_2 > MAX_CHG_VOL)
                charge_voltage_2 = MAX_CHG_VOL;
              Serial.print(charge_voltage_2);
              Serial.println("mV");
            }
            if(psu_dir_2 == 1) //Buck aka charge mode
            {
              state2 = 3;
              settle2 = 0;
              mah2 = 0;
              mwh2 = 0;
              //Timer2.resume(); //Start the timer counting
              setChg2(CHARGE);
              setLED2(LED_CYAN);
            }
            else if(psu_dir_2 == 0)
            {
              state2 = 1;
              settle2 = 0;
              mah2 = 0;
              mwh2 = 0;
              //Timer2.resume(); //Start the timer counting
              setChg2(DISCHARGE);
              setLED2(LED_YELLOW);
            }
            #ifdef REGEN_ENABLED
              else if(psu_dir_2 == 2)
              {
                mode2 = 1;
                state2 = 1;
                settle2 = 0;
                mah2 = 0;
                mwh2 = 0;
                //Timer2.resume(); //Start the timer counting
                setChg2(CHARGE);
                setLED2(LED_YELLOW);
              }
            #endif
            printMenu(mode2, Serial);
          }
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
        /*
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
        setLED1(LED_CYAN);
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
        setLED2(LED_CYAN);
        Serial.print("\r\n");
        Serial.println("> Cal Mode");
        Serial.println("> Slot 1/2 corr. reset to default");
        Serial.print(">  ADC corr. factor: ");
        Serial.print(corr_factor*100);
        Serial.println(" Slot 1+2 set to 4.20V, 1.0A");
        Serial.println(">  Press n1, n2 to end");
        Serial.print("\r\n");
        Serial.print("> ");
        */
        break;
      case 'n':
        if(args[0][1] == '1')
        {
          mode1 = 6;
          state1 = 8;
          setChg1(DISCONNECT);
          //Timer2.pause(); //Start the timer counting
          setLED1(LED_OFF);
          Serial.println("\r\n");
          //printMenu(mode1);
        }
        else if(args[0][1] == '2')
        {
          mode2 = 6;
          state2 = 8;
          setChg2(DISCONNECT);
          //Timer2.pause(); //Start the timer counting
          setLED2(LED_OFF);
          Serial.println("\r\n");
          //printMenu(mode2);
        }
        break;
      case 'v':        
        Serial.print("\r\n> Software version: ");
        Serial.println(vers);
        Serial.print("\r\n> ");
        break;
      case 'q':
        /*if(args[0][1] == '1')
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
        }*/
        break;
      case 's':
        Serial.println("\r\n");
        if((state1 == 8) || (state2 == 8))
        {
          //adciref = analogRead(AIREF); //Iref voltage
          vbuf_i = (int)(((float)getAuxADC(BUFV)) * BUF2V);
        }
        //Msg type 4 (Debug):
        //(Debug,4,Vbuf,mV,Vrev,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
        Serial.print("4,");
        Serial.print(vbuf_i); //2.417);
        Serial.print(",");
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
        break;
      #ifdef OLED_ENABLED
        case 'a':
          if(displayEnabled == 0)
          {
            displayEnabled = 1;
            Serial.println("\r\n> Disp Enable\r\n>");
          }
          else
          {
            displayEnabled = 0;
            Serial.println("\r\n> Disp Disable\r\n>");
          }
          break;
      #endif
      case '?':
        mode1 = 99;
        printMenu(mode1, Serial);
        //Timer2.pause(); //Start the timer counting
        break;
      case 'z':
        //Soft reset
        nvic_sys_reset();
        break;
      default:
        mode1 = 99;
        printMenu(mode1, Serial);
        //Timer2.pause(); //Start the timer counting
        break;
    }

    newData = false;
  }
  //Display thread; update 1 Hz
  if((tick - last_tick) >= 2000)
  {
    last_tick = tick;

    if(displayEnabled)
    {
      #ifdef OLED_ENABLED
      display.clearLine1();
      display.setCursor(0,0);
      display.print((int)vbat_1_1);
      display.setCursor(30,0);
      display.print((int)vbat_1_2);
      display.displayLine1();
      #endif
    }
    
    if((getChgPwr()+getDisPwr()) < 4500)
    {
      fanSpeed = 1;
    }
    else
    {
      fanSpeed = 2;
    }
  }
  if((state1 == 1) || (state2 == 1) || (state1 == 3) || (state2 == 3) || (state1 == 4) || (state2 == 4)) //Fan on
  {
    if(fanSpeed == 1) //Fan PWM
    {
      if((tick - last_tick_fan) >= 16)
      {
        last_tick_fan = tick;
        if(fantoggle) //50% duty, 62.5Hz
        {
          fantoggle = !fantoggle;
          digitalWrite(FANON, fantoggle); //Non-inverting mode
        }
        else
        {
          fantoggle = !fantoggle;
          digitalWrite(FANON, fantoggle); //Non-inverting mode
        }
      }
    }
    else //Fan full on
    {
      digitalWrite(FANON, HIGH); //Non-inverting mode
    }
  }
  else //Fan off
  {
    digitalWrite(FANON, LOW); //Non-inverting mode
  }
  //delay(100);
  /*loopcnt++;
  if(loopcnt > 9)
    loopcnt = 0;*/
  if (interruptCounter > 8) {
    setChg1(DISCONNECT);
    setChg2(DISCONNECT);
    mode1 = 5;
    state1 = 8;
    mode2 = 5;
    state2 = 8;
    setLED1(LED_RED);
    setLED2(LED_RED);
    Serial.println("> Int Ovf Error, Stopping!");
    Serial.print("> Ints Pending: ");
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
      digitalWrite(C1ON, HIGH); //synchronous buck started, reconnect cell 
  }
  if (slot2_startup > 0)
  {
    slot2_startup--;
    if(slot2_startup == 0)
      digitalWrite(C2ON, HIGH); //synchronous buck started, reconnect cell 
  }
}

//2kHz interrupt
void handler_loop(void) {
  interruptCounter++;
  tick++;
}

