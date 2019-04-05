# CCR_v2.0

This is the main code repository for the CCR v2.0 18650 charger/tester. More details on the project and hardware can be found here: http://rev0.net/index.php?title=CCR

This code is intended for the version 2.0 hardware, with the following hardware defines:
```c
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
#define AUXSEL2 PB5 //Lo = HS thermistor #2, Hi = HS thermistor #1 or ABUFV
#define AUXSEL1 PB2 //Lo = ABUFV, Hi = HS thermistor #1
#define HSTH1 0
#define HSTH2 1
#define BUFV 2
#define AIREF PA5 //Reference voltage input
//Pin definitions - Digital outputs
#define OC1NF1 PA8 //Boost NFET output
#define OC1PF PA9 //Buck PFET output
#define OC2NF1 PA10 //Boost NFET output
#define OC2PF PB8 //Buck PFET output
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
```

Library dependencies:
Adafruit_NeoPixel-ANDnXOR.h - modified for inverted logic, since CCR v2.x has an N-FET to level shift the MCU 3.3V to 5V logic for the WS2812B.

Usage:

The cycler has 5 modes for charging/discharging/testing batteries, as follows:

Charge: Command "c", arguments: cell number, charge current, charge voltage, cutoff current.
```
c[1-2] i[charge current, mA] v[charge voltage, mV] o[cutoff current, mA]
         100-1500, def.1500    2400-4500, def.4200   50-250, def.50
```
Discharge: Command "d", arguments: cell number, discharge current, cutoff voltage, mode (not yet implemented), cutoff current (not yet implemented)
```
d[1-2] i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
         100-1500, def.1500       1000-3900, def.2700   def.0
```
Cycle Test: Command "y", arguments: cell number, discharge current, discharge cutoff voltage, mode (not yet implemented), discharge cutoff current (not yet implemented), charge current, charge voltage, charge cutoff current, number of cycles
```
y[1-2] i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
         100-1500, def.1500       1000-3900, def.2700   def.0
       k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]
         100-1500, def.1500    2400-4500, def.4200   50-250, def.50        def.1
```
Internal Resistance Test: Command "r", arguments: cell number, test current
```
r[1-2] i[test current, mA]
         100-1500, def.1500
```
Power Supply Mode: Command "p", arguments: slot number, voltage setting, current limit, cutoff current, direction (output/charge or input/discharge)
```
p[1-2] r[direction: 0 = boost, 1 = buck] v[voltage setting, mV] i[current limit, mA]
         def.1                             0-9500, def.4200       50-1500, def.1500
       o[cutoff current, mA]
         50-250, def.50
```
Calibration: Command "l", arguments: none

Other commands: "?" - Help, prints menu, "v" - Prints software version, "s" - Prints status in the following format:
```
4,12162,3459,33,2039,28.54,-22.20
9,0,-19,-0.09,-0.00,28.05,0.00
10,595,6,-0.00,-0.00,29.86,0.00
```
Message 4, Buffer/input voltage (mV), Slot 1 reverse voltage (mV), Slot 2 reverse voltage (mV), ADC reference value (ADC counts), Heatsink 1 temp (C), Heatsink 2 temp (C)

Message 9, Slot 1 voltage (mV), Slot 1 current (mA), Slot 1 capacity (mAH), Slot 1 capacity (mWH), Slot 1 temperature (C), Slot 1 IR (mOhm)

Message 10, Slot 2 voltage (mV), Slot 2 current (mA), Slot 2 capacity (mAH), Slot 2 capacity (mWH), Slot 2 temperature (C), Slot 2 IR (mOhm)
