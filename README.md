# CCR_v2.0

This is the main unified code repository for the CCR 18650 charger/tester. More details on the project and hardware can be found here: http://rev0.net/index.php?title=CCR

This code is tested for the version 2.4 hardware, but backwards compatible with older hardware, selectable with the following defines:
```c
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
//#define DEBUG_MODE
#define REGEN_ENABLED
//#define MON_SHUNT
//#define V_1S //220k/150k 0-4.84V
//#define V_2S //430k/150k 0-9.46V, 2s charging allowed
#define V_2S1 //430k/150k 0-9.46V, only 1s charging allowed
//#define LP //Low power, 4A limit, 1.65A shunt, 90kHz Fsw
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
