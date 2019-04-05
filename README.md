# CCR_v2.0

This is the main code repository for the CCR v2.0 18650 charger/tester. More details on the project and hardware can be found here: http://rev0.net/index.php?title=CCR

This code is intended for the version 2.0 hardware, with the following hardware defines:
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
