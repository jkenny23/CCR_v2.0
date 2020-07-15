// serial_support.h

#ifndef SERIAL_SUPPORT_H
#define SERIAL_SUPPORT_H

#include "Arduino.h"

int fast_atoi_leading_pos(const char * p);
int fast_atoi_leading_neg(const char * p);
void printMenu(unsigned char menu2, USBSerial &serRef);

#endif
