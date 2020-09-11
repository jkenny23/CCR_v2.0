// serial_support.cpp

#include "serial_support.h"

int fast_atoi_leading_pos(const char * p) {
    int x = 0;
    ++p;
    while (*p >= '0' && *p <= '9') {
        x = (x*10) + (*p - '0');
        ++p;
    }
    return x;
}

int fast_atoi_leading_neg(const char * p) {
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

void printMenu(unsigned char menu2, USBSerial &serRef) {
  switch (menu2) {
    case 0:
      serRef.print("\r\n");
      serRef.println("> Charging");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    case 1:
      serRef.print("\r\n");
      serRef.println("> Discharging");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    case 2:
      serRef.print("\r\n");
      serRef.println("> Cycle Testing");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    case 3:
      serRef.print("\r\n");
      serRef.println("> PSU Mode");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    case 4:
      serRef.print("\r\n");
      serRef.println("> Test Mode");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    case 6:
      serRef.print("\r\n");
      serRef.println("> IR Test");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    case 7:
      serRef.print("\r\n");
      serRef.println("> Waiting");
      serRef.println(">  Press n to end");
      serRef.print("\r\n");
      break;
    default:
      //'c', 'd', 'y', 'p', 't', 'n', '?', 'v', 's', 'l', 'r', 'z', 'q', 'a'
      serRef.print("\r\n");
      serRef.println("> Select Mode:");
      serRef.println(">  Charge");
      serRef.println(">   c[1-2] i[charge current, mA] v[charge voltage, mV] o[cutoff current, mA] n[cell type: 0 = Li, 1 = Ni]");
      serRef.println(">            100-1500, def.1500    2400-4500, def.4200   5-1000, def.50        def.0");
      serRef.println(">          e[timeout, s]");
      serRef.println(">            1-65535, def.0");
      serRef.println(">  Discharge");
      serRef.println(">   d[1-2] i[dis current, mA] v[cutoff voltage, mV] m[mode: 0 = constant, 1 = taper]");
      serRef.println(">            100-1500, def.1500 700-3900, def.2700    def.0");
      serRef.println(">          r[dir: 0 = resistive, 2 = regen] o[cutoff current, mA]");
      serRef.println(">            def.0                            5-1000, def.400");
      serRef.println(">  Cycle");
      serRef.println(">   y[1-2] i[dis current, mA] v[cutoff voltage, mV] m[mode: 0 = constant, 1 = taper]");
      serRef.println(">            100-1500, def.1500 700-3900, def.2700    def.0");
      serRef.println(">          k[charge current, mA] u[charge voltage, mV] o[chg. cutoff current, mA] l[number of cycles]");
      serRef.println(">            100-1500, def.1500    2400-4500, def.4200   50-250, def.50             def.1");
      serRef.println(">          r[dir: 0 = resistive, 2 = regen] n[cell type: 0 = Li, 1 = Ni] e[timeout, s]");
      serRef.println(">            def.0                            def.0                      1-65535, def.0");
      serRef.println(">          q[disc. cutoff current, mA]");
      serRef.println(">            5-2000, def.400");
      serRef.println(">  Power Supply");
      serRef.println(">   p[1-2] r[dir: 0 = resistive, 1 = buck, 2 = regen] v[voltage setting, mV] i[current limit, mA]");
      serRef.println(">            def.1                                      0-9500, def.4200       50-1500, def.1500");
      serRef.println(">          o[cutoff current, mA]");
      serRef.println(">           50-250, def.50");
      serRef.println(">  Cal R/W Mode");
      serRef.println(">   t[r/w] a[address: 0-999] d[data, unsigned int (0-65535)]");
      serRef.println(">  IR Test Mode");
      serRef.println(">   r[1-2] i[test current, mA] r[direction: 0 = resistive, 2 = regen]");
      serRef.println(">            100-1500, def.1500  def.0");
      serRef.println(">  Wait");
      serRef.println(">   w[1-2] i[time, s]");
      serRef.println(">            1-32767, def.30");
      serRef.println(">  Test Mode (raw PWM duty)");
      serRef.println(">   q[1-2] l[duty cycle (0-399)] r[direction: 0 = resistive, 2 = regen]");
      serRef.println(">            0-399                 def.0");
      serRef.println(">  Help (Prints this menu)");
      serRef.println(">   ?");
      serRef.println(">  Stop current mode/test");
      serRef.println(">   n[1-2]");
      serRef.println(">  Version");
      serRef.println(">   v");
      serRef.println(">  Soft Reset");
      serRef.println(">   z");
      serRef.println(">  OLED En/Disable (Toggle)");
      serRef.println(">   a");
      serRef.println(">  Status");
      serRef.println(">   s");
      serRef.print("\r\n");
      serRef.print("> ");
      break;
  }
}
