#pragma once

#define SETUP_OK          0

#define INPUT  0 // is really 0 for control register!
#define OUTPUT 1 // is really 1 for control register!

#define HIGH 1
#define LOW  0

typedef int bool;

int gpio_init(void);
void gpio_close(void);
int gpio_open(int pinno, bool out);
int gpio_getpin(int pin);
int gpio_setpin(int pin, int value);
