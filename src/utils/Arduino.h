#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#include <stdint.h>
#include <math.h>
#include "pico/time.h"
#include "hardware/gpio.h"


typedef int boolean;
#define HIGH 1
#define LOW 0
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define micros time_us_32
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define digitalWrite  gpio_put
#define delayMicroseconds sleep_us
#define pinMode gpio_set_dir
#define OUTPUT GPIO_OUT

#endif // _ARDUINO_H_