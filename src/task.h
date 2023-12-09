#ifndef TASK_H
#define TASK_H

#include "arduino.h"
#include <SPI.h>
#include "soc/rtc_wdt.h"

extern void hardware_init(void);
extern void xTask_wifi(void *xTask);
extern void xTask_oled(void *xTask);
extern void xTask_dbg(void *xTask);
extern void xTask_adc(void *xTask);

#endif