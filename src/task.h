#ifndef TASK_H
#define TASK_H

#include "arduino.h"
#include <stdint.h>
#include "soc/rtc_wdt.h"
#include <SPI.h>
#include "kalman_filter.h"
#include <U8g2lib.h>
#include <OneButton.h>

#define PINNED_TO_CORE1 0
#define PINNED_TO_CORE2 1

#define ROTARY_A 25
#define ROTARY_B 32
#define ROTARY_P 33
#define ROTARY_GND 35

#define BEEP_PIN 27

// #define SPI2_SCLK 14
// #define SPI2_MISO 12 //DOUTA
// #define SPI2_MOSI 13 //SDI
// #define SPI2_CS   15

#define ADC_CONVST 2 //
#define ADC_BUSY 4   //

extern spinlock_t task_lock;
extern uint8_t rotary_dir;
extern uint8_t volume;
extern void hardware_init(void);
extern void xTask_wifi(void *xTask);
extern void xTask_oled(void *xTask);
extern void xTask_dbgx(void *xTask);
extern void xTask_adcx(void *xTask);
extern void xTask_rotK(void *xTask);
#endif