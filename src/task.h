#ifndef TASK_H
#define TASK_H

#include "arduino.h"
#include <stdint.h>
#include "soc/rtc_wdt.h"
#include <SPI.h>
#include "kalman_filter.h"
#include <U8g2lib.h>
#include <OneButton.h>
#include <phyphoxBle.h>

#include "bitmap.h"
#include "rotary.h"
#include "beep.h"
#include "externdraw.h"
#include "menu.h"
#include "AD7606C.h"

#define PINNED_TO_CORE1 0
#define PINNED_TO_CORE2 1

/*
 * hardware I/O pins
 */
#define ROTARY_A 25
#define ROTARY_B 32
#define ROTARY_P 33
#define ROTARY_GND 35

#define BEEP_PIN 27
#define DAC_CH2 26
#define SIN_TB_SIZE 256
// #define SPI2_SCLK 14
// #define SPI2_MISO 12 //DOUTA
// #define SPI2_MOSI 13 //SDI
// #define SPI2_CS   15

#define ADC_CONVST 2 //
#define ADC_BUSY 4   //

/*
 * oled define
 */
#define SCREEN_COLUMN 128
#define SCREEN_ROW 64
#define SCREEN_PAGE_NUM 8
#define SCREEN_FONT_ROW 4

extern float adc_disp_val[ADC_ALL_CH];
extern spinlock_t task_lock;
extern uint8_t rotary_dir;
extern uint8_t volume;
extern uint8_t sin_tab[SIN_TB_SIZE];
extern char ChipMAC_S[19];

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled;
extern KalmanFilter kf_main_ui;

extern void hardware_init(void);
extern void ble_phyphox_init();
extern void wifi_client_init();
extern void xTask_wifi(void *xTask);
extern void xTask_oled(void *xTask);
extern void xTask_dbgx(void *xTask);
extern void xTask_adcx(void *xTask);
extern void xTask_rotK(void *xTask);
extern void xTask_blex(void *xTask);

extern void get_sin(void);
#endif