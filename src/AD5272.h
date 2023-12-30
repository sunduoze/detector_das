/*
 * AD5272.h - Arduino for I2C communication with AD5272
 *
 * sunduoze modify from Jirapong Manit
 *  2023/12/30
 */
#ifndef _AD5272_H
#define _AD5272_H

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

#define AD5272_RES_VAL 2e4 // 20k Ohms

#define POT_ADDR_NC 0x2E
#define POT_ADDR_GND 0x2F
#define POT_ADDR_VDD 0x2C

// write the 10 or 8 data bits to the RDAC wiper register (it must be unlocked first)
#define AD5272_RDAC_WRITE 0x01
#define AD5272_RDAC_READ 0x02  // read the RDAC wiper register
#define AD5272_50TP_WRITE 0x03 // Store RDAC setting to 50-TP

// Write contents of the serial register data to the control register
#define AD5272_CONTROL_WRITE 0x07

/**
 * Control bits are three bits written with command 7
 */
// enable writing to the 50-TP memory by setting this control bit C0
// default is cleared so 50-TP writing is disabled
// only 50 total writes are possible!
#define AD5272_50TP_WRITE_ENABLE 0x01

// enable writing to volatile RADC wiper by setting this control bit C1
// otherwise it is frozen to the value in the 50-TP memory
// default is cleared, can't write to the wiper
#define AD5272_RDAC_WIPER_WRITE_ENABLE 0x02

// enable high precision calibration by clearing this control bit C2
// set this bit to disable high accuracy mode (dunno why you would want to)
// default is 0 = enabled
#define AD5272_RDAC_CALIB_DISABLE 0x04

// 50TP memory has been successfully programmed if this bit is set
#define AD5272_50TP_WRITE_SUCCESS 0x08

class AD5272
{
public:
  AD5272(int addr);

  int init(void);
  int write_data(uint16_t cmd, uint16_t data);
  int read_rdac(void);
  uint32_t set_res_val(uint32_t resistor);

  inline int get_addr(void) { return _addr; }
  inline void set_addr(int addr) { _addr = addr; }

private:
  int _addr;
};

#endif