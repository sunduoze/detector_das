#ifndef AD7606C_H
#define AD7606C_H

#include "arduino.h"
#include <SPI.h>

#define ADC_ALL_CH 8

#define PN20V0 0.00015258f
#define PN12V5 0.00009536f
#define PN10V0 0.00007629f
#define PN6V25 0.00004768f
#define PN5V00 0.00003815f
#define PN2V50 0.00001907f

#define PP12V5 0.00004768f
#define PP10V0 0.00003815f
#define PP5V00 0.00001907f
/*code to voltage*/
#define C2V(code, range_lsb) (code) * range_lsb

enum
{
	ADC_CH1 = 0x0,
	ADC_CH2 = 0x1,
	ADC_CH3 = 0x2,
	ADC_CH4 = 0x3,
	ADC_CH5 = 0x4,
	ADC_CH6 = 0x5,
	ADC_CH7 = 0x6,
	ADC_CH8 = 0x7
};

enum
{
	BIPOLAR_MODE = 0,
	UNIPOLAR_MODE = 1
};

class AD7606C
{
protected:
	uint8_t _RESET;
	uint8_t _CS;
	uint8_t _CONVST;
	uint8_t _FRSTDATA;
	uint8_t _BUSY;
	uint8_t _OS0;
	uint8_t _OS1;
	uint8_t _OS2;
	uint8_t _RANGE;
	uint8_t data_inc_status;
	void reset();
	void ipulse(uint8_t pin);

public:
	void setOversampling(uint8_t);
	void setRange(bool);
	// uint16_t debug_val;
};

class AD7606C_Serial : public AD7606C
{
protected:
#ifdef ALTERNATE_PINS
#define VSPI_MISO 2
#define VSPI_MOSI 4
#define VSPI_SCLK 0
#define VSPI_SS 33

#define HSPI_MISO 26
#define HSPI_MOSI 27
#define HSPI_SCLK 25
#define HSPI_SS 32
#else
#define VSPI_MISO MISO
#define VSPI_MOSI MOSI
#define VSPI_SCLK SCK
#define VSPI_SS SS

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS 15
#endif

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

	SPIClass *hspi = NULL;
	uint8_t channel_mode[ADC_ALL_CH];

public:
	AD7606C_Serial(int CONVST, int BUSY);
	// AD7606C_Serial(int FRSTDATA, int BUSY, int RESET);
	// AD7606C_Serial(int FRSTDATA, int BUSY, int RESET, int OS0, int OS1, int OS2, int RANGE);
	uint8_t read_reg(uint8_t, uint8_t *);
	uint8_t write_reg(uint8_t reg_addr, uint8_t reg_val);
	uint8_t write_and_chk_reg(uint8_t reg_addr, uint8_t reg_val);

	void convert_18bit_to_32bit(int32_t *unsigned_val, int32_t srcsz, int32_t *pdst);

	uint8_t data_read(int32_t *data);
	int32_t read(int32_t *data);	  // Read raw values from ADC
	int32_t fast_read(int32_t *data); // fast read last conv raw values from ADC

	void config(void);
	void debug(void);
	uint8_t check_id(void);
	void get_all_reg_val(void);

	void test_reg_rw(void);
};

#endif