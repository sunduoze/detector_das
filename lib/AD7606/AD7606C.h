#ifndef AD7606C_H
#define AD7606C_H

#include "arduino.h"
#include <SPI.h>

#define PN20V0 0.00015258f
#define PN12V5 0.00009536f
#define PN10V0 0.00007629f
#define PN6V25 0.00004768f
#define PN5V00 0.00003815f
#define PN2V50 0.00001907f

#define P12V5 0.00004768f
#define P10V0 0.00003815f
#define P5V00 0.00001907f

/*BIPOLAR_CODE_TO_VOLT*/
#define BC2V(code, range_lsb) (((code - 1) & 0x20000) == 0x20000) ? (-((code)-0x40000) * -range_lsb) : ((code) * range_lsb)
/*UNIPOLAR_CODE_TO_VOLT*/
#define UC2V(code) (code) * range_lsb

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
	// uninitalised pointers to SPI objects
	//  SPIClass * vspi = NULL;
	SPIClass *hspi = NULL;
	// static int32_t cpy26b32b(uint8_t *psrc, uint32_t srcsz, uint32_t *pdst);
	// static int32_t cpy18b32b(uint8_t *psrc, uint32_t srcsz, uint32_t *pdst);

public:
	AD7606C_Serial(int CONVST, int BUSY);
	// AD7606C_Serial(int FRSTDATA, int BUSY, int RESET);
	// AD7606C_Serial(int FRSTDATA, int BUSY, int RESET, int OS0, int OS1, int OS2, int RANGE);
	uint8_t read_reg(uint8_t, uint8_t *);
	uint8_t write_reg(uint8_t reg_addr, uint8_t reg_val);
	uint8_t write_and_chk_reg(uint8_t reg_addr, uint8_t reg_val);
	uint8_t data_read(uint32_t *data);
	int32_t read(uint32_t *data);	   // Read raw values from ADC
	int32_t fast_read(uint32_t *data); // fast read last conv raw values from ADC
	void config(void);
	void debug(void);
	void get_id(void);
	void get_all_reg_val(void);

	void test_reg_rw(void);
};

#endif