#include "AD7606C.h"

#define AD7606_REG_STATUS 0x01
#define AD7606_REG_CONFIG 0x02
#define AD7606_REG_RANGE_CH_ADDR(ch) (0x03 + ((ch) >> 1))
#define AD7606_REG_BANDWIDTH 0x07
#define AD7606_REG_OVERSAMPLING 0x08
#define AD7606_REG_GAIN_CH(ch) (0x09 + (ch))
#define AD7606_REG_OFFSET_CH(ch) (0x11 + (ch))
#define AD7606_REG_PHASE_CH(ch) (0x19 + (ch))
#define AD7606_REG_DIGITAL_DIAG_ENABLE 0x21
#define AD7606_REG_DIGITAL_DIAG_ERR 0x22
#define AD7606_REG_OPEN_DETECT_ENABLE 0x23
#define AD7606_REG_OPEN_DETECTED 0x24
#define AD7606_REG_AIN_OV_UV_DIAG_ENABLE 0x25
#define AD7606_REG_AIN_OV_DIAG_ERROR 0x26
#define AD7606_REG_AIN_UV_DIAG_ERROR 0x27
#define AD7606_REG_DIAGNOSTIC_MUX_CH(ch) (0x28 + ((ch) >> 1))
#define AD7606_REG_OPEN_DETECT_QUEUE 0x2C
#define AD7606_REG_CLK_FS_COUNTER 0x2D
#define AD7606_REG_CLK_OS_COUNTER 0x2E
#define AD7606_REG_ID 0x2F

#define DATA_SIZE 28
// Reset the AD7606
void AD7606C::reset()
{
	digitalWrite(_RESET, 1);
	delayMicroseconds(1);
	digitalWrite(_RESET, 0);
	delayMicroseconds(1);
}

void AD7606C::setOversampling(uint8_t times)
{
	times > 6 ? times = 6 : times;
	pinMode(_OS0, OUTPUT);
	pinMode(_OS1, OUTPUT);
	pinMode(_OS2, OUTPUT);
	digitalWrite(_OS0, bitRead(B001, times));
	digitalWrite(_OS1, bitRead(B010, times));
	digitalWrite(_OS2, bitRead(B100, times));
}

void AD7606C::setRange(bool range)
{
	pinMode(_RANGE, OUTPUT);
	digitalWrite(_RANGE, range);
}
// Send a pulse to 0 or more pin for 1us
void AD7606C::ipulse(uint8_t pin)
{
	digitalWrite(pin, LOW);
	digitalWrite(pin, LOW); /* wait LP_CNV time */
	digitalWrite(pin, HIGH);
	digitalWrite(pin, HIGH);
}

AD7606C_Serial::AD7606C_Serial(int CONVST, int BUSY)
{
	_CONVST = CONVST;
	pinMode(_CONVST, OUTPUT);
	digitalWrite(_CONVST, 0);
	_BUSY = BUSY;
	pinMode(_BUSY, INPUT_PULLUP);

	hspi = new SPIClass(HSPI); // default 12:SPI2_MISO 13:SPI2_MOSI 14:SPI2_SCLK 15:SPI2_CS

#ifndef ALTERNATE_PINS
	// initialise hspi with default pins
	// SCLK = 14, MISO = 12, MOSI = 13, SS = 15
	hspi->begin();
#else
	// alternatively route through GPIO pins
	hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); // SCLK, MISO, MOSI, SS
#endif
	pinMode(hspi->pinSS(), OUTPUT); // VSPI SS
	digitalWrite(hspi->pinSS(), HIGH);

	data_inc_status = 0;
}
// AD7606C_Serial::AD7606C_Serial(int FRSTDATA, int BUSY, int RESET)
// {
// 	_RESET = RESET;
// 	pinMode(_RESET, OUTPUT);
// 	_BUSY = BUSY;
// 	pinMode(_BUSY, OUTPUT);
// 	_FRSTDATA = FRSTDATA;
// 	pinMode(_FRSTDATA, INPUT);

// 	hspi = new SPIClass(HSPI); // default 12:SPI2_MISO 13:SPI2_MOSI 14:SPI2_SCLK 15:SPI2_CS

// #ifndef ALTERNATE_PINS
// 	// initialise hspi with default pins
// 	// SCLK = 14, MISO = 12, MOSI = 13, SS = 15
// 	hspi->begin();
// #else
// 	// alternatively route through GPIO pins
// 	hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); // SCLK, MISO, MOSI, SS
// #endif
// 	pinMode(hspi->pinSS(), OUTPUT); // VSPI SS
// 	digitalWrite(hspi->pinSS(), HIGH);

// 	data_inc_status = 0;
// }
// AD7606C_Serial::AD7606C_Serial(int FRSTDATA, int BUSY, int RESET, int OS0, int OS1, int OS2, int RANGE)
// {
// 	_RESET = RESET;
// 	pinMode(_RESET, OUTPUT);
// 	_BUSY = BUSY;
// 	pinMode(_BUSY, OUTPUT);
// 	_OS0 = OS0;
// 	pinMode(_OS0, OUTPUT);
// 	_OS1 = OS1;
// 	pinMode(_OS1, OUTPUT);
// 	_OS2 = OS2;
// 	pinMode(_OS2, OUTPUT);
// 	_RANGE = RANGE;
// 	pinMode(_RANGE, OUTPUT);
// 	_FRSTDATA = FRSTDATA;
// 	pinMode(_FRSTDATA, INPUT);

// 	hspi = new SPIClass(HSPI); // default 12:SPI2_MISO 13:SPI2_MOSI 14:SPI2_SCLK 15:SPI2_CS

// #ifndef ALTERNATE_PINS
// 	// initialise hspi with default pins
// 	// SCLK = 14, MISO = 12, MOSI = 13, SS = 15
// 	hspi->begin();
// #else
// 	// alternatively route through GPIO pins
// 	hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); // SCLK, MISO, MOSI, SS
// #endif
// 	pinMode(hspi->pinSS(), OUTPUT); // VSPI SS
// 	digitalWrite(hspi->pinSS(), HIGH);

// 	data_inc_status = 0;
// }

#define NO_OS_BIT(x) (1 << (x))
#define AD7606_RD_FLAG_MSK(x) (NO_OS_BIT(6) | ((x) & 0x3F))
#define AD7606_WR_FLAG_MSK(x) ((x) & 0x3F)

/**
 * @brief Read a device register via SPI.
 *
 * This function performs CRC8 computation and checking if enabled in the device.
 *
 * @param reg_addr   - Register address in device memory.
 * @param reg_val    - Pointer to the location where to store the register value.
 *
 * @return ret - return code.
 * @note   ad7606c-18 ds P.47
 */
uint8_t AD7606C_Serial::read_reg(uint8_t reg_addr, uint8_t *reg_val)
{
	static uint16_t data_temp;
	static uint8_t ret;
	// data_temp = (0x2 | (reg_addr & 0x3F) << 2) << 8;// 0x3F is mask 0bxx111111 shit!!!
	data_temp = AD7606_RD_FLAG_MSK(reg_addr) << 8;
	hspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
	digitalWrite(hspi->pinSS(), LOW);  // pull SS slow to prep other end for transfer
	hspi->transfer16(data_temp);	   // 发送第1帧
	digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer
	delayMicroseconds(1);
	digitalWrite(hspi->pinSS(), LOW);			   // pull SS slow to prep other end for transfer
	*reg_val = hspi->transfer16(data_temp) & 0xFF; // 发送第2帧（与第1帧内容相同） 并且取低8位为寄存器内容
	digitalWrite(hspi->pinSS(), HIGH);			   // pull ss high to signify end of data transfer
	hspi->endTransaction();
	return ret;
}

/**
 * @brief Write a device register via SPI.
 *
 * This function performs CRC8 computation and checking if enabled in the device.
 *
 * @param reg_addr   - Register address in device memory.
 * @param reg_data   - Value to write to register.
 *
 * @return ret - return code.
 *         Example: -EIO - SPI communication error.
 *                  -ENOTSUP - Device not in software mode.
 *                  -EBADMSG - CRC computation mismatch.
 *                  0 - No errors encountered.
 */
uint8_t AD7606C_Serial::write_reg(uint8_t reg_addr, uint8_t reg_val)
{
	static uint16_t data_temp;
	static uint8_t ret;
	static uint8_t debug;
	/* Dummy read to place the chip in register mode. */
	read_reg(reg_addr, &debug);
	data_temp = AD7606_WR_FLAG_MSK(reg_addr) << 8 | (reg_val & 0xFF);
	hspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
	digitalWrite(hspi->pinSS(), LOW);			// pull SS slow to prep other end for transfer
	debug = hspi->transfer16(data_temp) & 0xFF; // 发送第1帧
	digitalWrite(hspi->pinSS(), HIGH);			// pull ss high to signify end of data transfer
	hspi->endTransaction();
	return ret;
}

uint8_t AD7606C_Serial::write_and_chk_reg(uint8_t reg_addr, uint8_t reg_val)
{
	static uint16_t data_temp;
	static uint8_t ret;
	static uint8_t debug;
	static uint8_t data_rd;
	/* Dummy read to place the chip in register mode. */
	read_reg(reg_addr, &debug);
	data_temp = AD7606_WR_FLAG_MSK(reg_addr) << 8 | (reg_val & 0xFF);
	hspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
	digitalWrite(hspi->pinSS(), LOW);			// pull SS slow to prep other end for transfer
	debug = hspi->transfer16(data_temp) & 0xFF; // 发送第1帧
	digitalWrite(hspi->pinSS(), HIGH);			// pull ss high to signify end of data transfer
	hspi->endTransaction();
	read_reg(reg_addr, &data_rd);
	if (data_rd != reg_val)
	{
		Serial.printf("[error]reg write fail!\r\n");
		ret = 1;
	}
	return ret;
}

uint8_t AD7606C_Serial::check_id(void)
{
	uint8_t ret = 0;
	uint8_t byte1;

	read_reg(0x2F, &byte1);
	if ((byte1 & 0xF0) == 0x30)
	{
		if ((byte1 & 0x0F) != 0x02)
		{
			Serial.printf("[error]SILICON_REVISION is not 0x02, is:0x%x\r\n", byte1);
			ret = 2;
		}
		else
		{
			// Serial.printf("DEV_ID:D7606C-18 generic. SILICON_REVISION:0x%x\r\n", byte1 & 0x0F);
		}
	}
	else
	{
		ret = 1;
		Serial.printf("[error]DEV_ID is not D7606C-18 generic or AD7606C Init fail! data:0x%x\r\n", byte1);
	}
	return ret;
}

void AD7606C_Serial::get_all_reg_val(void)
{
	uint8_t byte1;

	for (uint8_t i = 0x01; i < 0x30; i++)
	{
		read_reg(i, &byte1);
		Serial.printf("[debug]reg:0x%2x,data:0x%2x\r\n", i, byte1);
	}
}

static const uint16_t tconv_max[] = {
	1,	 /* AD7606_OSR_1 */
	3,	 /* AD7606_OSR_2 */
	5,	 /* AD7606_OSR_4 */
	10,	 /* AD7606_OSR_8 */
	20,	 /* AD7606_OSR_16 */
	41,	 /* AD7606_OSR_32 */
	81,	 /* AD7606_OSR_64 */
	162, /* AD7606_OSR_128 */
	324	 /* AD7606_OSR_256 */
};
/**
 * @enum ad7606_osr
 * @brief Oversampling ratio
 */
enum ad7606_osr
{
	/** Oversample by 1 */
	AD7606_OSR_1,
	/** Oversample by 2 */
	AD7606_OSR_2,
	/** Oversample by 4 */
	AD7606_OSR_4,
	/** Oversample by 8 */
	AD7606_OSR_8,
	/** Oversample by 16 */
	AD7606_OSR_16,
	/** Oversample by 32 */
	AD7606_OSR_32,
	/** Oversample by 64 */
	AD7606_OSR_64,
	/** Oversample by 128, available for chips that have software mode only */
	AD7606_OSR_128,
	/** Oversample by 256, available for chips that have software mode only */
	AD7606_OSR_256
};

void AD7606C_Serial::config(void)
{
	uint8_t byte1;

	check_id();
	write_and_chk_reg(AD7606_REG_CONFIG, 0x00); // CONFIG 0x02(数据中增加状态位，用于区分通道x的数据)
	// write_and_chk_reg(AD7606_REG_CONFIG, 0x02);data_inc_status = 1;
	write_and_chk_reg(AD7606_REG_BANDWIDTH, 0x00); // 全部使用低带宽
	// write_and_chk_reg(AD7606_REG_OVERSAMPLING, 0xF8); // 使用最大过采样率 AD7606_OSR_256 & OS_PAD = 16 => busy=494.6us
	write_and_chk_reg(AD7606_REG_OVERSAMPLING, 0x08);	  // 使用最大过采样率 AD7606_OSR_256 & OS_PAD = 0  => busy=255.6us
	write_and_chk_reg(AD7606_REG_RANGE_CH_ADDR(1), 0xBB); // channel1 & channel2 => ±20 V differential range
	write_and_chk_reg(AD7606_REG_RANGE_CH_ADDR(3), 0x5B); // channel3  => ±20 V differential range channel4 => 0 V to 5 V single-ended range.
	write_and_chk_reg(AD7606_REG_RANGE_CH_ADDR(5), 0x65); // channel5  => 0 V to 5 V single-ended range channel6 => 0 V to 10 V single-ended range.
	write_and_chk_reg(AD7606_REG_RANGE_CH_ADDR(7), 0x15); // channel7  => 0 V to 5 V single-ended range channel8 => ±5 V single-ended range.

	channel_mode[ADC_CH1] = BIPOLAR_MODE;
	channel_mode[ADC_CH2] = BIPOLAR_MODE;
	channel_mode[ADC_CH3] = BIPOLAR_MODE;
	channel_mode[ADC_CH4] = UNIPOLAR_MODE;
	channel_mode[ADC_CH5] = UNIPOLAR_MODE;
	channel_mode[ADC_CH6] = UNIPOLAR_MODE;
	channel_mode[ADC_CH7] = UNIPOLAR_MODE;
	channel_mode[ADC_CH8] = BIPOLAR_MODE;

	write_reg(0, 0); // exti register mode & get into ADC mode
}

/* Internal function to copy the content of a buffer in 18-bit chunks to a 32-bit buffer by
 * extending the chunks to 32-bit size. */
static int32_t cpy18b32b(uint8_t *psrc, uint32_t srcsz, uint32_t *pdst)
{
	unsigned int i, j;

	if (srcsz % 9)
		return 1;

	for (i = 0; i < srcsz; i += 9)
	{
		j = 4 * (i / 9);
		pdst[j + 0] = ((uint32_t)(psrc[i + 0] & 0xff) << 10) | ((uint32_t)psrc[i + 1] << 2) | ((uint32_t)psrc[i + 2] >> 6);
		pdst[j + 1] = ((uint32_t)(psrc[i + 2] & 0x3f) << 12) | ((uint32_t)psrc[i + 3] << 4) | ((uint32_t)psrc[i + 4] >> 4);
		pdst[j + 2] = ((uint32_t)(psrc[i + 4] & 0x0f) << 14) | ((uint32_t)psrc[i + 5] << 6) | ((uint32_t)psrc[i + 6] >> 2);
		pdst[j + 3] = ((uint32_t)(psrc[i + 6] & 0x03) << 16) | ((uint32_t)psrc[i + 7] << 8) | ((uint32_t)psrc[i + 8] >> 0);
	}
	return 0;
}

/* Internal function to copy the content of a buffer in 26-bit chunks to a 32-bit buffer by
 * extending the chunks to 32-bit size. */
static int32_t cpy26b32b(uint8_t *psrc, uint32_t srcsz, uint32_t *pdst)
{
	unsigned int i, j;

	if (srcsz % 13)
		return 1;

	for (i = 0; i < srcsz; i += 13)
	{
		j = 4 * (i / 13);
		pdst[j + 0] = ((uint32_t)(psrc[i + 0] & 0xff) << 18) | ((uint32_t)psrc[i + 1] << 10) | ((uint32_t)psrc[i + 2] << 2) | ((uint32_t)psrc[i + 3] >> 6);
		pdst[j + 1] = ((uint32_t)(psrc[i + 3] & 0x3f) << 20) | ((uint32_t)psrc[i + 4] << 12) | ((uint32_t)psrc[i + 5] << 4) | ((uint32_t)psrc[i + 6] >> 4);
		pdst[j + 2] = ((uint32_t)(psrc[i + 6] & 0x0f) << 22) | ((uint32_t)psrc[i + 7] << 14) | ((uint32_t)psrc[i + 8] << 6) | ((uint32_t)psrc[i + 9] >> 2);
		pdst[j + 3] = ((uint32_t)(psrc[i + 9] & 0x03) << 24) | ((uint32_t)psrc[i + 10] << 16) | ((uint32_t)psrc[i + 11] << 8) | ((uint32_t)psrc[i + 12] >> 0);
	}
	return 0;
}

void AD7606C_Serial::convert_18bit_to_32bit(int32_t *unsigned_val, int32_t srcsz, int32_t *pdst)
{
	unsigned int i;
	for (i = 0; i < srcsz; i++)
	{							  // #Twos Complement Output Coding
								  // Bipolar Analog Input Ranges
		if (channel_mode[i] == 0) // BIPOLAR_MODE
		{
			pdst[i] = (unsigned_val[i] & 0x00020000) ? (unsigned_val[i] | 0xFFFC0000) : unsigned_val[i];
		}
		else // UNIPOLAR_MODE
		{
			pdst[i] = unsigned_val[i];
		}
	}
}

/**
 * @brief Read conversion data.
 *
 * --TODO:-This function performs CRC16 computation and checking if enabled in the device.
 * If the status is enabled in device settings, each sample of data will contain
 * status information in the lowest 8 bits.
 *
 * The output buffer provided by the user should be as wide as to be able to
 * contain 1 sample from each channel since this function reads conversion data
 * across all channels.
 *
 * @param data       - Pointer to location of buffer where to store the data.
 *
 * @return ret 		 - return code.
 *         Example: 1 - xxx error.
 *                  0 - No errors encountered.
 */
uint8_t AD7606C_Serial::data_read(int32_t *data)
{
	uint8_t ret;
	uint32_t sz;
	int32_t i;
	uint16_t crc, icrc;
	uint8_t bits = 18;
	uint8_t sbits = data_inc_status ? 8 : 0;
	uint8_t nchannels = 8;
	uint32_t data_temp[DATA_SIZE];
	uint8_t data_buf[DATA_SIZE] = {0x00};

	sz = nchannels * (bits + sbits);

	/* Number of bits to read, corresponds to SCLK cycles in transfer.
	 * This should always be a multiple of 8 to work with most SPI's.
	 * With this chip family this holds true because we either:
	 *  - multiply 8 channels * bits per sample
	 *  - multiply 4 channels * bits per sample (always multiple of 2)
	 * Therefore, due to design reasons, we don't check for the
	 * remainder of this division because it is zero by design.
	 */
	sz /= 8; // 不包含status时为18个

	memset(data_buf, 0, sz);

	digitalWrite(hspi->pinSS(), LOW); // pull SS slow to prep other end for transfer

	portDISABLE_INTERRUPTS();
	for (uint8_t i = 0; i < sz; i++)
	{
		data_buf[i] = hspi->transfer(0x00);
	}
	portENABLE_INTERRUPTS();
	digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer

	switch (bits)
	{
	case 18:
		if (data_inc_status)
		{
			ret = cpy26b32b(data_buf, sz, data_temp);
			// TODO:
		}
		else
		{
			ret = cpy18b32b(data_buf, sz, data_temp);
			convert_18bit_to_32bit((int32_t *)data_temp, 8, data);
		}

		if (ret)
			return ret;
		break;
	// case 16:
	// 	for (i = 0; i < nchannels; i++)
	// 	{
	// 		if (data_inc_status)
	// 		{
	// 			data[i] = (uint32_t)dev->data[i * 3] << 16;
	// 			data[i] |= (uint32_t)dev->data[i * 3 + 1] << 8;
	// 			data[i] |= (uint32_t)dev->data[i * 3 + 2];
	// 		}
	// 		else
	// 		{
	// 			data[i] = (uint32_t)dev->data[i * 2] << 8;
	// 			data[i] |= (uint32_t)dev->data[i * 2 + 1];
	// 		}
	// 	}
	// 	break;
	default:
		ret = 1;
		break;
	};

	return ret;
}

/**
 * @brief Blocking conversion start and data read.
 *
 * This function performs a conversion start and then proceeds to reading
 * the conversion data.
 *
 * @param data       - Pointer to location of buffer where to store the data.
 *
 * @return ret - return code.
 *                  0 - No errors encountered.
 */
int32_t AD7606C_Serial::read(int32_t *data)
{
	int32_t ret;
	uint8_t busy;
	uint32_t timeout = tconv_max[AD7606_OSR_256];

	ipulse(_CONVST);
	/* Wait for BUSY falling edge */
	while (timeout)
	{
		if (digitalRead(_BUSY) == 0)
			break;

		delayMicroseconds(1);
		timeout--;
	}

	if (timeout == 0)
	{
		Serial.printf("[error]timeout\r\n");
		return 1;
	}

	return data_read(data);
}

int32_t AD7606C_Serial::fast_read(int32_t *data)
{
	int32_t ret;
	uint8_t busy;
	uint32_t timeout = tconv_max[AD7606_OSR_256];

	ipulse(_CONVST);

	data_read(data); // get last sample data
	/* Wait for BUSY falling edge */
	while (timeout)
	{
		if (digitalRead(_BUSY) == 0)
			break;

		delayMicroseconds(1);
		timeout--;
	}

	if (timeout == 0)
	{
		Serial.printf("[error]timeout\r\n");
		return 1;
	}

	return 0;
}
