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
	// Serial.printf("[debug]data_temp:0x%x\r\n", data_temp);
	hspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
	digitalWrite(hspi->pinSS(), LOW);  // pull SS slow to prep other end for transfer
	hspi->transfer16(data_temp);	   // 发送第1帧
	digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer
	delayMicroseconds(1);
	digitalWrite(hspi->pinSS(), LOW);			   // pull SS slow to prep other end for transfer
	*reg_val = hspi->transfer16(data_temp) & 0xFF; // 发送第2帧（与第1帧内容相同） 并且取低8位为寄存器内容
	digitalWrite(hspi->pinSS(), HIGH);			   // pull ss high to signify end of data transfer
	hspi->endTransaction();
	// Serial.printf("[debug]data_temp:0x%4x\r\n", debug);
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
	// Serial.printf("[debug]data_temp:0x%x\r\n", data_temp);
	hspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
	digitalWrite(hspi->pinSS(), LOW);			// pull SS slow to prep other end for transfer
	debug = hspi->transfer16(data_temp) & 0xFF; // 发送第1帧
	digitalWrite(hspi->pinSS(), HIGH);			// pull ss high to signify end of data transfer
	hspi->endTransaction();
	// Serial.printf("[debug]data_temp:0x%4x\r\n", debug); // debug 为连续写入过程中上一次寄存器的地址
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
	// Serial.printf("[debug]data_temp:0x%x\r\n", data_temp);
	hspi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
	digitalWrite(hspi->pinSS(), LOW);			// pull SS slow to prep other end for transfer
	debug = hspi->transfer16(data_temp) & 0xFF; // 发送第1帧
	digitalWrite(hspi->pinSS(), HIGH);			// pull ss high to signify end of data transfer
	hspi->endTransaction();
	// Serial.printf("[debug]data_temp:0x%4x\r\n", debug); // debug 为连续写入过程中上一次寄存器的地址

	read_reg(reg_addr, &data_rd);
	if (data_rd != reg_val)
	{
		Serial.printf("[error]reg write fail!\r\n");
		ret = 1;
	}
	return ret;
}

void AD7606C_Serial::get_id(void)
{
	uint8_t byte1;

	read_reg(0x2F, &byte1);
	if ((byte1 & 0xF0) == 0x30)
	{
		Serial.printf("DEV_ID:D7606C-18 generic. SILICON_REVISION:0x%x\r\n", byte1 & 0x0F);
	}
	else
	{
		Serial.printf("[error]AD7606C Init fail! data:0x%x\r\n", byte1);
	}
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

void AD7606C_Serial::test_reg_rw(void)
{
	uint8_t byte1;

	// 测试寄存器读写
	for (uint8_t i = 0x01; i < 0x40; i++)
	{
		write_reg(0x08, i);
		read_reg(0x08, &byte1);

		if (i != byte1)
		{
			Serial.printf("[error]val wrtie fail!\r\n");
			Serial.printf("[debug]val:0x%2x,data:0x%2x\r\n", i, byte1);
		}
	}
}
void AD7606C_Serial::debug(void)
{

	uint16_t data_temp = 0;

	// exti register mode & get into ADC mode shit！P.47？
	// hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
	// digitalWrite(hspi->pinSS(), LOW);  // pull SS slow to prep other end for transfer
	// hspi->transfer16(data_temp);	   // 发送第1帧
	// digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer
	// hspi->endTransaction();

	// 更换为ADI参考代码的示例
	// exti register mode & get into ADC mode
	/* Enter ADC reading mode by writing at address zero. */
	write_reg(0, 0);

	delay(10);
}

void AD7606C_Serial::config(void)
{
	uint8_t byte1;
	write_and_chk_reg(AD7606_REG_CONFIG, 0x00); // CONFIG 0x02(数据中增加状态位，用于区分通道x的数据)
	// write_and_chk_reg(AD7606_REG_CONFIG, 0x02);data_inc_status = 1;
	write_and_chk_reg(AD7606_REG_BANDWIDTH, 0x00); // 全部使用低带宽
	// write_and_chk_reg(AD7606_REG_OVERSAMPLING, 0xF8); // 使用最大过采样率 AD7606_OSR_256 & OS_PAD = 16 => busy=494.6us
	write_and_chk_reg(AD7606_REG_OVERSAMPLING, 0x08); // 使用最大过采样率 AD7606_OSR_256 & OS_PAD = 0  => busy=255.6us
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

/**
 * @brief Read conversion data.
 *
 * ---------------------------------------------------TODO:---------------------------This function performs CRC16 computation and checking if enabled in the device.
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
uint8_t AD7606C_Serial::data_read(uint32_t *data)
{
	uint8_t ret;
	uint32_t sz;
	int32_t i;
	uint16_t crc, icrc;
	uint8_t bits = 18;
	uint8_t sbits = data_inc_status ? 8 : 0;
	uint8_t nchannels = 8;

	uint8_t data_buf[28] = {0x00};

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
	for (uint8_t i = 0; i < sz; i++)
	{
		data_buf[i] = hspi->transfer(0x00);
	}
	digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer

	// for (uint8_t i = 0; i < sz; i++)
	// {
	// 	Serial.printf("[debug]0x%.2d=0x%.4x\r\n", i, data_buf[i]);
	// }

	switch (bits)
	{
	case 18:
		if (data_inc_status)
			ret = cpy26b32b(data_buf, sz, data);
		else
		{
			// #warning "此处直接data_buf 为16bits，需要确认能否直接转换为8bits"
			ret = cpy18b32b(data_buf, sz, data);
			// Serial.printf("[debug]status=0 size:%d\r\n", sz);
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

	// for (uint8_t i = 0; i < sz; i++)
	// {
	// 	Serial.printf("[debug]0x%.2d=0x%.8x\r\n", i, data[i]);
	// }
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
int32_t AD7606C_Serial::read(uint32_t *data)
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

int32_t AD7606C_Serial::fast_read(uint32_t *data)
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

// /***************************************************************************//**
//  * @brief Read conversion data.
//  *
//  * This function performs CRC16 computation and checking if enabled in the device.
//  * If the status is enabled in device settings, each sample of data will contain
//  * status information in the lowest 8 bits.
//  *
//  * The output buffer provided by the user should be as wide as to be able to
//  * contain 1 sample from each channel since this function reads conversion data
//  * across all channels.
//  *
//  * @param dev        - The device structure.
//  * @param data       - Pointer to location of buffer where to store the data.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  -ENOTSUP - Device bits per sample not supported.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_spi_data_read(struct ad7606_dev *dev, uint32_t *data)
// {
// 	uint32_t sz;
// 	int32_t ret, i;
// 	uint16_t crc, icrc;
// 	uint8_t bits = ad7606_chip_info_tbl[dev->device_id].bits;
// 	uint8_t sbits = dev->config.status_header ? 8 : 0;
// 	uint8_t nchannels = ad7606_chip_info_tbl[dev->device_id].num_channels;

// 	sz = nchannels * (bits + sbits);

// 	/* Number of bits to read, corresponds to SCLK cycles in transfer.
// 	 * This should always be a multiple of 8 to work with most SPI's.
// 	 * With this chip family this holds true because we either:
// 	 *  - multiply 8 channels * bits per sample
// 	 *  - multiply 4 channels * bits per sample (always multiple of 2)
// 	 * Therefore, due to design reasons, we don't check for the
// 	 * remainder of this division because it is zero by design.
// 	 */
// 	sz /= 8;

// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		sz += 2;
// 	}

// 	memset(dev->data, 0, sz);
// 	ret = no_os_spi_write_and_read(dev->spi_desc, dev->data, sz);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		sz -= 2;
// 		crc = no_os_crc16(ad7606_crc16, dev->data, sz, 0);
// 		icrc = ((uint16_t)dev->data[sz] << 8) |
// 		       dev->data[sz+1];
// 		if (icrc != crc)
// 			return -EBADMSG;
// 	}

// 	switch(bits) {
// 	case 18:
// 		if (dev->config.status_header)
// 			ret = cpy26b32b(dev->data, sz, data);
// 		else
// 			ret = cpy18b32b(dev->data, sz, data);
// 		if (ret < 0)
// 			return ret;
// 		break;
// 	case 16:
// 		for(i = 0; i < nchannels; i++) {
// 			if (dev->config.status_header) {
// 				data[i] = (uint32_t)dev->data[i*3] << 16;
// 				data[i] |= (uint32_t)dev->data[i*3+1] << 8;
// 				data[i] |= (uint32_t)dev->data[i*3+2];
// 			} else {
// 				data[i] = (uint32_t)dev->data[i*2] << 8;
// 				data[i] |= (uint32_t)dev->data[i*2+1];
// 			}
// 		}
// 		break;
// 	default:
// 		ret = -ENOTSUP;
// 		break;
// 	};

// 	return ret;
// }

// /***************************************************************************//**
//  *   @file   ad7606.c
//  *   @brief  Implementation of ad7606 Driver.
//  *   @author Stefan Popa (stefan.popa@analog.com)
//  *   @author Darius Berghe (darius.berghe@analog.com)
// ********************************************************************************
//  * Copyright 2019(c) Analog Devices, Inc.
//  *
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *  - Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *  - Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  *  - Neither the name of Analog Devices, Inc. nor the names of its
//  *    contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *  - The use of this software may or may not infringe the patent rights
//  *    of one or more patent holders.  This license does not release you
//  *    from the requirement that you obtain separate licenses from these
//  *    patent holders to use this software.
//  *  - Use of the software either in source or binary form, must be run
//  *    on or directly connected to an Analog Devices Inc. component.
//  *
//  * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
//  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
//  * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//  * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *******************************************************************************/

// /******************************************************************************/
// /***************************** Include Files **********************************/
// /******************************************************************************/
// #include <stdio.h>
// #include <stdlib.h>
// #include <stdbool.h>
// #include <string.h>
// #include <errno.h>
// #include "ad7606.h"
// #include "no_os_error.h"
// #include "no_os_util.h"
// #include "no_os_crc.h"
// #include "no_os_alloc.h"

// struct ad7606_chip_info {
// 	uint8_t num_channels;
// 	uint8_t bits;
// 	uint8_t max_dout_lines;
// 	bool has_oversampling;
// 	bool has_registers;
// 	uint8_t device_id;
// 	const struct ad7606_range *hw_range_table;
// 	uint32_t hw_range_table_sz;
// 	const struct ad7606_range *sw_range_table;
// 	uint32_t sw_range_table_sz;
// };

// NO_OS_DECLARE_CRC8_TABLE(ad7606_crc8);
// NO_OS_DECLARE_CRC16_TABLE(ad7606_crc16);

// static const struct ad7606_range ad7606_range_table[] = {
// 	{-5000, 5000, false},	/* RANGE pin LOW */
// 	{-10000, 10000, false},	/* RANGE pin HIGH */
// };

// static const struct ad7606_range ad7609_range_table[] = {
// 	{-10000, 10000, true},	/* RANGE pin LOW */
// 	{-20000, 20000, true},	/* RANGE pin HIGH */
// };

// static const struct ad7606_range ad7606b_range_table[] = {
// 	{-2500, 2500, false},	/* 0000 */
// 	{-5000, 5000, false},	/* 0001 */
// 	{-10000, 10000, false},	/* 0010 */
// 	{-10000, 10000, false},	/* 0011 */
// 	{-10000, 10000, false},	/* 0100 */
// 	{-10000, 10000, false},	/* 0101 */
// 	{-10000, 10000, false},	/* 0110 */
// 	{-10000, 10000, false},	/* 0111 */
// 	{-10000, 10000, false},	/* 1000 */
// 	{-10000, 10000, false},	/* 1001 */
// 	{-10000, 10000, false},	/* 1010 */
// 	{-10000, 10000, false},	/* 1011 */
// };

// static const struct ad7606_range ad7606c_range_table[] = {
// 	{-2500, 2500, false},	/* 0000 */
// 	{-5000, 5000, false},	/* 0001 */
// 	{-6250, 6250, false},	/* 0010 */
// 	{-10000, 10000, false},	/* 0011 */
// 	{-12500, 12500, false},	/* 0100 */
// 	{0, 5000, false},	/* 0101 */
// 	{0, 10000, false},	/* 0110 */
// 	{0, 12500, false},	/* 0111 */
// 	{-5000, 5000, true},	/* 1000 */
// 	{-10000, 10000, true},	/* 1001 */
// 	{-12500, 12500, true},	/* 1010 */
// 	{-20000, 20000, true},	/* 1011 */
// 	{-20000, 20000, true},	/* 1100 */
// 	{-20000, 20000, true},	/* 1101 */
// 	{-20000, 20000, true},	/* 1110 */
// 	{-20000, 20000, true},	/* 1111 */
// };

// static const struct ad7606_chip_info ad7606_chip_info_tbl[] = {
// 	[ID_AD7605_4] = {
// 		.num_channels = 4,
// 		.bits = 16,
// 		.max_dout_lines = AD7606_2_DOUT,
// 		.has_oversampling = false,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 	},
// 	[ID_AD7606_4] = {
// 		.num_channels = 4,
// 		.bits = 16,
// 		.max_dout_lines = AD7606_2_DOUT,
// 		.has_oversampling = true,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 	},
// 	[ID_AD7606_6] = {
// 		.num_channels = 6,
// 		.bits = 16,
// 		.max_dout_lines = AD7606_2_DOUT,
// 		.has_oversampling = true,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 	},
// 	[ID_AD7606_8] = {
// 		.num_channels = 8,
// 		.bits = 16,
// 		.max_dout_lines = AD7606_2_DOUT,
// 		.has_oversampling = true,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 	},
// 	[ID_AD7606B] = {
// 		.num_channels = 8,
// 		.bits = 16,
// 		.max_dout_lines = AD7606_4_DOUT,
// 		.has_oversampling = true,
// 		.has_registers = true,
// 		.device_id = 0x1,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 		.sw_range_table = ad7606b_range_table,
// 		.sw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606b_range_table),
// 	},
// 	[ID_AD7606C_16] = {
// 		.num_channels = 8,
// 		.bits = 16,
// 		.max_dout_lines = AD7606_8_DOUT,
// 		.has_oversampling = true,
// 		.has_registers = true,
// 		.device_id = 0x3,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 		.sw_range_table = ad7606c_range_table,
// 		.sw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606c_range_table),
// 	},
// 	[ID_AD7606C_18] = {
// 		.num_channels = 8,
// 		.bits = 18,
// 		.max_dout_lines = AD7606_8_DOUT,
// 		.has_oversampling = true,
// 		.has_registers = true,
// 		.device_id = 0x3,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 		.sw_range_table = ad7606c_range_table,
// 		.sw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606c_range_table),
// 	},
// 	[ID_AD7608] = {
// 		.num_channels = 8,
// 		.bits = 18,
// 		.max_dout_lines = AD7606_2_DOUT,
// 		.has_oversampling = true,
// 		.hw_range_table = ad7606_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7606_range_table),
// 	},
// 	[ID_AD7609] = {
// 		.num_channels = 8,
// 		.bits = 18,
// 		.max_dout_lines = AD7606_2_DOUT,
// 		.has_oversampling = true,
// 		.hw_range_table = ad7609_range_table,
// 		.hw_range_table_sz = NO_OS_ARRAY_SIZE(ad7609_range_table),
// 	},
// };

// static const uint16_t tconv_max[] = {
// 	1, 	/* AD7606_OSR_1 */
// 	3,	/* AD7606_OSR_2 */
// 	5,	/* AD7606_OSR_4 */
// 	10,	/* AD7606_OSR_8 */
// 	20,	/* AD7606_OSR_16 */
// 	41,	/* AD7606_OSR_32 */
// 	81,	/* AD7606_OSR_64 */
// 	162,	/* AD7606_OSR_128 */
// 	324	/* AD7606_OSR_256 */
// };

// /***************************************************************************//**
//  * @brief Read a device register via SPI.
//  *
//  * This function performs CRC8 computation and checking if enabled in the device.
//  *
//  * @param dev        - The device structure.
//  * @param reg_addr   - Register address in device memory.
//  * @param reg_data   - Pointer to the location where to store the register value.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_spi_reg_read(struct ad7606_dev *dev,
// 			    uint8_t reg_addr,
// 			    uint8_t *reg_data)
// {
// 	uint8_t buf[3];
// 	uint8_t crc;
// 	uint32_t sz = 2;
// 	int32_t ret;

// 	if (!dev->sw_mode)
// 		return -ENOTSUP;

// 	buf[0] = AD7606_RD_FLAG_MSK(reg_addr);
// 	buf[1] = 0x00;
// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		crc = no_os_crc8(ad7606_crc8, buf, 2, 0);
// 		buf[2] = crc;
// 		sz += 1;
// 	}
// 	ret = no_os_spi_write_and_read(dev->spi_desc, buf, sz);
// 	if (ret < 0)
// 		return ret;

// 	dev->reg_mode = true;

// 	buf[0] = AD7606_RD_FLAG_MSK(reg_addr);
// 	buf[1] = 0x00;
// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		crc = no_os_crc8(ad7606_crc8, buf, 2, 0);
// 		buf[2] = crc;
// 	}
// 	ret = no_os_spi_write_and_read(dev->spi_desc, buf, sz);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		crc = no_os_crc8(ad7606_crc8, buf, 2, 0);
// 		if (crc != buf[2])
// 			return -EBADMSG;
// 	}

// 	if (reg_data)
// 		*reg_data = buf[1];

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Write a device register via SPI.
//  *
//  * This function performs CRC8 computation and checking if enabled in the device.
//  *
//  * @param dev        - The device structure.
//  * @param reg_addr   - Register address in device memory.
//  * @param reg_data   - Value to write to register.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_spi_reg_write(struct ad7606_dev *dev,
// 			     uint8_t reg_addr,
// 			     uint8_t reg_data)
// {
// 	uint8_t buf[3];
// 	int32_t ret;
// 	uint8_t crc;
// 	uint32_t sz = 2;

// 	if (!dev->sw_mode)
// 		return -ENOTSUP;

// 	/* Dummy read to place the chip in register mode. */
// 	if (!dev->reg_mode) {
// 		ret = ad7606_spi_reg_read(dev, reg_addr, NULL);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	buf[0] = AD7606_WR_FLAG_MSK(reg_addr);
// 	buf[1] = reg_data;
// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		crc = no_os_crc8(ad7606_crc8, buf, 2, 0);
// 		buf[2] = crc;
// 		sz += 1;
// 	}

// 	ret = no_os_spi_write_and_read(dev->spi_desc, buf, sz);
// 	if (ret < 0)
// 		return ret;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Write a device register via SPI with masking.
//  *
//  * @param dev        - The device structure.
//  * @param addr       - Register address in device memory.
//  * @param mask       - Only bits set to 1 in mask will be modified.
//  * @param val        - Value to write to register.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_spi_write_mask(struct ad7606_dev *dev,
// 			      uint32_t addr,
// 			      uint32_t mask,
// 			      uint32_t val)
// {
// 	uint8_t reg_data;
// 	int ret;

// 	ret = ad7606_spi_reg_read(dev, addr, &reg_data);
// 	if (ret < 0)
// 		return ret;

// 	reg_data &= ~mask;
// 	reg_data |= val;

// 	return ad7606_spi_reg_write(dev, addr, reg_data);
// }

// /* Internal function to copy the content of a buffer in 18-bit chunks to a 32-bit buffer by
//  * extending the chunks to 32-bit size. */
// static int32_t cpy18b32b(uint8_t *psrc, uint32_t srcsz, uint32_t *pdst)
// {
// 	unsigned int i, j;

// 	if (srcsz % 9)
// 		return -EINVAL;

// 	for(i = 0; i < srcsz; i += 9) {
// 		j = 4 * (i / 9);
// 		pdst[j+0] = ((uint32_t)(psrc[i+0] & 0xff) << 10) | ((uint32_t)psrc[i+1] << 2)
// 			    | ((uint32_t)psrc[i+2] >> 6);
// 		pdst[j+1] = ((uint32_t)(psrc[i+2] & 0x3f) << 12) | ((uint32_t)psrc[i+3] << 4)
// 			    | ((uint32_t)psrc[i+4] >> 4);
// 		pdst[j+2] = ((uint32_t)(psrc[i+4] & 0x0f) << 14) | ((uint32_t)psrc[i+5] << 6)
// 			    | ((uint32_t)psrc[i+6] >> 2);
// 		pdst[j+3] = ((uint32_t)(psrc[i+6] & 0x03) << 16) | ((uint32_t)psrc[i+7] << 8)
// 			    | ((uint32_t)psrc[i+8] >> 0);
// 	}
// 	return 0;
// }

// /* Internal function to copy the content of a buffer in 26-bit chunks to a 32-bit buffer by
//  * extending the chunks to 32-bit size. */
// static int32_t cpy26b32b(uint8_t *psrc, uint32_t srcsz, uint32_t *pdst)
// {
// 	unsigned int i, j;

// 	if (srcsz % 13)
// 		return -EINVAL;

// 	for(i = 0; i < srcsz; i += 13) {
// 		j = 4 * (i / 13);
// 		pdst[j+0] = ((uint32_t)(psrc[i+0] & 0xff) << 18) | ((uint32_t)psrc[i+1] << 10)
// 			    | ((uint32_t)psrc[i+2] << 2) | ((uint32_t)psrc[i+3] >> 6);
// 		pdst[j+1] = ((uint32_t)(psrc[i+3] & 0x3f) << 20) | ((uint32_t)psrc[i+4] << 12)
// 			    | ((uint32_t)psrc[i+5] << 4) | ((uint32_t)psrc[i+6] >> 4);
// 		pdst[j+2] = ((uint32_t)(psrc[i+6] & 0x0f) << 22) | ((uint32_t)psrc[i+7] << 14)
// 			    | ((uint32_t)psrc[i+8] << 6) | ((uint32_t)psrc[i+9] >> 2);
// 		pdst[j+3] = ((uint32_t)(psrc[i+9] & 0x03) << 24) | ((uint32_t)psrc[i+10] << 16)
// 			    | ((uint32_t)psrc[i+11] << 8) | ((uint32_t)psrc[i+12] >> 0);
// 	}
// 	return 0;
// }

// /***************************************************************************//**
//  * @brief Toggle the CONVST pin to start a conversion.
//  *
//  * If needed, this function also puts the device in ADC reading mode by a write
//  * at address zero.
//  *
//  * @param dev        - The device structure.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EIO - CONVST GPIO not available.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_convst(struct ad7606_dev *dev)
// {
// 	int32_t ret;

// 	if (dev->reg_mode) {
// 		/* Enter ADC reading mode by writing at address zero. */
// 		ret = ad7606_spi_reg_write(dev, 0, 0);
// 		if (ret < 0)
// 			return ret;

// 		dev->reg_mode = false;
// 	}

// 	ret = no_os_gpio_set_value(dev->gpio_convst, 0);
// 	if (ret < 0)
// 		return ret;

// 	/* wait LP_CNV time */
// 	no_os_udelay(1);

// 	return no_os_gpio_set_value(dev->gpio_convst, 1);
// }

// /***************************************************************************//**
//  * @brief Read conversion data.
//  *
//  * This function performs CRC16 computation and checking if enabled in the device.
//  * If the status is enabled in device settings, each sample of data will contain
//  * status information in the lowest 8 bits.
//  *
//  * The output buffer provided by the user should be as wide as to be able to
//  * contain 1 sample from each channel since this function reads conversion data
//  * across all channels.
//  *
//  * @param dev        - The device structure.
//  * @param data       - Pointer to location of buffer where to store the data.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  -ENOTSUP - Device bits per sample not supported.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_spi_data_read(struct ad7606_dev *dev, uint32_t *data)
// {
// 	uint32_t sz;
// 	int32_t ret, i;
// 	uint16_t crc, icrc;
// 	uint8_t bits = ad7606_chip_info_tbl[dev->device_id].bits;
// 	uint8_t sbits = dev->config.status_header ? 8 : 0;
// 	uint8_t nchannels = ad7606_chip_info_tbl[dev->device_id].num_channels;

// 	sz = nchannels * (bits + sbits);

// 	/* Number of bits to read, corresponds to SCLK cycles in transfer.
// 	 * This should always be a multiple of 8 to work with most SPI's.
// 	 * With this chip family this holds true because we either:
// 	 *  - multiply 8 channels * bits per sample
// 	 *  - multiply 4 channels * bits per sample (always multiple of 2)
// 	 * Therefore, due to design reasons, we don't check for the
// 	 * remainder of this division because it is zero by design.
// 	 */
// 	sz /= 8;

// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		sz += 2;
// 	}

// 	memset(dev->data, 0, sz);
// 	ret = no_os_spi_write_and_read(dev->spi_desc, dev->data, sz);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->digital_diag_enable.int_crc_err_en) {
// 		sz -= 2;
// 		crc = no_os_crc16(ad7606_crc16, dev->data, sz, 0);
// 		icrc = ((uint16_t)dev->data[sz] << 8) |
// 		       dev->data[sz+1];
// 		if (icrc != crc)
// 			return -EBADMSG;
// 	}

// 	switch(bits) {
// 	case 18:
// 		if (dev->config.status_header)
// 			ret = cpy26b32b(dev->data, sz, data);
// 		else
// 			ret = cpy18b32b(dev->data, sz, data);
// 		if (ret < 0)
// 			return ret;
// 		break;
// 	case 16:
// 		for(i = 0; i < nchannels; i++) {
// 			if (dev->config.status_header) {
// 				data[i] = (uint32_t)dev->data[i*3] << 16;
// 				data[i] |= (uint32_t)dev->data[i*3+1] << 8;
// 				data[i] |= (uint32_t)dev->data[i*3+2];
// 			} else {
// 				data[i] = (uint32_t)dev->data[i*2] << 8;
// 				data[i] |= (uint32_t)dev->data[i*2+1];
// 			}
// 		}
// 		break;
// 	default:
// 		ret = -ENOTSUP;
// 		break;
// 	};

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Blocking conversion start and data read.
//  *
//  * This function performs a conversion start and then proceeds to reading
//  * the conversion data.
//  *
//  * @param dev        - The device structure.
//  * @param data       - Pointer to location of buffer where to store the data.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -ETIME - Timeout while waiting for the BUSY signal.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_read(struct ad7606_dev *dev, uint32_t * data)
// {
// 	int32_t ret;
// 	uint8_t busy;
// 	uint32_t timeout = tconv_max[AD7606_OSR_256];

// 	ret = ad7606_convst(dev);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_busy) {
// 		/* Wait for BUSY falling edge */
// 		while(timeout) {
// 			ret = no_os_gpio_get_value(dev->gpio_busy, &busy);
// 			if (ret < 0)
// 				return ret;

// 			if (busy == 0)
// 				break;

// 			no_os_udelay(1);
// 			timeout--;
// 		}

// 		if (timeout == 0)
// 			return -ETIME;
// 	} else {
// 		/* wait CONV time */
// 		no_os_udelay(tconv_max[dev->oversampling.os_ratio]);
// 	}

// 	return ad7606_spi_data_read(dev, data);
// }

// /* Internal function to reset device settings to default state after chip reset. */
// static inline void ad7606_reset_settings(struct ad7606_dev *dev)
// {
// 	int i;
// 	const struct ad7606_range *rt = dev->sw_mode ?
// 						ad7606_chip_info_tbl[dev->device_id].sw_range_table:
// 						ad7606_chip_info_tbl[dev->device_id].hw_range_table;

// 	for(i = 0; i < dev->num_channels; i++) {
// 		if (dev->sw_mode)
// 			dev->range_ch[i] = rt[3];
// 		else
// 			dev->range_ch[i] = rt[0];

// 		dev->offset_ch[i] = 0;
// 		dev->phase_ch[i] = 0;
// 		dev->gain_ch[i] = 0;
// 	}

// 	dev->oversampling.os_ratio = AD7606_OSR_1;
// 	dev->oversampling.os_pad = 0;
// 	dev->config.op_mode = AD7606_NORMAL;
// 	dev->config.dout_format = AD7606_2_DOUT;
// 	dev->config.ext_os_clock = false;
// 	dev->config.status_header = false;
// 	dev->digital_diag_enable.rom_crc_err_en = true;
// 	dev->digital_diag_enable.mm_crc_err_en = false;
// 	dev->digital_diag_enable.int_crc_err_en = false;
// 	dev->digital_diag_enable.spi_write_err_en = false;
// 	dev->digital_diag_enable.spi_read_err_en = false;
// 	dev->digital_diag_enable.busy_stuck_high_err_en = false;
// 	dev->digital_diag_enable.clk_fs_os_counter_en = false;
// 	dev->digital_diag_enable.interface_check_en = false;
// 	dev->reg_mode = false;
// }

// /***************************************************************************//**
//  * @brief Reset the device by toggling the reset GPIO.
//  *
//  * @param dev        - The device structure.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - Reset GPIO not available.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_reset(struct ad7606_dev *dev)
// {
// 	int32_t ret;

// 	ret = no_os_gpio_set_value(dev->gpio_reset, 1);
// 	if (ret < 0)
// 		return ret;

// 	no_os_udelay(3);

// 	ret = no_os_gpio_set_value(dev->gpio_reset, 0);
// 	if (ret < 0)
// 		return ret;

// 	ad7606_reset_settings(dev);

// 	return ret;
// }

// /* Internal function that initializes GPIOs. */
// static int32_t ad7606_request_gpios(struct ad7606_dev *dev,
// 				    struct ad7606_init_param *init_param)
// {
// 	int32_t ret;

// 	ret = no_os_gpio_get_optional(&dev->gpio_reset, init_param->gpio_reset);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_reset) {
// 		ret = no_os_gpio_direction_output(dev->gpio_reset, NO_OS_GPIO_LOW);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_convst, init_param->gpio_convst);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_convst) {
// 		ret = no_os_gpio_direction_output(dev->gpio_convst, NO_OS_GPIO_LOW);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_busy, init_param->gpio_busy);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_busy) {
// 		ret = no_os_gpio_direction_input(dev->gpio_busy);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_stby_n, init_param->gpio_stby_n);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_stby_n) {
// 		ret = no_os_gpio_direction_output(dev->gpio_stby_n, NO_OS_GPIO_HIGH);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_range, init_param->gpio_range);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_range) {
// 		ret = no_os_gpio_direction_output(dev->gpio_range, NO_OS_GPIO_LOW);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	if (!ad7606_chip_info_tbl[dev->device_id].has_oversampling)
// 		return ret;

// 	ret = no_os_gpio_get_optional(&dev->gpio_os0, init_param->gpio_os0);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_os0) {
// 		ret = no_os_gpio_direction_output(dev->gpio_os0, NO_OS_GPIO_LOW);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_os1, init_param->gpio_os1);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_os1) {
// 		ret = no_os_gpio_direction_output(dev->gpio_os1, NO_OS_GPIO_LOW);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_os2, init_param->gpio_os2);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_os2) {
// 		ret = no_os_gpio_direction_output(dev->gpio_os2, NO_OS_GPIO_LOW);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	ret = no_os_gpio_get_optional(&dev->gpio_par_ser, init_param->gpio_par_ser);
// 	if (ret < 0)
// 		return ret;

// 	if (dev->gpio_par_ser) {
// 		/* Driver currently supports only serial interface, therefore,
// 		 * if available, pull the GPIO HIGH. */
// 		ret = no_os_gpio_direction_output(dev->gpio_par_ser, NO_OS_GPIO_HIGH);
// 		if (ret < 0)
// 			return ret;
// 	}

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Set the oversampling ratio.
//  *
//  * In hardware mode, it silently sets AD7606_OSR_64 if higher oversampling
//  * is provided.
//  *
//  * @param dev          - The device structure.
//  * @param oversampling - Oversampling settings.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_oversampling(struct ad7606_dev *dev,
// 				struct ad7606_oversampling oversampling)
// {
// 	int32_t ret;
// 	uint8_t val;

// 	if (dev->sw_mode) {
// 		val = no_os_field_prep(AD7606_OS_RATIO_MSK, oversampling.os_ratio);
// 		val |= no_os_field_prep(AD7606_OS_PAD_MSK, oversampling.os_pad);
// 		ret = ad7606_spi_reg_write(dev, AD7606_REG_OVERSAMPLING, val);
// 		if (ret < 0)
// 			return ret;
// 	} else {
// 		/* In hardware mode, OSR 128 and 256 are not avaialable */
// 		if (oversampling.os_ratio > AD7606_OSR_64)
// 			oversampling.os_ratio = AD7606_OSR_64;

// 		ret = no_os_gpio_set_value(dev->gpio_os0,
// 					   ((oversampling.os_ratio & 0x01) >> 0));
// 		if (ret < 0)
// 			return ret;

// 		ret = no_os_gpio_set_value(dev->gpio_os1,
// 					   ((oversampling.os_ratio & 0x02) >> 1));
// 		if (ret < 0)
// 			return ret;

// 		ret = no_os_gpio_set_value(dev->gpio_os2,
// 					   ((oversampling.os_ratio & 0x04) >> 2));
// 		if (ret < 0)
// 			return ret;
// 	}

// 	dev->oversampling = oversampling;

// 	return 0;
// }

// /* Internal function to find the index of a given operation range in the
//  * operation range table specific to a device. */
// static int8_t ad7606_find_range(struct ad7606_dev *dev,
// 				struct ad7606_range range)
// {
// 	uint8_t i;
// 	int8_t v = -1;
// 	const struct ad7606_range *rt = dev->sw_mode ?
// 						ad7606_chip_info_tbl[dev->device_id].sw_range_table:
// 						ad7606_chip_info_tbl[dev->device_id].hw_range_table;

// 	uint32_t rtsz = dev->sw_mode ?
// 			ad7606_chip_info_tbl[dev->device_id].sw_range_table_sz:
// 			ad7606_chip_info_tbl[dev->device_id].hw_range_table_sz;

// 	for (i = 0; i < rtsz; i++) {
// 		if (range.min != rt[i].min)
// 			continue;
// 		if (range.max != rt[i].max)
// 			continue;
// 		if (range.differential != rt[i].differential)
// 			continue;
// 		v = i;
// 		break;
// 	}

// 	return v;
// }

// /***************************************************************************//**
//  * @brief Set the channel operation range.
//  *
//  * @param dev        - The device structure.
//  * @param ch         - Channel number (0-7).
//  * @param range      - Operation range.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EINVAL - Invalid input.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_ch_range(struct ad7606_dev *dev, uint8_t ch,
// 			    struct ad7606_range range)
// {
// 	int value;
// 	int32_t ret;

// 	if (range.min > range.max)
// 		return -EINVAL;

// 	if (ch >= dev->num_channels)
// 		return -EINVAL;

// 	value = ad7606_find_range(dev, range);
// 	if (value < 0)
// 		return -EINVAL;

// 	if (dev->sw_mode)
// 		ret = ad7606_spi_write_mask(dev, AD7606_REG_RANGE_CH_ADDR(ch),
// 					    AD7606_RANGE_CH_MSK(ch),
// 					    AD7606_RANGE_CH_MODE(ch, value));
// 	else
// 		ret = no_os_gpio_set_value(dev->gpio_range, value);

// 	if (ret)
// 		return ret;

// 	dev->range_ch[ch] = range;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Set the channel offset.
//  *
//  * The offset parameter is a signed 8-bit integer ranging from -128 to 127 to
//  * make it intuitive and user-friendly.
//  *
//  * This offset gets converted to the register representation where 0x80 is
//  * calibration offset 0, 0x0 is calibration offset -128 and 0xFF is calibration
//  * offset 127, etc.
//  *
//  * @param dev        - The device structure.
//  * @param ch         - Channel number (0-7).
//  * @param offset     - Offset calibration amount (-128...127).
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EINVAL - Invalid input.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_ch_offset(struct ad7606_dev *dev, uint8_t ch,
// 			     int8_t offset)
// {
// 	int ret;
// 	uint8_t value = (uint8_t)(offset - 0x80);

// 	if (ch >= dev->num_channels)
// 		return -EINVAL;

// 	if (!dev->sw_mode)
// 		return -ENOTSUP;

// 	ret = ad7606_spi_reg_write(dev, AD7606_REG_OFFSET_CH(ch), value);
// 	if (ret < 0)
// 		return ret;

// 	dev->offset_ch[ch] = offset;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Set the channel phase.
//  *
//  * @param dev        - The device structure.
//  * @param ch         - Channel number (0-7).
//  * @param phase      - Phase calibration amount.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EINVAL - Invalid input.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_ch_phase(struct ad7606_dev *dev, uint8_t ch,
// 			    uint8_t phase)
// {
// 	int ret;

// 	if (ch >= dev->num_channels)
// 		return -EINVAL;

// 	if (!dev->sw_mode)
// 		return -ENOTSUP;

// 	ret = ad7606_spi_reg_write(dev, AD7606_REG_PHASE_CH(ch), phase);
// 	if (ret < 0)
// 		return ret;

// 	dev->phase_ch[ch] = phase;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Set the channel gain.
//  *
//  * @param dev        - The device structure.
//  * @param ch         - Channel number (0-7).
//  * @param gain       - Gain calibration amount.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EINVAL - Invalid input.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_ch_gain(struct ad7606_dev *dev, uint8_t ch,
// 			   uint8_t gain)
// {
// 	int ret;

// 	if (ch >= dev->num_channels)
// 		return -EINVAL;

// 	if (!dev->sw_mode)
// 		return -ENOTSUP;

// 	gain = no_os_field_get(AD7606_GAIN_MSK, gain);
// 	ret = ad7606_spi_reg_write(dev, AD7606_REG_GAIN_CH(ch), gain);
// 	if (ret < 0)
// 		return ret;

// 	dev->gain_ch[ch] = gain;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Set the device config register.
//  *
//  * Configuration structure affects the CONFIG register of the device.
//  *
//  * @param dev        - The device structure.
//  * @param config     - Configuration structure.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -EIO - GPIO not available.
//  *                  -EINVAL - Invalid input.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_config(struct ad7606_dev *dev,
// 			  struct ad7606_config config)
// {
// 	int32_t ret;
// 	uint8_t val = 0;
// 	uint8_t range_pin, stby_n_pin;

// 	if (dev->sw_mode) {

// 		val |= no_os_field_prep(AD7606_CONFIG_OPERATION_MODE_MSK, config.op_mode);
// 		/* This driver currently supports only normal SPI with 1 DOUT line.
// 		 * TODO: remove this check when implementing multi-line DOUT. */
// 		if ((uint8_t)config.dout_format > AD7606_1_DOUT)
// 			return -EINVAL;
// 		if ((uint8_t)config.dout_format > (uint8_t)dev->max_dout_lines)
// 			return -EINVAL;
// 		val |= no_os_field_prep(AD7606_CONFIG_DOUT_FORMAT_MSK, config.dout_format);
// 		val |= no_os_field_prep(AD7606_CONFIG_EXT_OS_CLOCK_MSK, config.ext_os_clock);
// 		val |= no_os_field_prep(AD7606_CONFIG_STATUS_HEADER_MSK, config.status_header);

// 		ret = ad7606_spi_reg_write(dev, AD7606_REG_CONFIG, val);
// 		if (ret)
// 			return ret;
// 	} else {
// 		switch(config.op_mode) {
// 		case AD7606_NORMAL:
// 			range_pin = NO_OS_GPIO_LOW;
// 			stby_n_pin = NO_OS_GPIO_HIGH;
// 			break;
// 		case AD7606_STANDBY:
// 			range_pin = NO_OS_GPIO_LOW;
// 			stby_n_pin = NO_OS_GPIO_LOW;
// 			break;
// 		case AD7606_SHUTDOWN:
// 			range_pin = NO_OS_GPIO_HIGH;
// 			stby_n_pin = NO_OS_GPIO_LOW;
// 			break;
// 		default:
// 			return -EINVAL;
// 		};

// 		ret = no_os_gpio_set_value(dev->gpio_stby_n, stby_n_pin);
// 		if (ret)
// 			return ret;

// 		ret = no_os_gpio_set_value(dev->gpio_range, range_pin);
// 		if (ret)
// 			return ret;
// 	}

// 	dev->config = config;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Set the device digital diagnostics configuration.
//  *
//  * Digital diagnostics structure affects the DIGITAL_DIAG register of the device.
//  *
//  * @param dev        - The device structure.
//  * @param diag       - Configuration structure.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  -ENOTSUP - Device not in software mode.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_set_digital_diag(struct ad7606_dev *dev,
// 				struct ad7606_digital_diag diag)
// {
// 	int32_t ret;
// 	uint8_t val = 0;

// 	if (!dev->sw_mode)
// 		return -ENOTSUP;

// 	val |= no_os_field_prep(AD7606_ROM_CRC_ERR_EN_MSK, diag.rom_crc_err_en);
// 	val |= no_os_field_prep(AD7606_MM_CRC_ERR_EN_MSK, diag.mm_crc_err_en);
// 	val |= no_os_field_prep(AD7606_INT_CRC_ERR_EN_MSK, diag.int_crc_err_en);
// 	val |= no_os_field_prep(AD7606_SPI_WRITE_ERR_EN_MSK, diag.spi_write_err_en);
// 	val |= no_os_field_prep(AD7606_SPI_READ_ERR_EN_MSK, diag.spi_read_err_en);
// 	val |= no_os_field_prep(AD7606_BUSY_STUCK_HIGH_ERR_EN_MSK,
// 				diag.busy_stuck_high_err_en);
// 	val |= no_os_field_prep(AD7606_CLK_FS_OS_COUNTER_EN_MSK,
// 				diag.clk_fs_os_counter_en);
// 	val |= no_os_field_prep(AD7606_INTERFACE_CHECK_EN_MSK, diag.interface_check_en);

// 	ret = ad7606_spi_reg_write(dev, AD7606_REG_DIGITAL_DIAG_ENABLE, val);
// 	if (ret < 0)
// 		return ret;

// 	dev->digital_diag_enable = diag;

// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Initialize the ad7606 device structure.
//  *
//  * Performs memory allocation of the device structure.
//  *
//  * @param device     - Pointer to location of device structure to write.
//  * @param init_param - Pointer to configuration of the driver.
//  *
//  * @return ret - return code.
//  *         Example: -ENOMEM - Memory allocation error.
//  *                  -EIO - SPI communication error.
//  *                  -EIO - GPIO initialization error.
//  *                  -ENODEV - Unexpected device id.
//  *                  -EBADMSG - CRC computation mismatch.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_init(struct ad7606_dev **device,
// 		    struct ad7606_init_param *init_param)
// {
// 	struct ad7606_dev *dev;
// 	uint8_t reg, id;
// 	int32_t i, ret;

// 	no_os_crc8_populate_msb(ad7606_crc8, 0x7);
// 	no_os_crc16_populate_msb(ad7606_crc16, 0x755b);

// 	dev = (struct ad7606_dev *)no_os_calloc(1, sizeof(*dev));
// 	if (!dev)
// 		return -ENOMEM;

// 	dev->device_id = init_param->device_id;
// 	dev->num_channels = ad7606_chip_info_tbl[dev->device_id].num_channels;
// 	dev->max_dout_lines = ad7606_chip_info_tbl[dev->device_id].max_dout_lines;
// 	if (ad7606_chip_info_tbl[dev->device_id].has_registers)
// 		dev->sw_mode = init_param->sw_mode;

// 	ret = ad7606_request_gpios(dev, init_param);
// 	if (ret < 0)
// 		goto error;

// 	if (init_param->sw_mode) {
// 		ret = no_os_gpio_set_value(dev->gpio_os0, NO_OS_GPIO_HIGH);
// 		if (ret < 0)
// 			goto error;

// 		ret = no_os_gpio_set_value(dev->gpio_os1, NO_OS_GPIO_HIGH);
// 		if (ret < 0)
// 			goto error;

// 		ret = no_os_gpio_set_value(dev->gpio_os2, NO_OS_GPIO_HIGH);
// 		if (ret < 0)
// 			goto error;
// 	}

// 	ret = ad7606_reset(dev);
// 	if (ret < 0)
// 		goto error;

// 	/* wait DEVICE_SETUP time */
// 	no_os_udelay(253);

// 	ret = no_os_spi_init(&dev->spi_desc, &init_param->spi_init);
// 	if (ret < 0)
// 		goto error;

// 	if (dev->sw_mode) {
// 		ret = ad7606_spi_reg_read(dev, AD7606_REG_ID, &reg);
// 		if (ret < 0)
// 			goto error;

// 		id = ad7606_chip_info_tbl[dev->device_id].device_id;
// 		if (no_os_field_get(AD7606_ID_DEVICE_ID_MSK, reg) != id) {
// 			printf("ad7606: device id mismatch, expected 0x%.2x, got 0x%.2x\n",
// 			       id,
// 			       (int)no_os_field_get(AD7606_ID_DEVICE_ID_MSK, reg));
// 			ret = -ENODEV;
// 			goto error;
// 		}

// 		ret = ad7606_set_digital_diag(dev, init_param->digital_diag_enable);
// 		if (ret < 0)
// 			goto error;

// 		ret = ad7606_set_config(dev, init_param->config);
// 		if (ret < 0)
// 			goto error;

// 		for (i = 0; i < dev->num_channels; i++) {
// 			ret = ad7606_set_ch_range(dev, i, init_param->range_ch[i]);
// 			if (ret < 0)
// 				goto error;
// 		}

// 		for(i = 0; i < dev->num_channels; i++) {
// 			ret = ad7606_set_ch_offset(dev, i, init_param->offset_ch[i]);
// 			if (ret < 0)
// 				goto error;
// 		}

// 		for(i = 0; i < dev->num_channels; i++) {
// 			ret = ad7606_set_ch_phase(dev, i, init_param->phase_ch[i]);
// 			if (ret < 0)
// 				goto error;
// 		}

// 		for(i = 0; i < dev->num_channels; i++) {
// 			ret = ad7606_set_ch_gain(dev, i, init_param->gain_ch[i]);
// 			if (ret < 0)
// 				goto error;
// 		}
// 	} else {
// 		ret = ad7606_set_ch_range(dev, 0, init_param->range_ch[0]);
// 		if (ret < 0)
// 			goto error;
// 	}

// 	ret = no_os_gpio_set_value(dev->gpio_convst, 1);
// 	if (ret < 0)
// 		goto error;

// 	if (ad7606_chip_info_tbl[dev->device_id].has_oversampling)
// 		ad7606_set_oversampling(dev, init_param->oversampling);

// 	*device = dev;

// 	printf("ad7606 successfully initialized\n");

// 	return ret;
// error:
// 	printf("ad7606 initialization failed\n");
// 	ad7606_remove(dev);
// 	return ret;
// }

// /***************************************************************************//**
//  * @brief Free any resource used by the driver.
//  *
//  * @param dev        - The device structure.
//  *
//  * @return ret - return code.
//  *         Example: -EIO - SPI communication error.
//  *                  0 - No errors encountered.
// *******************************************************************************/
// int32_t ad7606_remove(struct ad7606_dev *dev)
// {
// 	int32_t ret;

// 	no_os_gpio_remove(dev->gpio_reset);
// 	no_os_gpio_remove(dev->gpio_convst);
// 	no_os_gpio_remove(dev->gpio_busy);
// 	no_os_gpio_remove(dev->gpio_stby_n);
// 	no_os_gpio_remove(dev->gpio_range);
// 	no_os_gpio_remove(dev->gpio_os0);
// 	no_os_gpio_remove(dev->gpio_os1);
// 	no_os_gpio_remove(dev->gpio_os2);
// 	no_os_gpio_remove(dev->gpio_par_ser);

// 	ret = no_os_spi_remove(dev->spi_desc);

// 	no_os_free(dev);

// 	return ret;
// }
