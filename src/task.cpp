#include "task.h"
#include <U8g2lib.h>
#include <WiFi.h>
#include "AD7606C.cpp"
#include "soc/rtc_wdt.h" // 设置看门狗应用

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// #define SPI2_SCLK 14
// #define SPI2_MISO 12 //DOUTA
// #define SPI2_MOSI 13 //SDI
// #define SPI2_CS   15

#define ADC_CONVST 2 //
#define ADC_BUSY 4	 //

// #Twos Complement Output Coding
// Bipolar Analog Input Ranges

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

uint32_t adc_raw_data[8];
uint32_t adc_raw_data_sum_256[8] = {0, 0};
uint32_t adc_r_d_avg[8]; // adc raw data average

uint8_t conn_wifi = 0;

const char *ssid = "CandyTime_857112";		 // wifi名
const char *password = "23399693";			 // wifi密码
const IPAddress serverIP(192, 168, 100, 25); // 欲访问的服务端IP地址
uint16_t serverPort = 1234;					 // 服务端口号

/* 这个变量保持队列句柄 */
xQueueHandle xQueue;
TaskHandle_t xTask1;
TaskHandle_t xTask2;

typedef struct
{
	float x; // 状态估计
	float P; // 状态估计误差协方差
	float Q; // 过程噪声协方差
	float R; // 测量噪声协方差
	float K; // 卡尔曼增益
} KalmanFilter;

KalmanFilter kf;
WiFiClient client; // 声明一个ESP32客户端对象，用于与服务器进行连接
AD7606C_Serial AD7606C_18(ADC_CONVST, ADC_BUSY);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping

// 初始化Kalman Filter
void kalman_filter_init(KalmanFilter *kf, float initial_x, float initial_P, float process_noise, float measurement_noise)
{
	kf->x = initial_x;
	kf->P = initial_P;
	kf->Q = process_noise;
	kf->R = measurement_noise;
}

// 卡尔曼滤波预测步骤
void kalman_predict(KalmanFilter *kf)
{
	// 预测状态
	kf->x = kf->x;
	// 预测误差协方差
	kf->P = kf->P + kf->Q;
}

// 卡尔曼滤波更新步骤
void kalman_update(KalmanFilter *kf, float measurement)
{
	// 计算卡尔曼增益
	kf->K = kf->P / (kf->P + kf->R);

	// 更新状态估计
	kf->x = kf->x + kf->K * (measurement - kf->x);

	// 更新状态估计误差协方差
	kf->P = (1 - kf->K) * kf->P;
}

void hardware_init(void)
{
	Serial.begin(500000);
	Serial.printf("\r\n\r\nEVAL-AD7606CFMCZ debug!\r\n");
	Serial.println(getCpuFrequencyMhz());
	delay(1000);
	u8g2.begin();
	u8g2.enableUTF8Print();
	Wire.setClock(1000000);

	WiFi.mode(WIFI_STA);
	WiFi.setSleep(false); // 关闭STA模式下wifi休眠，提高响应速度
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
	Serial.print("Connected \r\nIP Address:");
	Serial.println(WiFi.localIP());

	AD7606C_18.get_id();
	AD7606C_18.config();
	// AD7606C_18.get_all_reg_val();
	AD7606C_18.debug();

	AD7606C_18.read(adc_raw_data);
	for (uint8_t i = 0; i < 8; i++)
	{
		Serial.printf("[debug]0x%.2d=0x%.8x\r\n", i, adc_raw_data[i]);
	}

	rtc_wdt_protect_off();					// 看门狗写保护关闭，关闭后可以喂狗
	rtc_wdt_enable();						// 启动看门狗
	rtc_wdt_set_time(RTC_WDT_STAGE0, 8000); // 设置看门狗超时 800ms，超时重启
}

void disp_format_data(uint32_t *raw, uint32_t *buf)
{
	uint8_t index = 1;
	uint16_t min, max, range;
	uint16_t num = raw[0];
	min = num;
	max = num;
	do
	{
		num = raw[index];
		if (num < min)
			min = num;
		if (num > max)
			max = num;
	} while (++index < 128);
	index = 0;
	range = max - min;
	if (range > 0)
		do
		{
			num = raw[index];
			buf[index] = ((num - min) << 6) / range;
		} while (++index < 128);
}

uint32_t disp_buf[128];

void draw_curve(uint32_t val)
{
	uint32_t buffer_formated[128];
	disp_buf[127] = val;
	for (uint8_t i = 1; i < 128; i++)
	{
		disp_buf[i - 1] = disp_buf[i];
	}
	u8g2.clearBuffer();
	disp_format_data(disp_buf, buffer_formated);
	for (uint8_t i = 0; i < 127; i++)
	{
		u8g2.drawLine(i, 64 - buffer_formated[i], i + 1, 64 - buffer_formated[i + 1]);
	}
	u8g2.setFont(u8g2_font_wqy14_t_gb2312);

	char str[50];
	kalman_predict(&kf);							  // 预测步骤
	kalman_update(&kf, BC2V(adc_r_d_avg[1], PN10V0)); // 更新步骤
	sprintf(str, "CH[1]Volt:%2.6fV", kf.x);
	u8g2.drawUTF8(1, 63, str);
	u8g2.drawHLine(1, 63, 127);
	u8g2.drawVLine(0, 0, 64);
	u8g2.sendBuffer();
}

#define SUN 0
#define SUN_CLOUD 1
#define CLOUD 2
#define RAIN 3
#define THUNDER 4

void drawWeatherSymbol(u8g2_uint_t x, u8g2_uint_t y, uint8_t symbol)
{
	// fonts used:
	// u8g2_font_open_iconic_embedded_6x_t
	// u8g2_font_open_iconic_weather_6x_t
	// encoding values, see: https://github.com/olikraus/u8g2/wiki/fntgrpiconic

	switch (symbol)
	{
	case SUN:
		u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
		u8g2.drawGlyph(x, y, 69);
		break;
	case SUN_CLOUD:
		u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
		u8g2.drawGlyph(x, y, 65);
		break;
	case CLOUD:
		u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
		u8g2.drawGlyph(x, y, 64);
		break;
	case RAIN:
		u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
		u8g2.drawGlyph(x, y, 67);
		break;
	case THUNDER:
		u8g2.setFont(u8g2_font_open_iconic_embedded_6x_t);
		u8g2.drawGlyph(x, y, 67);
		break;
	}
}

void drawWeather(uint8_t symbol, int degree)
{
	drawWeatherSymbol(0, 48, symbol);
	u8g2.setFont(u8g2_font_logisoso32_tf);
	u8g2.setCursor(48 + 3, 42);
	u8g2.print(degree);
	u8g2.print("°C"); // requires enableUTF8Print()
}

/*
  Draw a string with specified pixel offset.
  The offset can be negative.
  Limitation: The monochrome font with 8 pixel per glyph
*/
void drawScrollString(int16_t offset, const char *s)
{
	static char buf[36]; // should for screen with up to 256 pixel width
	size_t len;
	size_t char_offset = 0;
	u8g2_uint_t dx = 0;
	size_t visible = 0;

	u8g2.setDrawColor(0); // clear the scrolling area
	u8g2.drawBox(0, 49, u8g2.getDisplayWidth() - 1, u8g2.getDisplayHeight() - 1);
	u8g2.setDrawColor(1); // set the color for the text

	len = strlen(s);
	if (offset < 0)
	{
		char_offset = (-offset) / 8;
		dx = offset + char_offset * 8;
		if (char_offset >= u8g2.getDisplayWidth() / 8)
			return;
		visible = u8g2.getDisplayWidth() / 8 - char_offset + 1;
		strncpy(buf, s, visible);
		buf[visible] = '\0';
		u8g2.setFont(u8g2_font_8x13_mf);
		u8g2.drawStr(char_offset * 8 - dx, 62, buf);
	}
	else
	{
		char_offset = offset / 8;
		if (char_offset >= len)
			return; // nothing visible
		dx = offset - char_offset * 8;
		visible = len - char_offset;
		if (visible > u8g2.getDisplayWidth() / 8 + 1)
			visible = u8g2.getDisplayWidth() / 8 + 1;
		strncpy(buf, s + char_offset, visible);
		buf[visible] = '\0';
		u8g2.setFont(u8g2_font_8x13_mf);
		u8g2.drawStr(-dx, 62, buf);
	}
}

void draw(const char *s, uint8_t symbol, int degree)
{
	int16_t offset = -(int16_t)u8g2.getDisplayWidth();
	int16_t len = strlen(s);

	u8g2.clearBuffer();			 // clear the internal memory
	drawWeather(symbol, degree); // draw the icon and degree only once
	for (;;)					 // then do the scrolling
	{

		drawScrollString(offset, s); // no clearBuffer required, screen will be partially cleared here
		u8g2.sendBuffer();			 // transfer internal memory to the display

		delay(20);
		offset += 2;
		if (offset > len * 8 + 1)
			break;
	}
}

void xTask_oled(void *xTask)
{
	float initial_x = 0.0;			 // 初始状态估计
	float initial_P = 1.0;			 // 初始状态估计误差协方差
	float process_noise = 10.0;		 // 过程噪声协方差
	float measurement_noise = 100.0; // 测量噪声协方差

	// 初始化Kalman Filter
	kalman_filter_init(&kf, initial_x, initial_P, process_noise, measurement_noise);
	while (1)
	{
		// Serial.print("core[");
		// Serial.print(xTaskGetAffinity(xTask1));
		// Serial.printf("]xTask_oled \r\n");
		// draw("What a beautiful day!", SUN, 27);
		// draw("The sun's come out!", SUN_CLOUD, 19);
		// draw("It's raining cats and dogs.", RAIN, 8);
		// draw("That sounds like thunder.", THUNDER, 12);
		// draw("It's stopped raining", CLOUD, 15);
		draw_curve(adc_r_d_avg[0]);
		vTaskDelay(100);
	}
}

/*
 * @TODO: 去噪点&平滑滤波，not 平均滤波
 *
 */
void xTask_dbg(void *xTask)
{
	while (1)
	{
		// Serial.print("core[");
		// Serial.print(xTaskGetAffinity(xTask));
		// Serial.printf("]xTask_dbg \r\n");
		// Serial.printf("[%6d %6d %6d]%.6f %.6f\r\n", adc_raw_data[0], adc_r_d_avg[0], adc_r_d_avg[3], BC2V(adc_r_d_avg[0], PN10V0), BC2V(adc_r_d_avg[1], PN10V0));
		vTaskDelay(100);
		Serial.printf("%2.6f, %2.6f, %2.6f, %2.6f, %d, %d, %d, %d \r\n", BC2V(adc_r_d_avg[0], PN10V0), BC2V(adc_r_d_avg[1], PN10V0), BC2V(adc_r_d_avg[2], PN10V0), BC2V(adc_r_d_avg[3], PN10V0), adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);
	}
}

void xTask_adc(void *xTask)
{
	static uint16_t cnt;
	while (1)
	{
		// Serial.print("core[");
		// Serial.print(xTaskGetAffinity(xTask));
		// Serial.printf("]xTask_adc \r\n");
		// Serial.print("tsk_adc: ");
		// Serial.println(millis());
		AD7606C_18.fast_read(adc_raw_data);
		adc_raw_data[3] = 262143;
		for (uint8_t i = 0; i < 8; i++)
		{
			adc_raw_data_sum_256[i] += adc_raw_data[i];
		}
		if (++cnt >= 256)
		{
			cnt = 0;
			for (uint8_t i = 0; i < 8; i++)
			{
				adc_r_d_avg[i] = adc_raw_data_sum_256[i] >> 8;
				adc_raw_data_sum_256[i] = 0;
			}
		}
		delayMicroseconds(730);
	}
}

void xTask_wifi(void *xTask)
{
	while (1)
	{
		Serial.println("connect server -ing");
		if (client.connect(serverIP, serverPort)) // 连接目标地址
		{
			Serial.println("connect success!");
			client.print("Hello world!");					 // 向服务器发送数据
			while (client.connected() || client.available()) // 如果已连接或有收到的未读取的数据
			{
				conn_wifi = 1;
				client.printf("%2.6f, %2.6f, %2.6f, %2.6f, %d, %d, %d, %d \r\n", BC2V(adc_raw_data[0], PN10V0), BC2V(adc_raw_data[1], PN10V0), BC2V(adc_raw_data[2], PN10V0), BC2V(adc_raw_data[3], PN10V0), adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);

				// if (client.available()) // 如果有数据可读取
				// {
				// 	String line = client.readStringUntil('\r\n'); // 读取数据到回车换行符
				// 	Serial.print("read:");
				// 	Serial.println(line);
				// 	client.write(line.c_str()); // 将收到的数据回发
				// }
				vTaskDelay(100);
			}
			conn_wifi = 0;
			Serial.println("close clent");
			client.stop(); // 关闭客户端
		}
		else
		{
			conn_wifi = 0;
			Serial.println("connect fail!");
			client.stop(); // 关闭客户端
		}
		vTaskDelay(1000);
	}
}

/* 保存数据的结构*/
typedef struct
{
	int sender;
	char *msg;
} Data;

void sendTask(void *parameter)
{
	/*保持发送数据的状态 */
	BaseType_t xStatus;
	/* 阻止任务的时间，直到队列有空闲空间 */
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	/* 创建要发送的数据 */
	Data data;
	/* sender 1的id为1 */
	data.sender = 1;
	for (;;)
	{
		Serial.print("sendTask run on core");
		/* 获取任务被固定到 */
		Serial.print(xTaskGetAffinity(xTask1));
		Serial.println("is sending data");
		data.msg = (char *)malloc(20);
		memset(data.msg, 0, 20);
		memcpy(data.msg, "hello world", strlen("hello world"));
		/* 将数据发送到队列前面*/
		xStatus = xQueueSendToFront(xQueue, &data, xTicksToWait);
		/* 检查发送是否正常 */ if (xStatus == pdPASS)
		{
			/* 增加发送方1 */
			Serial.println("sendTask sent data");
		}
		/* 我们在这里延迟，以便receiveTask有机会接收数据 */
		delay(1000);
	}
	vTaskDelete(NULL);
}

void receiveTask(void *parameter)
{
	/*保持接收数据的状态 */
	BaseType_t xStatus;
	/* 阻止任务的时间，直到数据可用 */
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	Data data;
	for (;;)
	{
		/*从队列接收数据 */
		xStatus = xQueueReceive(xQueue, &data, xTicksToWait);
		/* 检查接收是否正常 */
		if (xStatus == pdPASS)
		{
			Serial.print("receiveTask run on core ");
			/*获取任务固定的核心 */
			Serial.print(xTaskGetAffinity(xTask2));
			/* 将数据打印到终端*/
			Serial.print("got data:");
			Serial.print("sender=");
			Serial.print(data.sender);
			Serial.print("msg=");
			Serial.println(data.msg);
			free(data.msg);
		}
	}
	vTaskDelete(NULL);
}