#include "task.h"

#include <WiFi.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include "soc/rtc_wdt.h" // 设置看门狗应用

#include "AD7606C.cpp"
#include "PD_UFP.h"
#include "kalman_filter.h"
#include "event.h"
#include "rotary.h"
uint8_t rotary_dir = false;
uint8_t volume = true;
// #Twos Complement Output Coding
// Bipolar Analog Input Ranges
int32_t adc_raw_data[8];
int32_t adc_raw_data_sum_256[8] = {0, 0};
int32_t adc_r_d_avg[8]; // adc raw data average

uint8_t conn_wifi = 0;

const char *ssid = "CandyTime_857112";		 // wifi名
const char *password = "23399693";			 // wifi密码
const IPAddress serverIP(192, 168, 100, 25); // 欲访问的服务端IP地址
uint16_t serverPort = 1234;					 // 服务端口号

KalmanFilter kf_disp(0.00f, 1.0f, 10.0f, 100.0f);

WiFiClient client; // 声明一个ESP32客户端对象，用于与服务器进行连接
AD7606C_Serial AD7606C_18(ADC_CONVST, ADC_BUSY);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
class PD_UFP_c PD_UFP;

void pd_init(void)
{
	PD_UFP.init(PD_POWER_OPTION_MAX_20V);

	while (1)
	{
		PD_UFP.run();
		if (PD_UFP.is_power_ready())
		{
			if (PD_UFP.get_voltage() == PD_V(20.0) && PD_UFP.get_current() >= PD_A(1.5))
			{
				PD_UFP.set_output(1); // Turn on load switch
				break;
				Serial.printf("PD 20V ENABLE Sucess\r\n");
				// PD_UFP.set_led(1);      // Output reach 20V and 1.5A, set indicators on
			}
			else
			{
				PD_UFP.set_output(0); // Turn off load switch
				Serial.printf("DISABLE\r\n");
				break;
				// PD_UFP.blink_led(400);  // Output less than 20V or 1.5A, blink LED
			}
		}
	}
}

void hardware_init(void)
{
	Serial.begin(500000);
	Wire.setClock(1e6);
	// pd_init();

	Serial.printf("\r\n\r\nEVAL-AD7606CFMCZ debug!\r\n");
	Serial.println(getCpuFrequencyMhz());

	delay(1000);
	u8g2.begin();
	u8g2.enableUTF8Print();

	rotary_init(); // 初始化编码器

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

	rtc_wdt_protect_off(); // 看门狗写保护关闭，关闭后可以喂狗
	rtc_wdt_enable();
	rtc_wdt_set_time(RTC_WDT_STAGE0, 3000); // wdt timeout
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

	kf_disp.predict();							  // 预测步骤
	kf_disp.update(BC2V(adc_r_d_avg[1], PN10V0)); // 更新步骤
	sprintf(str, "CH[1]Volt:%2.6fV", kf_disp.get_val());
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

#define WINDOW_SIZE 3

// 毛刺降噪算法
void removeSpikes(int *signal, int length)
{
#define DESPIKES_THRESHOLD 10
	int window[WINDOW_SIZE];
	int i, j, sum;

	for (i = 1; i < length - 1; i++)
	{
		// 构建滑动窗口
		for (j = 0; j < WINDOW_SIZE; j++)
		{
			window[j] = signal[i - 1 + j];
		}
		// 计算窗口内的平均值
		sum = 0;
		for (j = 0; j < WINDOW_SIZE; j++)
		{
			sum += window[j];
		}
		int average = sum / WINDOW_SIZE;

		// 如果当前点与平均值相差较大，则用平均值替代当前值
		if (signal[i] - average > DESPIKES_THRESHOLD || average - signal[i] > DESPIKES_THRESHOLD)
		{
			signal[i] = average;
		}
	}
}
/*
 * @TODO: 去噪点&平滑滤波，not 平均滤波
 *
 */
void xTask_dbgx(void *xTask)
{
	while (1)
	{
		// Serial.print("core[");
		// Serial.print(xTaskGetAffinity(xTask));
		// Serial.printf("]xTask_dbg \r\n");
		Serial.printf("[%6d %6d %6d %6d][%6d %6d %6d %6d]\r\n", adc_raw_data[0], adc_raw_data[1], adc_raw_data[2], adc_raw_data[3], adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);
		vTaskDelay(100);
		// Serial.printf("%2.6f, %2.6f, %2.6f, %2.6f, %d, %d, %d, %d \r\n", BC2V(adc_raw_data[0], PN10V0), BC2V(adc_raw_data[1], PN10V0), BC2V(adc_raw_data[2], PN10V0), BC2V(adc_raw_data[3], PN10V0), adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);
	}
}

void xTask_adcx(void *xTask)
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
		for (uint8_t i = 0; i < 8; i++)
		{
			// adc_norm_data[i] = convert_18bit_to_32bit(adc_raw_data[i]);
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
		// Serial.println("connect server -ing");
		if (client.connect(serverIP, serverPort)) // 连接目标地址
		{
			Serial.println("connect success!");
			// client.print("Hello world!");					 // 向服务器发送数据
			while (client.connected() || client.available()) // 如果已连接或有收到的未读取的数据
			{
				conn_wifi = 1;
				client.printf("%2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f\r\n", BC2V(adc_r_d_avg[0], PN10V0), BC2V(adc_r_d_avg[1], PN10V0), BC2V(adc_r_d_avg[2], PN10V0), BC2V(adc_r_d_avg[3], PN10V0), BC2V(adc_r_d_avg[4], PN10V0), BC2V(adc_r_d_avg[5], PN10V0), BC2V(adc_r_d_avg[6], PN10V0), BC2V(adc_r_d_avg[7], PN10V0));

				// if (client.available()) // 如果有数据可读取
				// {
				// 	String line = client.readStringUntil('\r\n'); // 读取数据到回车换行符
				// 	Serial.print("read:");
				// 	Serial.println(line);
				// 	client.write(line.c_str()); // 将收到的数据回发
				// }
				vTaskDelay(5);
			}
			conn_wifi = 0;
			Serial.println("close clent");
			client.stop(); // 关闭客户端
		}
		else
		{
			conn_wifi = 0;
			// Serial.println("connect fail!");
			client.stop(); // 关闭客户端
		}
		vTaskDelay(1000);
	}
}

void xTask_rotK(void *xTask)
{
	static double rotary;
	static double rotary_hist;
	while (1)
	{
		sys_KeyProcess();
		TimerEventLoop();
		rotary = sys_Counter_Get();

		if (rotary != rotary_hist)
		{
			Serial.printf("rotary:%lf\n", rotary);
		}
		rotary_hist = rotary;

		vTaskDelay(1);
	}
}

/***************************************************use case ****************************************************************/

/* 创建队列，其大小可包含5个元素Data */
// xQueue = xQueueCreate(5, sizeof(Data));
// xTaskCreatePinnedToCore(
//     sendTask, "sendTask", /* 任务名称. */ 10000, /* 任务的堆栈大小 */ NULL, /* 任务的参数 */ 1, /* 任务的优先级 */ &xTask1, /* 跟踪创建的任务的任务句柄 */ 0); /* pin任务到核心0 */
// xTaskCreatePinnedToCore(
//     receiveTask, "receiveTask", 10000, NULL, 1, &xTask2, 1);

// /* 这个变量保持队列句柄 */
// xQueueHandle xQueue;
// TaskHandle_t xTask1;
// TaskHandle_t xTask2;
// /* 保存数据的结构*/
// typedef struct
// {
// 	int sender;
// 	char *msg;
// } Data;

// void sendTask(void *parameter)
// {
// 	/*保持发送数据的状态 */
// 	BaseType_t xStatus;
// 	/* 阻止任务的时间，直到队列有空闲空间 */
// 	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
// 	/* 创建要发送的数据 */
// 	Data data;
// 	/* sender 1的id为1 */
// 	data.sender = 1;
// 	for (;;)
// 	{
// 		Serial.print("sendTask run on core");
// 		/* 获取任务被固定到 */
// 		Serial.print(xTaskGetAffinity(xTask1));
// 		Serial.println("is sending data");
// 		data.msg = (char *)malloc(20);
// 		memset(data.msg, 0, 20);
// 		memcpy(data.msg, "hello world", strlen("hello world"));
// 		/* 将数据发送到队列前面*/
// 		xStatus = xQueueSendToFront(xQueue, &data, xTicksToWait);
// 		/* 检查发送是否正常 */ if (xStatus == pdPASS)
// 		{
// 			/* 增加发送方1 */
// 			Serial.println("sendTask sent data");
// 		}
// 		/* 我们在这里延迟，以便receiveTask有机会接收数据 */
// 		delay(1000);
// 	}
// 	vTaskDelete(NULL);
// }

// void receiveTask(void *parameter)
// {
// 	/*保持接收数据的状态 */
// 	BaseType_t xStatus;
// 	/* 阻止任务的时间，直到数据可用 */
// 	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
// 	Data data;
// 	for (;;)
// 	{
// 		/*从队列接收数据 */
// 		xStatus = xQueueReceive(xQueue, &data, xTicksToWait);
// 		/* 检查接收是否正常 */
// 		if (xStatus == pdPASS)
// 		{
// 			Serial.print("receiveTask run on core ");
// 			/*获取任务固定的核心 */
// 			Serial.print(xTaskGetAffinity(xTask2));
// 			/* 将数据打印到终端*/
// 			Serial.print("got data:");
// 			Serial.print("sender=");
// 			Serial.print(data.sender);
// 			Serial.print("msg=");
// 			Serial.println(data.msg);
// 			free(data.msg);
// 		}
// 	}
// 	vTaskDelete(NULL);
// }