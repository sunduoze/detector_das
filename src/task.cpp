#include "task.h"

#include <WiFi.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include "soc/rtc_wdt.h" // 设置看门狗应用

#include "PD_UFP.h"
#include "kalman_filter.h"
#include "event.h"
#include "rotary.h"
#include "beep.h"
#include "ad5272.h"

uint8_t *C_table[] = {c1, c2, c3, Lightning, c5, c6, c7};

uint8_t rotary_dir = false;
uint8_t volume = true;
// #Twos Complement Output Coding
// Bipolar Analog Input Ranges
int32_t adc_raw_data[ADC_ALL_CH];
int32_t adc_raw_data_sum_256[ADC_ALL_CH] = {0, 0};
int32_t adc_r_d_avg[ADC_ALL_CH]; // adc raw data average
float adc_disp_val[ADC_ALL_CH];

uint8_t conn_wifi = 0;

adc_calibration_ adc_cali;

char ssid[] = "CandyTime_857112"; // wifi名
char password[] = "23399693";	  // wifi密码
char ssid_bk[] = "GBA";			  // wifi名
char password_bk[] = "XTyjy8888"; // wifi密码

const IPAddress serverIP(192, 168, 100, 25); // 欲访问的服务端IP地址
uint16_t serverPort = 1234;					 // 服务端口号

KalmanFilter kf_disp(0.00f, 1.0f, 10.0f, 100.0f);
KalmanFilter kf_main_ui(0.00f, 1.0f, 100.0f, 20000.0f);

WiFiClient client; // 声明一个ESP32客户端对象，用于与服务器进行连接
AD7606C_Serial AD7606C_18(ADC_CONVST, ADC_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
class PD_UFP_c PD_UFP;

#define POT_ADDR 0x2E
AD5272 digital_pot(POT_ADDR); // creates communication object with address 0x2E

uint8_t pd_init(void)
{
	uint8_t ret = 0;
	uint16_t timeout = 2000;
	PD_UFP.init(PD_POWER_OPTION_MAX_20V);
	Serial.printf("PD_UFP.init-ing\r\n");
	while (1)
	{
		PD_UFP.run();
		if (PD_UFP.is_power_ready())
		{
			if (PD_UFP.get_voltage() == PD_V(20.0) && PD_UFP.get_current() >= PD_A(1.5))
			{
				PD_UFP.set_output(1); // Turn on load switch
				Serial.printf("PD 20V ENABLE Sucess\r\n");
				SetSound(BootSound); // 播放音效
				break;
				// PD_UFP.set_led(1);      // Output reach 20V and 1.5A, set indicators on
			}
			else
			{
				PD_UFP.set_output(0); // Turn off load switch
				Serial.printf("DISABLE\r\n");
				ret = 1;
				break;
				// PD_UFP.blink_led(400);  // Output less than 20V or 1.5A, blink LED
			}
		}
		if (timeout-- <= 1)
		{
			Serial.printf("PD power not ready\r\n");
			ret = 2;
			break;
		}
		vTaskDelay(1);
	}
	return ret;
}

uint8_t wifi_init()
{
	uint8_t ret = 0;
	WiFi.mode(WIFI_STA);
	WiFi.setSleep(false); // 关闭STA模式下wifi休眠，提高响应速度
	WiFi.begin(ssid, password);
	uint8_t i = 100;
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(200);
		Serial.print(".");
		if (i-- == 50)
		{
			Serial.printf("Connecting to %s\r\n", ssid_bk);
			WiFi.begin(ssid_bk, password_bk);
			Serial.println("WiFi connected");
		}
		if (i <= 1)
		{
			Serial.print("\r\nWifi connect failed\r\n");
			ret = 1;
			break;
		}
	}

	if (ret == 0)
	{
		if (i > 50)
		{
			Serial.printf("Connecting to %s\r\n", ssid);
		}
		Serial.println("IP address: ");
		Serial.println(WiFi.localIP());
		Serial.println("MAC address: ");
		Serial.println(WiFi.macAddress());
		Serial.println("DNS address: ");
		Serial.println(WiFi.dnsIP());
		Serial.println("Gateway address: ");
		Serial.println(WiFi.gatewayIP());
		Serial.println("Subnet mask: ");
		Serial.println(WiFi.subnetMask());
	}
	return ret;
}

void adc_init()
{
	for (uint8_t i = 0; i < ADC_ALL_CH; i++)
	{
		adc_cali.adc_gain_ch[i] = 1.0f;
		adc_cali.adc_offset_ch[i] = 0.0f;
	}
	adc_cali.adc_gain_ch[ADC_CH7] = 5.0f;
	adc_cali.adc_gain_ch[ADC_CH8] = -2.0f; // curr = volt / 100 / 0.005R

	AD7606C_18.config();
	// AD7606C_18.get_all_reg_val();

	AD7606C_18.read(adc_raw_data);
	for (uint8_t i = 0; i < 8; i++)
	{
		Serial.printf("[debug]0x%.2d=0x%.8x\r\n", i, adc_raw_data[i]);
	}
}

void oled_init()
{
	oled.begin();
	oled.setBusClock(4e5);
	oled.enableUTF8Print();
	oled.setFontDirection(0);
	oled.setFontPosTop();
	oled.setFont(u8g2_font_wqy12_t_gb2312);
	oled.setDrawColor(1);
	oled.setFontMode(1);
}

void ad527x_init()
{
	// read the current RDAC value
	int ret = digital_pot.read_rdac();
	Serial.print("Read RDAC Value: ");
	Serial.println(ret, DEC);

	// set new value to RDAC (0~1024)
	uint16_t data = 102;
	ret = digital_pot.write_data(AD5272_RDAC_WRITE, data);
	if (ret != 0) // check if data is sent successfully
		Serial.println("Error!");

	// // copy RDAC value to 50TP memory
	// ret = digital_pot.write_data(AD5272_50TP_WRITE, 0);
	// if (ret != 0) // check if data is sent successfully
	// 	Serial.println("Error!");

	// read the new RDAC value
	ret = digital_pot.read_rdac();
	Serial.print("New RDAC Value: ");
	Serial.println(ret, DEC);
}

uint64_t ChipMAC;
char ChipMAC_S[19] = {0};
char CompileTime[20];
void hardware_init(void)
{
	beep_init();
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // 关闭断电检测
	Serial.begin(5e5);
	ChipMAC = ESP.getEfuseMac();
	sprintf(CompileTime, "%s %s", __DATE__, __TIME__);
	for (uint8_t i = 0; i < 6; i++)
		sprintf(ChipMAC_S + i * 3, "%02X%s", ((uint8_t *)&ChipMAC)[i], (i != 5) ? ":" : "");

	Wire.begin(SDA, SCL, 1e6);
	if (pd_init() == 0)
	{
		pd_status = true;
	}
	// FilesSystemInit(); // 启动文件系统，并读取存档 **bug***

	Serial.printf("\r\n\r\nDetector DAS based on EVAL-AD7606CFMCZ !\r\n");

	oled_init();
	ShowBootMsg();

	if (digital_pot.init() != 0)
	{
		Serial.println("Cannot send data to the IC.");
	}
	ad527x_init();

	rotary_init(); // 初始化编码器

	adc_init();
	// ble_phyphox_init();
	rtc_wdt_protect_off(); // 看门狗写保护关闭，关闭后可以喂狗
	rtc_wdt_enable();
	rtc_wdt_set_time(RTC_WDT_STAGE0, 3000); // wdt timeout

	get_sin();
}

uint8_t sin_tab[SIN_TB_SIZE];
void get_sin(void)
{
	const uint16_t uPoints = SIN_TB_SIZE;
	float x, uAng;
	uAng = 360.000 / uPoints;
	for (int i = 0; i < uPoints; i++)
	{
		x = uAng * i;
		x = x * (3.14159 / 180); // 弧度=角度*（π/180）
		sin_tab[i] = (255 / 2) * sin(x) + (255 / 2);
		// printf("sin tab[%d]: %f\n", i, sin_tab[i]);
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

		// draw_curve(adc_r_d_avg[ADC_CH3]);

		vTaskDelay(100);
	}
}

#define WINDOW_SIZE 3

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
		adc_disp_val[ADC_CH1] = BC2V(adc_r_d_avg[ADC_CH1], PN20V0) * adc_cali.adc_gain_ch[ADC_CH1] + adc_cali.adc_offset_ch[ADC_CH1];
		adc_disp_val[ADC_CH2] = BC2V(adc_r_d_avg[ADC_CH2], PN20V0) * adc_cali.adc_gain_ch[ADC_CH2] + adc_cali.adc_offset_ch[ADC_CH2];
		adc_disp_val[ADC_CH3] = BC2V(adc_r_d_avg[ADC_CH3], PN20V0) * adc_cali.adc_gain_ch[ADC_CH3] + adc_cali.adc_offset_ch[ADC_CH3];
		adc_disp_val[ADC_CH4] = BC2V(adc_r_d_avg[ADC_CH4], PP5V00) * adc_cali.adc_gain_ch[ADC_CH4] + adc_cali.adc_offset_ch[ADC_CH4];
		adc_disp_val[ADC_CH5] = BC2V(adc_r_d_avg[ADC_CH5], PP5V00) * adc_cali.adc_gain_ch[ADC_CH5] + adc_cali.adc_offset_ch[ADC_CH5];
		adc_disp_val[ADC_CH6] = BC2V(adc_r_d_avg[ADC_CH6], PP10V0) * adc_cali.adc_gain_ch[ADC_CH6] + adc_cali.adc_offset_ch[ADC_CH6];
		adc_disp_val[ADC_CH7] = BC2V(adc_r_d_avg[ADC_CH7], PP5V00) * adc_cali.adc_gain_ch[ADC_CH7] + adc_cali.adc_offset_ch[ADC_CH7];
		adc_disp_val[ADC_CH8] = BC2V(adc_r_d_avg[ADC_CH8], PN5V00) * adc_cali.adc_gain_ch[ADC_CH8] + adc_cali.adc_offset_ch[ADC_CH8];

		// Serial.printf("[%6d %6d %6d %6d][%6d %6d %6d %.6f]\r\n", adc_raw_data[0], adc_raw_data[1], adc_raw_data[2], adc_raw_data[3], adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], BC2V(adc_r_d_avg[ADC_CH8], PN5V00));
		vTaskDelay(100);
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
	if (wifi_init() == 0)
	{
		wifi_status = true;
	}
	while (1)
	{
		// Serial.println("connect server -ing");
		if (client.connect(serverIP, serverPort)) // 连接目标地址1
		{
			Serial.println("connect success!");
			// client.print("Hello world!");					 // 向服务器发送数据
			while (client.connected() || client.available()) // 如果已连接或有收到的未读取的数据
			{
				conn_wifi = 1;
				client.printf("%2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f\r\n",
							  BC2V(adc_r_d_avg[ADC_CH1], PN20V0), BC2V(adc_r_d_avg[ADC_CH2], PN20V0),
							  UC2V(adc_r_d_avg[ADC_CH3], PN20V0), BC2V(adc_r_d_avg[ADC_CH4], PP5V00),
							  BC2V(adc_r_d_avg[ADC_CH5], PP5V00), BC2V(adc_r_d_avg[ADC_CH6], PP10V0),
							  BC2V(adc_r_d_avg[ADC_CH7], PP5V00) * adc_cali.adc_gain_ch[ADC_CH7] + adc_cali.adc_offset_ch[ADC_CH7],
							  BC2V(adc_r_d_avg[ADC_CH8], PN5V00) * adc_cali.adc_gain_ch[ADC_CH8] + adc_cali.adc_offset_ch[ADC_CH8]);

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
			// Serial.println("connect fail!");
			client.stop(); // 关闭客户端
		}
		vTaskDelay(1000);
	}
}

#include "menu.h"

void xTask_rotK(void *xTask)
{
	static double rotary;
	static double rotary_hist;
	while (1)
	{
		sys_KeyProcess();
		// TimerEventLoop();
		rotary = sys_Counter_Get();
#ifdef ROTARY_DEBUG
		if (rotary != rotary_hist)
		{
			Serial.printf("rotary:%lf\n", rotary);
		}
#endif // ROTARY_DEBUG

		rotary_hist = rotary;
		// 刷新UI
		System_UI();
		vTaskDelay(1);
	}
}

void xTask_test(void *xTask)
{
	// 获取按键
	sys_KeyProcess();
	// Serial.printf("Temp:%.6fmV,%.6fmV\r\n", analogRead(TIP_ADC_PIN) / 4096.0 * 3300, analogRead(CUR_ADC_PIN) / 4096.0 * 3300);
	// if (!Menu_System_State)
	// {
	// 温度闭环控制
	// TemperatureControlLoop();
	// 更新系统事件：：系统事件可能会改变功率输出
	// TimerEventLoop();
	// }
	// 更新状态码
	SYS_StateCode_Update();

	// 刷新UI
	//  System_UI();
}

void ble_phyphox_init()
{
	PhyphoxBLE::start("Detector-DAS");
	PhyphoxBleExperiment MultiGraph;
	MultiGraph.setTitle("Detector-DAS");
	MultiGraph.setCategory("DigiKey创意大赛:多通道微型气相色谱采集单元");
	MultiGraph.setDescription("基于AD7606C-18的多通道检测器数据采集系统");
	PhyphoxBleExperiment::View firstView;
	firstView.setLabel("FirstView"); // Create a "view"
	PhyphoxBleExperiment::Graph detector_data_graph;
	PhyphoxBleExperiment::Graph::Subgraph pid_data;
	pid_data.setChannel(1, 2);
	pid_data.setColor("ff00ff");
	pid_data.setStyle(STYLE_LINES);
	pid_data.setLinewidth(2);
	detector_data_graph.addSubgraph(pid_data);
	PhyphoxBleExperiment::Graph::Subgraph did_data;
	did_data.setChannel(1, 3);
	did_data.setColor("0000ff");
	did_data.setStyle(STYLE_LINES);
	did_data.setLinewidth(2);
	detector_data_graph.addSubgraph(did_data);
	PhyphoxBleExperiment::Graph::Subgraph tcd_data;
	tcd_data.setChannel(1, 4);
	tcd_data.setColor("0ee0ff");
	tcd_data.setStyle(STYLE_LINES);
	tcd_data.setLinewidth(2);
	detector_data_graph.addSubgraph(tcd_data);

	firstView.addElement(detector_data_graph);
	MultiGraph.addView(firstView);
	PhyphoxBLE::addExperiment(MultiGraph);

	// PhyphoxBLE::printXML(&Serial);
}

float periodTime2 = 2.0; // in s
float generateSin2(float x)
{
	return 1.0 * sin(x * 2.0 * PI / periodTime2);
}
void xTask_blex(void *xTask)
{

	float currentTime = millis() / 1000.0;
	float sinus = generateSin2(currentTime);
	float cosinus = generateSin2(currentTime + 0.5 * periodTime2);
	PhyphoxBLE::write(currentTime, sinus, cosinus, cosinus);
	delay(100);
	// PhyphoxBLE::poll(); //Only required for the Arduino Nano 33 IoT, but it does no harm for other boards.
}

void xTask_blex2(void *xTask)
{
	float time;
	float data0, data3, data4;
	while (1)
	{
		time = millis() / 1000.0;
		data0 = BC2V(adc_r_d_avg[0], PN10V0);
		data3 = BC2V(adc_r_d_avg[3], PN10V0);
		data4 = BC2V(adc_r_d_avg[4], PN10V0);
		PhyphoxBLE::write(time, data0, data3, data4);
		vTaskDelay(100);
		PhyphoxBLE::poll(); // Only required for the Arduino Nano 33 IoT, but it does no harm for other boards.
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
		oled.setFont(u8g2_font_open_iconic_weather_6x_t);
		oled.drawGlyph(x, y, 69);
		break;
	case SUN_CLOUD:
		oled.setFont(u8g2_font_open_iconic_weather_6x_t);
		oled.drawGlyph(x, y, 65);
		break;
	case CLOUD:
		oled.setFont(u8g2_font_open_iconic_weather_6x_t);
		oled.drawGlyph(x, y, 64);
		break;
	case RAIN:
		oled.setFont(u8g2_font_open_iconic_weather_6x_t);
		oled.drawGlyph(x, y, 67);
		break;
	case THUNDER:
		oled.setFont(u8g2_font_open_iconic_embedded_6x_t);
		oled.drawGlyph(x, y, 67);
		break;
	}
}

void drawWeather(uint8_t symbol, int degree)
{
	drawWeatherSymbol(0, 48, symbol);
	oled.setFont(u8g2_font_logisoso32_tf);
	oled.setCursor(48 + 3, 42);
	oled.print(degree);
	oled.print("°C"); // requires enableUTF8Print()
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

	oled.setDrawColor(0); // clear the scrolling area
	oled.drawBox(0, 49, oled.getDisplayWidth() - 1, oled.getDisplayHeight() - 1);
	oled.setDrawColor(1); // set the color for the text

	len = strlen(s);
	if (offset < 0)
	{
		char_offset = (-offset) / 8;
		dx = offset + char_offset * 8;
		if (char_offset >= oled.getDisplayWidth() / 8)
			return;
		visible = oled.getDisplayWidth() / 8 - char_offset + 1;
		strncpy(buf, s, visible);
		buf[visible] = '\0';
		oled.setFont(u8g2_font_8x13_mf);
		oled.drawStr(char_offset * 8 - dx, 62, buf);
	}
	else
	{
		char_offset = offset / 8;
		if (char_offset >= len)
			return; // nothing visible
		dx = offset - char_offset * 8;
		visible = len - char_offset;
		if (visible > oled.getDisplayWidth() / 8 + 1)
			visible = oled.getDisplayWidth() / 8 + 1;
		strncpy(buf, s + char_offset, visible);
		buf[visible] = '\0';
		oled.setFont(u8g2_font_8x13_mf);
		oled.drawStr(-dx, 62, buf);
	}
}

void draw(const char *s, uint8_t symbol, int degree)
{
	int16_t offset = -(int16_t)oled.getDisplayWidth();
	int16_t len = strlen(s);

	oled.clearBuffer();			 // clear the internal memory
	drawWeather(symbol, degree); // draw the icon and degree only once
	for (;;)					 // then do the scrolling
	{

		drawScrollString(offset, s); // no clearBuffer required, screen will be partially cleared here
		oled.sendBuffer();			 // transfer internal memory to the display

		delay(20);
		offset += 2;
		if (offset > len * 8 + 1)
			break;
	}
}
