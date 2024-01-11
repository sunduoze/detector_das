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
#include "menu.h"

uint8_t *C_table[] = {c1, c2, c3, Lightning, c5, c6, c7};
uint8_t rotary_dir = false;
uint8_t volume = true;

int32_t adc_raw_data[ADC_ALL_CH];
float adc_disp_val[ADC_ALL_CH];

uint8_t conn_wifi = 0;
adc_calibration_ adc_cali;

char ssid[] = "xxxxx";		  // wifi名
char password[] = "xxxxx";	  // wifi密码
char ssid_bk[] = "xxxxx";	  // wifi名
char password_bk[] = "xxxxx"; // wifi密码

const IPAddress serverIP(192, 168, 1, 1); // 欲访问的服务端IP地址
uint16_t serverPort = 1234;				  // 服务端口号
uint64_t ChipMAC;
char ChipMAC_S[19] = {0};
char CompileTime[20];

KalmanFilter kf_disp(0.00f, 1.0f, 10.0f, 100.0f);
KalmanFilter kf_main_ui(0.00f, 1.0f, 100.0f, 20000.0f);

WiFiClient client; // 声明一个ESP32客户端对象，用于与服务器进行连接
AD7606C_Serial AD7606C_18(ADC_CONVST, ADC_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
AD5272 digital_pot(POT_ADDR_NC);
PD_UFP_c PD_UFP;

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
			}
			else
			{
				PD_UFP.set_output(0); // Turn off load switch
				Serial.printf("DISABLE\r\n");
				ret = 1;
				break;
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
	adc_cali.adc_gain_ch[ADC_CH1] = 1.0f;
	adc_cali.adc_offset_ch[ADC_CH1] = -0.017699; //-0.017547;

	adc_cali.adc_gain_ch[ADC_CH2] = 0.99664f;	 // 未校准
	adc_cali.adc_offset_ch[ADC_CH2] = -0.017547; // 未校准

	adc_cali.adc_gain_ch[ADC_CH3] = 0.997993; // 0.990973;
	adc_cali.adc_offset_ch[ADC_CH3] = -0.01098;

	adc_cali.adc_gain_ch[ADC_CH4] = 1.000207f;
	adc_cali.adc_offset_ch[ADC_CH4] = -0.00011;

	adc_cali.adc_gain_ch[ADC_CH5] = 1.000132f;
	adc_cali.adc_offset_ch[ADC_CH5] = 0.0;

	adc_cali.adc_gain_ch[ADC_CH6] = 0.999882f;
	adc_cali.adc_offset_ch[ADC_CH6] = 0.0;

	adc_cali.adc_gain_ch[ADC_CH7] = 5.0f * 1.000188f;
	adc_cali.adc_offset_ch[ADC_CH7] = 0.0;

	adc_cali.adc_gain_ch[ADC_CH8] = -2.0f * 1.000188f; // 未校准增益 curr = volt / 100 / 0.005R
	adc_cali.adc_offset_ch[ADC_CH8] = 0.00605;

	AD7606C_18.config();
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
	Wire.begin(SDA, SCL, 1e5);
	int ret = digital_pot.set_res_val(0);
	if (ret != 0) // check if data is sent successfully
		Serial.println("[Error]digital_pot init !");
	Wire.begin(SDA, SCL, 1e6);
}

void hardware_init(void)
{
	beep_init();
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // 关闭断电检测
	Serial.begin(5e5);
	Wire.begin(SDA, SCL, 4e5);
	if (pd_init() == 0)
	{
		pd_status = true;
	}
	// FilesSystemInit(); // 启动文件系统，并读取存档 **bug***
	Serial.printf("\r\n\r\nDetector DAS based on EVAL-AD7606CFMCZ !\r\n");
	oled_init();

	disp_boot_info();
	delay(500);
	enter_logo();

	if (digital_pot.init() != 0)
	{
		Serial.println("Cannot send data to the digital_pot.");
	}
	ad527x_init();
	rotary_init(); // 初始化编码器
	adc_init();
	// ble_phyphox_init();
	rtc_wdt_protect_off(); // 看门狗写保护关闭
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

/*
 *
 */
void xTask_dbgx(void *xTask)
{
	// Serial.print("core[");
	// Serial.print(xTaskGetAffinity(xTask));
	// Serial.printf("]xTask_dbg \r\n");
	while (1)
	{
		// Serial.printf("[%6d %6d %6d %6d][%6d %6d %6d %.6f]\r\n", adc_raw_data[0], adc_raw_data[1], adc_raw_data[2], adc_raw_data[3], adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], C2V(adc_r_d_avg[ADC_CH8], PN5V00));
		vTaskDelay(100);
	}
}

void xTask_adcx(void *xTask)
{
	static uint16_t cnt;
	// Serial.print("core[");
	// Serial.print(xTaskGetAffinity(xTask));
	// Serial.printf("]xTask_adc \r\n");
	// Serial.print("tsk_adc: ");
	// Serial.println(millis());
	while (1)
	{
		AD7606C_18.fast_read(adc_raw_data);
		adc_disp_val[ADC_CH1] = (C2V(adc_raw_data[ADC_CH1], PN20V0) + adc_cali.adc_offset_ch[ADC_CH1]) * adc_cali.adc_gain_ch[ADC_CH1];
		adc_disp_val[ADC_CH2] = (C2V(adc_raw_data[ADC_CH2], PN20V0) + adc_cali.adc_offset_ch[ADC_CH2]) * adc_cali.adc_gain_ch[ADC_CH2];
		adc_disp_val[ADC_CH3] = (C2V(adc_raw_data[ADC_CH3], PN20V0) + adc_cali.adc_offset_ch[ADC_CH3]) * adc_cali.adc_gain_ch[ADC_CH3];
		adc_disp_val[ADC_CH4] = (C2V(adc_raw_data[ADC_CH4], PP5V00) + adc_cali.adc_offset_ch[ADC_CH4]) * adc_cali.adc_gain_ch[ADC_CH4];
		adc_disp_val[ADC_CH5] = (C2V(adc_raw_data[ADC_CH5], PP5V00) + adc_cali.adc_offset_ch[ADC_CH5]) * adc_cali.adc_gain_ch[ADC_CH5];
		adc_disp_val[ADC_CH6] = (C2V(adc_raw_data[ADC_CH6], PP10V0) + adc_cali.adc_offset_ch[ADC_CH6]) * adc_cali.adc_gain_ch[ADC_CH6];
		adc_disp_val[ADC_CH7] = (C2V(adc_raw_data[ADC_CH7], PP5V00) + adc_cali.adc_offset_ch[ADC_CH7]) * adc_cali.adc_gain_ch[ADC_CH7];
		adc_disp_val[ADC_CH8] = (C2V(adc_raw_data[ADC_CH8], PN5V00) + adc_cali.adc_offset_ch[ADC_CH8]) * adc_cali.adc_gain_ch[ADC_CH8];

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
							  adc_disp_val[ADC_CH1], adc_disp_val[ADC_CH2],
							  adc_disp_val[ADC_CH3], adc_disp_val[ADC_CH4],
							  adc_disp_val[ADC_CH5], adc_disp_val[ADC_CH6],
							  adc_disp_val[ADC_CH7], adc_disp_val[ADC_CH8]);
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
		// data0 = C2V(adc_r_d_avg[0], PN10V0);
		// data3 = C2V(adc_r_d_avg[3], PN10V0);
		// data4 = C2V(adc_r_d_avg[4], PN10V0);
		PhyphoxBLE::write(time, data0, data3, data4);
		vTaskDelay(100);
		PhyphoxBLE::poll(); // Only required for the Arduino Nano 33 IoT, but it does no harm for other boards.
	}
}

float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
	const float run = in_max - in_min;
	if (run == 0)
	{
		log_e("map(): Invalid input range, min == max");
		return -1; // AVR returns -1, SAM returns 0
	}
	const float rise = out_max - out_min;
	const float delta = x - in_min;
	return (delta * rise) / run + out_min;
}

void i2c_dev_scan()
{
	Wire.begin(SDA, SCL, 1e3);
	Serial.begin(5e5);
	uint8_t error, address;
	int nDevices;
	Serial.println("Scanning...");
	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println(" !");
			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
	delay(5000); // wait 5 seconds for next scan
}
