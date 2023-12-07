#include <Arduino.h>
// #include <esp_heap_caps.h>

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
// #include <TaskScheduler.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include "AD7606C.cpp"

#include "soc/rtc_wdt.h" // 设置看门狗应用
/* 保存数据的结构*/
typedef struct
{
  int sender;
  char *msg;
} Data;

/* 这个变量保持队列句柄 */
xQueueHandle xQueue;
TaskHandle_t xTask1;
TaskHandle_t xTask2;

// #define SPI2_SCLK 14
// #define SPI2_MISO 12 //DOUTA
// #define SPI2_MOSI 13 //SDI
// #define SPI2_CS   15

#define ADC_CONVST 2 //
#define ADC_BUSY 4   //

AD7606C_Serial AD7606C_18(ADC_CONVST, ADC_BUSY);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping

uint32_t adc_raw_data[8];
uint32_t adc_raw_data_sum_256[8] = {0, 0};
uint32_t adc_raw_data_avg[8];

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
#define BC2V(code, range_lsb) (((code - 1) & 0x20000) == 0x20000) ? (((code)-0x40000) * range_lsb) : ((code) * range_lsb)
/*UNIPOLAR_CODE_TO_VOLT*/
#define UC2V(code) (code) * range_lsb

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

  u8g2.clearBuffer();          // clear the internal memory
  drawWeather(symbol, degree); // draw the icon and degree only once
  for (;;)                     // then do the scrolling
  {

    drawScrollString(offset, s); // no clearBuffer required, screen will be partially cleared here
    u8g2.sendBuffer();           // transfer internal memory to the display

    delay(20);
    offset += 2;
    if (offset > len * 8 + 1)
      break;
  }
}

void hardware_init(void)
{
  Serial.begin(500000);
  Serial.printf("EVAL-AD7606CFMCZ debug!\r\n");
  Serial.println(getCpuFrequencyMhz());
  delay(1000);
  u8g2.begin();
  u8g2.enableUTF8Print();
  Wire.setClock(1000000);

  AD7606C_18.get_id();
  AD7606C_18.config();
  // AD7606C_18.get_all_reg_val();
  AD7606C_18.debug();

  AD7606C_18.read(adc_raw_data);
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.printf("[debug]0x%.2d=0x%.8x\r\n", i, adc_raw_data[i]);
  }

  rtc_wdt_protect_off();                  // 看门狗写保护关闭，关闭后可以喂狗
  rtc_wdt_enable();                       // 启动看门狗
  rtc_wdt_set_time(RTC_WDT_STAGE0, 8000); // 设置看门狗超时 800ms，超时重启
}

void xTask_oled(void *xTask)
{
  while (1)
  {
    // Serial.print("core[");
    // /* 获取任务被固定到 */
    // Serial.print(xTaskGetAffinity(xTask1));
    // Serial.printf("]xTask_oled \r\n");
    draw("What a beautiful day!", SUN, 27);
    draw("The sun's come out!", SUN_CLOUD, 19);
    draw("It's raining cats and dogs.", RAIN, 8);
    draw("That sounds like thunder.", THUNDER, 12);
    draw("It's stopped raining", CLOUD, 15);
    delay(1000);
  }
}
/*
 * @TODO: 平滑滤波，not 平均滤波
 *
 */
void xTask_dbg(void *xTask)
{
  while (1)
  {
    // Serial.print("core[");
    // /* 获取任务被固定到 */
    // Serial.print(xTaskGetAffinity(xTask));
    // Serial.printf("]xTask_dbg \r\n");
    static int i = 262144;

    Serial.printf("[%6d]%d\r\n", adc_raw_data_avg[0], BC2V(adc_raw_data_avg[0], 1));
    i--;
    delay(100);

    // Serial.printf("%2.6f, %2.6f, %2.6f, %d %d, %d, %d, %d \r\n", adc_raw_data_avg[0] / 262144.0f * 20.0f, adc_raw_data_avg[1] / 262144.0f * 20.0f, adc_raw_data_avg[2] / 262144.0f * 20.0f, adc_raw_data_avg[3], adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);
  }
}

void xTask_adc(void *xTask)
{
  while (1)
  {
    // Serial.print("core[");
    // /* 获取任务被固定到 */
    // Serial.print(xTaskGetAffinity(xTask));
    // Serial.printf("]xTask_adc \r\n");
    static uint16_t cnt;
    // Serial.print("tsk_adc: ");
    // Serial.println(millis());
    AD7606C_18.fast_read(adc_raw_data);
    adc_raw_data[3] = 262144;
    for (uint8_t i = 0; i < 8; i++)
    {
      adc_raw_data_sum_256[i] += adc_raw_data[i];
    }
    if (++cnt >= 256)
    {
      cnt = 0;
      for (uint8_t i = 0; i < 8; i++)
      {
        adc_raw_data_avg[i] = adc_raw_data_sum_256[i] >> 8;
        adc_raw_data_sum_256[i] = 0;
      }
    }
    delayMicroseconds(730);
  }
}

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
    Serial.print("sendTask run on core ");
    /* 获取任务被固定到 */
    Serial.print(xTaskGetAffinity(xTask1));
    Serial.println(" is sending data");
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
      Serial.print(" got data: ");
      Serial.print("sender = ");
      Serial.print(data.sender);
      Serial.print(" msg = ");
      Serial.println(data.msg);
      free(data.msg);
    }
  }
  vTaskDelete(NULL);
}

void setup(void)
{
  hardware_init();

  /* 创建队列，其大小可包含5个元素Data */
  // xQueue = xQueueCreate(5, sizeof(Data));
  // xTaskCreatePinnedToCore(
  //     sendTask, "sendTask", /* 任务名称. */ 10000, /* 任务的堆栈大小 */ NULL, /* 任务的参数 */ 1, /* 任务的优先级 */ &xTask1, /* 跟踪创建的任务的任务句柄 */ 0); /* pin任务到核心0 */
  // xTaskCreatePinnedToCore(
  //     receiveTask, "receiveTask", 10000, NULL, 1, &xTask2, 1);

  // 最后一个参数至关重要，决定这个任务创建在哪个核上.PRO_CPU 为 0, APP_CPU 为 1,或者 tskNO_AFFINITY 允许任务在两者上运
  /*xTaskOne*/ /* 任务名称. */ /* 任务的堆栈大小 */ /* 任务的参数 */ /* 任务的优先级 */ /* 跟踪创建的任务的任务句柄 */ /* pin任务到核心0 */
  xTaskCreatePinnedToCore(xTask_dbg, "Task_dbg", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(xTask_adc, "Task_adc", 8072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(xTask_oled, "Task_oled", 8072, NULL, 2, NULL, 0);

  // r.setHighPriorityScheduler(&hpr);
  // r.enableAll(true); // this will recursively enable the higher priority tasks as well
}

void loop(void)
{
  delay(1000);
  rtc_wdt_feed(); // 喂狗函数
}
