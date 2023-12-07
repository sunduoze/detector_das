#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <TaskScheduler.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include "AD7606C.cpp"

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

Scheduler r, hpr;

// Callback methods prototypes
void adc_tskCallback();
void dbg_tskCallback();
void oled_tksCallback();

// Tasks
Task tsk_adc(500, TASK_FOREVER, &adc_tskCallback, &hpr); // adding task to the chain on creation
Task tsk_oled(100000, TASK_FOREVER, &oled_tksCallback, &r);
// Task tsk_dbg(100000, TASK_FOREVER, &dbg_tskCallback, &r);

// #Twos Complement Output Coding
// Bipolar Analog Input Ranges

/*
 * @TODO: 平滑滤波，not 平均滤波
 *
 */
void adc_tskCallback()
{
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
}

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

void dbg_tskCallback(void)
{
  static int i = 262144;

  Serial.printf("[%6d]%d\r\n", adc_raw_data_avg[0], BC2V(adc_raw_data_avg[0], 1));

  i--;
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

void oled_tksCallback(void)
{
  draw("What a beautiful day!", SUN, 27);
  draw("The sun's come out!", SUN_CLOUD, 19);
  draw("It's raining cats and dogs.", RAIN, 8);
  draw("That sounds like thunder.", THUNDER, 12);
  draw("It's stopped raining", CLOUD, 15);
  delay(1000);
}

void dbg_tskCallback2(void)
{
  Serial.printf("%2.6f, %2.6f, %2.6f, %d %d, %d, %d, %d \r\n", adc_raw_data_avg[0] / 262144.0f * 20.0f, adc_raw_data_avg[1] / 262144.0f * 20.0f, adc_raw_data_avg[2] / 262144.0f * 20.0f, adc_raw_data_avg[3], adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);
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
  for (uint8_t i = 0; i < 10; i++)
  {
    Serial.printf("[debug]0x%.2d=0x%.8x\r\n", i, adc_raw_data[i]);
  }
}

void setup(void)
{
  hardware_init();
  r.setHighPriorityScheduler(&hpr);
  r.enableAll(true); // this will recursively enable the higher priority tasks as well
}

uint8_t m = 24;

void loop(void)
{
  r.execute();

  // delayMicroseconds(5);

  // default ±10V single-end
  // Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d \r\n", adc_raw_data[0], adc_raw_data[1], adc_raw_data[2], adc_raw_data[3], adc_raw_data[4], adc_raw_data[5], adc_raw_data[6], adc_raw_data[7]);
}
