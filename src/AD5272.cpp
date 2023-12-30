#include "AD5272.h"

// Constructor
AD5272::AD5272(int addr)
{
  _addr = addr;
}

int AD5272::init(void)
{
  Wire.begin();

  // Enable wiper adjustment.
  return write_data(AD5272_CONTROL_WRITE, AD5272_RDAC_WIPER_WRITE_ENABLE);
}

int AD5272::write_data(uint16_t cmd, uint16_t data)
{
  Wire.beginTransmission(_addr);
  Wire.write(byte((cmd << 2) | (0x0003 & (data >> 8))));
  Wire.write(byte(0x00FF & data));
  return Wire.endTransmission();
}

uint32_t AD5272::set_res_val(uint32_t resistor)
{
  uint32_t temp = resistor;

  if (temp > AD5272_RES_VAL)
    temp = AD5272_RES_VAL;
  temp = resistor * 1023 / AD5272_RES_VAL;
  // Serial.printf("set_res_val:%d\r\n", temp);
  return write_data(AD5272_RDAC_WRITE, temp);
}

int AD5272::read_rdac(void)
{
  uint16_t ret = 0;

  Wire.beginTransmission(_addr);
  Wire.write(AD5272_RDAC_READ << 2);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Failed");
    return -1;
  }

  Wire.requestFrom(_addr, 2);
  while (Wire.available())
    ret = (ret << 8) | Wire.read();

  return int(ret);
}