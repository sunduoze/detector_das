#include "task.h"
void i2c_dev_scan();

void setup(void)
{
  hardware_init();
  xTaskCreatePinnedToCore(xTask_adcx, "Task_adcx", 8072, NULL, 1, NULL, PINNED_TO_CORE2);
  xTaskCreatePinnedToCore(xTask_wifi, "Task_wifi", 8072, NULL, 2, NULL, PINNED_TO_CORE1);
  // xTaskCreatePinnedToCore(xTask_blex, "Task_blex", 8072, NULL, 4, NULL, PINNED_TO_CORE1);
  xTaskCreatePinnedToCore(xTask_oled, "Task_oled", 8072, NULL, 3, NULL, PINNED_TO_CORE1);
  xTaskCreatePinnedToCore(xTask_dbgx, "Task_dbgx", 4096, NULL, 6, NULL, PINNED_TO_CORE1);
  xTaskCreatePinnedToCore(xTask_rotK, "Task_rotK", 8072, NULL, 5, NULL, PINNED_TO_CORE1);
}

int i = 0;
void loop(void)
{
  vTaskDelay(100);
  rtc_wdt_feed(); // 喂狗函数
  if (i > 255)
    i = 0;
  dacWrite(DAC_CH2, sin_tab[i++]);
}

// #include <Wire.h>
// #include <Arduino.h>

// #define Addr 0x2E // AD5252默认地址
// #define wp 13     // 写入接保护引脚

// #define AD5252_EE_RDAC (1 << 5) // 读写emm辅助计算

// void res_set(int date, int chal);
// unsigned int red_res(int chal);

// unsigned int red_eem(int chal);
// void set_eem(int date, int chal);

// void test_2()
// {

//   uint8_t c, val;

//   // Unlocking *******************************************************************************

//   Wire.beginTransmission(0x2E); // transmit to device with address 0x2C
//   Wire.write(0b00011100);       // Write contents of the serial register data to the control register.
//   Wire.write(0b00000010);       // sends instruction byte for unlocking
//   c = Wire.endTransmission();   // stop transmitting
//   Serial.printf("[debug1]0x%x", c);
//   // Writing to RDAC procedure *******************************************************************************
//   Wire.beginTransmission(0x2C); // transmit to device 0x2C
//   Wire.write(0b00000100);       // Write contents of serial register data to RDAC.
//   Wire.write(0b0000111);        // write 7 to RDAC (random number)
//   c = Wire.endTransmission();   // stop transmitting
//   Serial.printf("[debug2]0x%x", c);
//   delay(1000);

//   // Reading from RDAC procedure ***************************************************************************

//   Wire.beginTransmission(0x2C); // transmit to device 0x2C
//   Wire.write(0b00001000);       // sends instruction byte for RDAC reading
//   Wire.write(0b00000000);       // doesn't care actually
//   c = Wire.endTransmission();   // stop transmitting
//   delay(300);
//   Serial.printf("[debug3]0x%x", c);
//   // Picking up the response, containing 2 bytes RDAC readback*********************************

//   Wire.requestFrom(0x2c, 2);
//   if (Wire.available() == 2)
//   {
//     c = Wire.read();        // receive a byte as character
//     Serial.print("RDAC ="); // print the character
//     Serial.print(c);        // print the character
//     Serial.println(" ");
//   }

//   // Requesting control register content***************************************************
//   Wire.beginTransmission(0x2C); // transmit to device 0x2C
//   Wire.write(0b0100000);        // sends instruction byte for reading
//   Wire.write(0b00000000);       // doesn't care
//   Wire.endTransmission();       // stop transmitting

//   Wire.requestFrom(0x2c, 2);

//   if (Wire.available() == 2)
//   {
//     val = Wire.read(); // receive a byte as character
//     Serial.println(" ");
//     Serial.print("Register ="); // print the character
//     Serial.print(val);          // print the character
//     Serial.println(" ");
//   }
// }
// void setup()
// {
//   // i2c接口初始化
//   Wire.begin();
//   // 串口初始化，波特率9600
//   Serial.begin(115200);

//   delay(4000);
//   Wire.beginTransmission(0x5C);//
//   // Wire.write(byte((0x07 << 2) | (0x0003 & (0x02 >> 8))));
//   // Wire.write(byte(0x00FF & 0x02));
//   Serial.printf("i2c ==== ret:0x%d", Wire.endTransmission());
//   delay(1000);
//   test_2();
//   while (1)
//   {
//   }

//   res_set(0, 1);
//   delay(100);
//   res_set(0, 2);

//   set_eem(128, 1);
//   set_eem(64, 11);
// }

// void loop()
// {
// }
//   if (Serial.available() > 0)
//   {
//     int value = Serial.parseInt(); // 获取串口输入的整数
//     if (value > 0 && value < 256)
//     {
//       Serial.print("set value: ");
//       Serial.println(value);
//       res_set(value, 1);
//       delay(100);
//       res_set(value, 2);
//       set_eem(value, 1);
//     }
//     else
//     {
//       if (value != 0)
//         Serial.println("设置值错误，设置的值大于0小于256");
//     }
//   }

//   float res_1 = (red_res(1) / 256.0);
//   float res_2 = (red_res(2) / 256.0);

//   unsigned int res1_read = analogRead(A0);
//   unsigned int res2_read = analogRead(A1);

//   double b1 = 0.0049 * res1_read;     // 测到的电压
//   double res1_real = 3.2 - (11 / b1); // 计算电阻

//   double b2 = 0.0049 * res2_read;     // 测到的电压
//   double res2_real = 3.2 - (11 / b2); // 计算电阻
//   // Output data to serial monitor
//   unsigned int red_eem1 = red_eem(1);
//   unsigned int red_eem11 = red_eem(11);
//   Serial.print("eem1: ");
//   Serial.println(red_eem1);
//   Serial.print("eem11: ");
//   Serial.println(red_eem11);

//   Serial.print("res_1: ");
//   Serial.print(res_1);
//   Serial.print(" K;");
//   Serial.print("res1_read: ");
//   Serial.print(res1_real);
//   Serial.println(" K");
//   Serial.print("res_2: ");
//   Serial.print(res_2);
//   Serial.print(" K;");
//   Serial.print("res1_read: ");
//   Serial.print(res2_real);
//   Serial.println(" K");
//   delay(1000);
// }
// /*
// 功能: ad5252电阻设置函数
// 输出参数： date设置的电阻值 0 -- 255
//          chal = 1 --> RDAC1
//          chal = 2 --> RDAC3
// */
// void res_set(int date, int chal)
// {
//   int channel;
//   if (chal == 1)
//   {
//     channel = 0x01;
//   }
//   if (chal == 2)
//   {
//     channel = 0x03;
//   }
//   digitalWrite(wp, HIGH); // pin2 always low
//   delay(100);
//   // 开始i2c传输
//   Wire.beginTransmission(Addr);
//   // 设置通道
//   Wire.write(channel);
//   // Input resistance value, 0x80(128)
//   Wire.write(date);
//   // 停止i2c传输
//   Wire.endTransmission();
//   delay(100);
//   digitalWrite(wp, LOW);
// }
// /*
// 功能: ad5252电阻设置值读取函数
// 输出参数：
//          chal = 1 --> RDAC1
//          chal = 2 --> RDAC3
// */
// unsigned int red_res(int chal)
// {
//   unsigned int data;
//   int channel;
//   if (chal == 1)
//   {
//     channel = 0x01;
//   }
//   if (chal == 2)
//   {
//     channel = 0x03;
//   }
//   // 开始i2c传输
//   Wire.beginTransmission(Addr);
//   // 选择寄存器
//   Wire.write(channel);
//   // 停止i2c通信
//   Wire.endTransmission();
//   // 传输1位读命令
//   Wire.requestFrom(Addr, 1);
//   // Read 1 byte of data
//   if (Wire.available() == 1)
//   {
//     data = Wire.read();
//   }
//   return data;
// }
// /*
// 功能: 写入数据到寄存器
// 输出参数：
//          date ——> 数据
//          chal 1到11 对应 eem4到eem15
// */
// void set_eem(int date, int chal)
// {
//   int channel;
//   switch (chal)
//   {
//   case 1:
//     channel = 0x04;
//     break;
//   case 2:
//     channel = 0x05;
//     break;
//   case 3:
//     channel = 0x06;
//     break;
//   case 4:
//     channel = 0x07;
//     break;
//   case 5:
//     channel = 0x08;
//     break;
//   case 6:
//     channel = 0x0A;
//     break;
//   case 7:
//     channel = 0x0B;
//     break;
//   case 8:
//     channel = 0x0C;
//     break;
//   case 9:
//     channel = 0x0D;
//     break;
//   case 10:
//     channel = 0x0E;
//     break;
//   case 11:
//     channel = 0x0F;
//   }
//   digitalWrite(wp, HIGH); // pin2 always low
//   delay(100);
//   // 开始i2c传输
//   Wire.beginTransmission(Addr);
//   // 设置寄存器
//   Wire.write(AD5252_EE_RDAC | channel);
//   Wire.write(date);
//   // 停止i2c
//   delay(100);
//   Wire.endTransmission();
//   delay(100);
//   digitalWrite(wp, LOW);
// }
// /*
// 功能: 读寄存器数据
// 输出参数：
//          chal 1到11 对应 eem4到eem15
// */
// unsigned int red_eem(int chal)
// {
//   unsigned int data;
//   int channel;
//   switch (chal)
//   {
//   case 1:
//     channel = 0x04;
//     break;
//   case 2:
//     channel = 0x05;
//     break;
//   case 3:
//     channel = 0x06;
//     break;
//   case 4:
//     channel = 0x07;
//     break;
//   case 5:
//     channel = 0x08;
//     break;
//   case 6:
//     channel = 0x0A;
//     break;
//   case 7:
//     channel = 0x0B;
//     break;
//   case 8:
//     channel = 0x0C;
//     break;
//   case 9:
//     channel = 0x0D;
//     break;
//   case 10:
//     channel = 0x0E;
//     break;
//   case 11:
//     channel = 0x0F;
//   }
//   // 开始i2c
//   Wire.beginTransmission(Addr);
//   // 选择寄存器
//   Wire.write(AD5252_EE_RDAC | channel);
//   // 停止i2c通信
//   Wire.endTransmission();
//   Wire.requestFrom(Addr, 1);
//   if (Wire.available() == 1)
//   {
//     data = Wire.read();
//   }
//   return data;
// }

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