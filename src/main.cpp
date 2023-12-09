#include "task.h"

#define MCU_CORE1 0
#define MCU_CORE2 1

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
  xTaskCreatePinnedToCore(xTask_adc, "Task_adc", 8072, NULL, 1, NULL, MCU_CORE2);
  xTaskCreatePinnedToCore(xTask_oled, "Task_oled", 8072, NULL, 3, NULL, MCU_CORE1);
  xTaskCreatePinnedToCore(xTask_wifi, "Task_wifi", 8072, NULL, 2, NULL, MCU_CORE1);
  xTaskCreatePinnedToCore(xTask_dbg, "Task_dbg", 4096, NULL, 4, NULL, MCU_CORE1);
}

void loop(void)
{
  vTaskDelay(1000);
  rtc_wdt_feed(); // 喂狗函数
}
