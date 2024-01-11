#include "task.h"

void setup(void)
{
  hardware_init();
  xTaskCreatePinnedToCore(xTask_adcx, "Task_adcx", 8072, NULL, 1, NULL, PINNED_TO_CORE2);
  xTaskCreatePinnedToCore(xTask_wifi, "Task_wifi", 8072, NULL, 2, NULL, PINNED_TO_CORE1);
  xTaskCreatePinnedToCore(xTask_oled, "Task_oled", 8072, NULL, 3, NULL, PINNED_TO_CORE1);
  // xTaskCreatePinnedToCore(xTask_blex, "Task_blex", 8072, NULL, 4, NULL, PINNED_TO_CORE1);
  // xTaskCreatePinnedToCore(xTask_dbgx, "Task_dbgx", 4096, NULL, 6, NULL, PINNED_TO_CORE1);
}

int i = 0;                                                                                                                                                                                                                                                                                                  
void loop(void)
{
  vTaskDelay(50);
  rtc_wdt_feed(); // 喂狗函数
  if (i > 255)
    i = 0;
  dacWrite(DAC_CH2, sin_tab[i++]);
}
