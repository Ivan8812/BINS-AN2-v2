#include "app.h"
#include "packets.h"


//------------------------------------------------------------------------------
void task_func_idle(void *arg)
{
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
    packer.send(PID_SYNC);
    vTaskDelay(500);
  }
}
//------------------------------------------------------------------------------
