#include "app.h"

//------------------------------------------------------------------------------
void task_func_gnss(void *arg)
{
  HAL_GPIO_WritePin(RST_GNSS_GPIO_Port, RST_GNSS_Pin, GPIO_PIN_SET);
  for(;;)
  {
    vTaskDelay(500);
  }
}
//------------------------------------------------------------------------------
