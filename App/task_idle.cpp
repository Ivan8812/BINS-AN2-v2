#include <app.h>
#include <build_time.h>


//------------------------------------------------------------------------------
void task_func_idle(void *arg)
{
  /* Infinite loop */
  const can_msg_version_t version_can = {.timestamp=BuildTime, .maj_ver=MajVersion, .min_ver=MinVersion, .build=Build};
  const opack_version_t version_pack = {.timestamp=BuildTime, .maj_ver=MajVersion, .min_ver=MinVersion, .build=Build};

  for(;;)
  {
    HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);

    packer.send(PID_VERSION, &version_pack, sizeof(version_pack));

    if(bins_status.bins_standalone)
      can_send_dat(CAN_VERSION_BINS2, &version_can, sizeof(version_can));
    else
      can_send_dat(CAN_VERSION_BINS1, &version_can, sizeof(version_can));

    vTaskDelay(2000);
  }
}
//------------------------------------------------------------------------------
