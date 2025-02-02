#include "main.h"
/*
#include "bmp390_port_spi.h"
#include "bmp390_port_i2c.h"
#include "can_port.h"
#include "rs422_port.h"
#include "mti_port.h"
#include "stdbool.h"
#include "packets.h"
#include "air_signals.h"
*/

extern "C"
{
  #include "FreeRTOS.h"
  #include "task.h"
  #include "os.h"

  void vApplicationTickHook( void );
  void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
  void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
  void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);
}

TaskHandle_t task_idle;


//------------------------------------------------------------------------------
void vApplicationTickHook( void )
{
	/*rs422_io_state_t rs422_state = rs422_serve_io();
	if(rs422_state.rxd)
	{
		BaseType_t yield_req;
		vTaskNotifyGiveFromISR( (TaskHandle_t)TaskRS422Handle, &yield_req );
		portYIELD_FROM_ISR(yield_req);
	}

	if(rs422_state.mode != RS422_MTI_BRIDGE)
	{
		mti_io_state_t mti_state = mti_serve_io(1);
		if(mti_state.rxd)
		{
			BaseType_t yield_req;
			vTaskNotifyGiveFromISR( (TaskHandle_t)TaskMTIHandle, &yield_req );
			portYIELD_FROM_ISR(yield_req);
		}
	}
	else
	{
		mti_bypass = true;
	}

	systime++;
	bins_status.bins_ok = bins_status.mti_ok & bins_status.stat_pres_ok & bins_status.full_pres_ok;*/
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void tf_idle(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
    vTaskDelay(500);
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void os_run(void)
{

  //HAL_GPIO_WritePin(ANT_EN_GPIO_Port, ANT_EN_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
  //can_init();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  //TimerBmpHandle = osTimerNew(on_tim_bmp, osTimerPeriodic, NULL, &TimerBmp_attributes);
  //TimerCanHandle = osTimerNew(on_tim_can, osTimerPeriodic, NULL, &TimerCan_attributes);


  xTaskCreate(tf_idle, "Idle", 64, NULL, 1, &task_idle );


  //task_mti = osThreadNew(tf_mti, NULL, &TaskMTI_attributes);


  //task_mti = osThreadNew(tf_bmp390, NULL, &TaskBMP390_attributes);


  //task_rs422 = osThreadNew(tf_rs422, NULL, &TaskRS422_attributes);


  vTaskStartScheduler();
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  /* Idle task control block and stack */
  static StaticTask_t Idle_TCB;
  static StackType_t  Idle_Stack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer   = &Idle_TCB;
  *ppxIdleTaskStackBuffer = &Idle_Stack[0];
  *pulIdleTaskStackSize   = (uint32_t)configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  /* Timer task control block and stack */
  static StaticTask_t Timer_TCB;
  static StackType_t  Timer_Stack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer   = &Timer_TCB;
  *ppxTimerTaskStackBuffer = &Timer_Stack[0];
  *pulTimerTaskStackSize   = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}
//------------------------------------------------------------------------------
