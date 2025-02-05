#include "app.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#include "main.h"
#include "usart.h"
#pragma GCC diagnostic pop


//------------------------------------------------------------------------------
extern "C"
{
  void vApplicationTickHook( void );
  void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
  void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
  void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
TaskHandle_t task_air_signals;
TaskHandle_t task_rs422;
TaskHandle_t task_idle;
TaskHandle_t task_gnss;
TaskHandle_t task_mti;
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
using rs422_io_state_t = struct
{
  uint8_t mode:2;
  uint8_t rxd:1;
  uint8_t txd:1;
};
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
volatile bins_status_t bins_status = {0x0};
volatile uint8_t rs422_mode = RS422_PACKER;
//volatile uint8_t rs422_mode = RS422_MTI_BRIDGE;
volatile uint32_t systime = 0;
volatile bool mti_bypass = false;
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void app_run(void)
{
  HAL_GPIO_WritePin(ANT_PWR_EN_GPIO_Port, ANT_PWR_EN_Pin, GPIO_PIN_SET);
  can_init();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  xTaskCreate(task_func_air_signals, "AirSignals", 512, NULL, 9,  &task_air_signals );
  xTaskCreate(task_func_rs422,       "RS-422",     128, NULL, 5,  &task_rs422 );
  xTaskCreate(task_func_idle,        "Idle",       64,  NULL, 1,  &task_idle );
  xTaskCreate(task_func_gnss,        "GNSS",       128, NULL, 1,  &task_gnss );
  xTaskCreate(task_func_mti,         "1024",       512, NULL, 10, &task_mti );

  vTaskStartScheduler();
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
rs422_io_state_t rs422_serve_io()
{
  rs422_io_state_t state;
  state.mode = rs422_mode;
  state.rxd = false;
  state.txd = false;
  switch(rs422_mode)
  {
  case RS422_PACKER:
    while(__HAL_UART_GET_FLAG(&uart_422, UART_FLAG_RXFNE))
    {
      state.rxd = true;
      if(!packer.inbox_full())
        packer.inbox_push(uart_422.Instance->RDR & 0xFF);
    }
    if(!packer.outbox_empty())
      state.txd = true;
    while(__HAL_UART_GET_FLAG(&uart_422, UART_FLAG_TXFNF) && !packer.outbox_empty())
      uart_422.Instance->TDR = (uint32_t)packer.outbox_pop();
    state.txd &= packer.outbox_empty();
    break;
  //---------------------------------------------
  case RS422_GNSS_BRIDGE:
    while(__HAL_UART_GET_FLAG(&uart_gnss, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_422, UART_FLAG_TXFNF))
      uart_422.Instance->TDR = (uart_gnss.Instance->RDR & 0xFF);
    while(__HAL_UART_GET_FLAG(&uart_422, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_gnss, UART_FLAG_TXFNF))
      uart_gnss.Instance->TDR = (uart_422.Instance->RDR & 0xFF);
    break;
  //---------------------------------------------
  case RS422_ECHO:
    while(__HAL_UART_GET_FLAG(&uart_422, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_422, UART_FLAG_TXFNF))
      uart_422.Instance->TDR = (uart_422.Instance->RDR) & 0xFF;
    break;
  //---------------------------------------------
  case RS422_MTI_BRIDGE:
    while(__HAL_UART_GET_FLAG(&uart_mti, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_422, UART_FLAG_TXFNF))
      uart_422.Instance->TDR = (uart_mti.Instance->RDR & 0xFF);
    while(__HAL_UART_GET_FLAG(&uart_422, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_mti, UART_FLAG_TXFNF))
      uart_mti.Instance->TDR = (uart_422.Instance->RDR & 0xFF);
    break;
  };
  return state;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void vApplicationTickHook( void )
{
	rs422_io_state_t rs422_state = rs422_serve_io();
	if(rs422_state.rxd)
	{
		BaseType_t yield_req;
		vTaskNotifyGiveFromISR( (TaskHandle_t)task_rs422, &yield_req );
		portYIELD_FROM_ISR(yield_req);
	}

	if(rs422_state.mode != RS422_MTI_BRIDGE)
	//if(rs422_mode != RS422_MTI_BRIDGE)
	{
	  MTI_Uart::io_state_t mti_state = mti.serve_io(1);
		if(mti_state.rxd)
		{
			BaseType_t yield_req;
			vTaskNotifyGiveFromISR( (TaskHandle_t)task_mti, &yield_req );
			portYIELD_FROM_ISR(yield_req);
		}
	}
	else
	{
		mti_bypass = true;
	}

	systime = systime + 1;
	//bins_status.stat_pres_ok = stat_pres_err ? 1 : 0;
	//bins_status.full_pres_ok = full_pres_err ? 1 : 0;
	bins_status.bins_ok = bins_status.mti_ok & bins_status.stat_pres_ok & bins_status.full_pres_ok;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
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
