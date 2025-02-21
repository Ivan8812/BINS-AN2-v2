#include <app.h>
#include <NMEA.h>
#include <cstring>
#include <array>



//------------------------------------------------------------------------------
using rs422_io_state_t = struct
{
  uint8_t mode:2;
  uint8_t rxd:1;
  uint8_t txd:1;
};
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
extern "C"
{
  void vApplicationTickHook( void );
  void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
  void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
  void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);
}

rs422_io_state_t rs422_serve_io();
bool rs232_ping_test();
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
TaskHandle_t task_air_signals;
TaskHandle_t task_rs422;
TaskHandle_t task_idle;
TaskHandle_t task_gnss;
TaskHandle_t task_mti;
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
volatile bins_status_t bins_status = {0x0};
volatile uint8_t rs422_mode = RS422_PACKER;
volatile uint32_t systime = 0;
volatile bool mti_bypass = false;
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void app_run(void)
{
  bins_status.raw = 0x0;
  bins_status.bins_init = 1;

  bins_status.bins_standalone = (rs232_ping_test() ? 1 : 0);

  HAL_GPIO_WritePin(ANT_PWR_EN_GPIO_Port, ANT_PWR_EN_Pin, GPIO_PIN_SET);
  can_init();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  xTaskCreate(task_func_air_signals, "AirSignals", 512, NULL, 9,  &task_air_signals );
  xTaskCreate(task_func_rs422,       "RS-422",     256, NULL, 5,  &task_rs422 );
  xTaskCreate(task_func_idle,        "Idle",        64, NULL, 1,  &task_idle );
  xTaskCreate(task_func_gnss,        "GNSS",       512, NULL, 8,  &task_gnss );
  xTaskCreate(task_func_mti,         "MTI",       1024, NULL, 10, &task_mti );

  bins_status.bins_init = 0;
  bins_status.bins_self_test = 1;
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
bool rs232_ping_test()
{
  static const std::array<uint8_t, 16> test_set = {0x3A, 0x1F, 0x4B, 0x7C, 0xE2, 0x9D, 0x56, 0x8A, 0x0F, 0x3D, 0x6E, 0x91, 0xA4, 0xC7, 0x2B, 0xF8};

  // purge rx fifo
  while(__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_RXFNE))
    [[maybe_unused]] uint8_t byte = uart_232.Instance->RDR;

  for(auto test_val : test_set)
  {
    uart_232.Instance->TDR = test_val;
    while(!__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_TC)); // waiting for transmit completion
    for(volatile int i=0; i<1000; ) i = i + 1; // short delay
    if(uart_232.Instance->RDR != test_val)
      return false;
  }

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void vApplicationTickHook( void )
{
  // process RS422 data
	rs422_io_state_t rs422_state = rs422_serve_io();
	if(rs422_state.rxd)
	{
		BaseType_t yield_req;
		vTaskNotifyGiveFromISR( (TaskHandle_t)task_rs422, &yield_req );
		portYIELD_FROM_ISR(yield_req);
	}

	// process MTI data
	if(rs422_state.mode != RS422_MTI_BRIDGE)
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

	// process GNSS data
	static NMEA nmea;
	while(__HAL_UART_GET_FLAG(&uart_gnss, UART_FLAG_RXFNE))
	{
	  if(nmea.parse(uart_gnss.Instance->RDR) == NMEA::NMEA_RECEIVED)
	  {
	    memcpy(nmea_string, nmea.get_message(), nmea.get_message_length());
	    nmea.reset();
	    BaseType_t yield_req;
	    vTaskNotifyGiveFromISR( (TaskHandle_t)task_gnss, &yield_req );
	    portYIELD_FROM_ISR(yield_req);
	  }
	}

	systime = systime + 1;
	//bins_status.stat_pres_ok = stat_pres_err ? 1 : 0;
	//bins_status.full_pres_ok = full_pres_err ? 1 : 0;
	bins_status.bins_ok = bins_status.mti_ok || bins_status.stat_pres_ok || bins_status.full_pres_ok;

	if(bins_status.bins_self_test && (systime > 2000))
	  bins_status.bins_self_test = 0;
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
