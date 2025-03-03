#include <app.h>
#include <NMEA.h>
#include <cstring>
#include <array>
#include <SerialDevice.h>
#include <ubx_cfg_msg.h>



//------------------------------------------------------------------------------
using serial_io_state_t = struct
{
  uint8_t mode:6;
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

serial_io_state_t serial_serve_io();

bool serial_ping_test();

uint32_t process_int_gnss();
uint32_t process_ext_gnss();

void process_serial();

void init_int_gnss();
void init_ext_gnss();
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
TaskHandle_t task_air_signals;
TaskHandle_t task_serial;
TaskHandle_t task_idle;
TaskHandle_t task_gnss;
TaskHandle_t task_mti;
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
volatile bins_status_t bins_status = {0x0};
volatile uint8_t serial_mode = SERIAL_PACKER;
volatile uint32_t systime = 0;
volatile bool mti_bypass = false;
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void app_run(void)
{
  bins_status.raw = 0x0;
  bins_status.bins_init = 1;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_GPIO_WritePin(RST_GNSS_GPIO_Port, RST_GNSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ANT_PWR_EN_GPIO_Port, ANT_PWR_EN_Pin, GPIO_PIN_SET);

  can_init();

  if(serial_ping_test())
  {
    bins_status.bins_standalone = 1;
    serial_mode = SERIAL_NONE;
  }
  else
    bins_status.bins_standalone = 0;

  HAL_Delay(1000);
  init_int_gnss();
  init_ext_gnss();

  xTaskCreate(task_func_air_signals, "AirSignals", 512, NULL, 9,  &task_air_signals );
  xTaskCreate(task_func_serial,      "Serial",     256, NULL, 2,  &task_serial );
  xTaskCreate(task_func_idle,        "Idle",        64, NULL, 1,  &task_idle );
  xTaskCreate(task_func_gnss,        "GNSS",      1024, NULL, 8,  &task_gnss );
  xTaskCreate(task_func_mti,         "MTI",       1024, NULL, 10, &task_mti );

  bins_status.bins_init = 0;
  bins_status.bins_self_test = 1;
  vTaskStartScheduler();
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
serial_io_state_t serial_serve_io()
{
  serial_io_state_t state;
  state.mode = serial_mode;
  state.rxd = false;
  state.txd = false;
  switch(serial_mode)
  {
  case SERIAL_PACKER:
    while(__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_RXFNE))
    {
      state.rxd = true;
      if(!packer.inbox_full())
        packer.inbox_push(uart_232.Instance->RDR & 0xFF);
    }
    if(!packer.outbox_empty())
      state.txd = true;
    while(__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_TXFNF) && !packer.outbox_empty())
      uart_232.Instance->TDR = (uint32_t)packer.outbox_pop();
    state.txd &= packer.outbox_empty();
    break;
  //---------------------------------------------
  case SERIAL_GNSS_BRIDGE:
    while(__HAL_UART_GET_FLAG(&uart_gnss, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_232, UART_FLAG_TXFNF))
      uart_232.Instance->TDR = (uart_gnss.Instance->RDR & 0xFF);
    while(__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_gnss, UART_FLAG_TXFNF))
      uart_gnss.Instance->TDR = (uart_232.Instance->RDR & 0xFF);
    break;
  //---------------------------------------------
  case SERIAL_ECHO:
    while(__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_232, UART_FLAG_TXFNF))
      uart_232.Instance->TDR = (uart_232.Instance->RDR) & 0xFF;
    break;
  //---------------------------------------------
  case SERIAL_MTI_BRIDGE:
    while(__HAL_UART_GET_FLAG(&uart_mti, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_232, UART_FLAG_TXFNF))
      uart_232.Instance->TDR = (uart_mti.Instance->RDR & 0xFF);
    while(__HAL_UART_GET_FLAG(&uart_232, UART_FLAG_RXFNE) && __HAL_UART_GET_FLAG(&uart_mti, UART_FLAG_TXFNF))
      uart_mti.Instance->TDR = (uart_232.Instance->RDR & 0xFF);
    break;
  //---------------------------------------------
  case SERIAL_NONE:
    break;
  //---------------------------------------------
  default:
    break;
  };
  return state;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool serial_ping_test()
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
  process_serial();

  if(process_ext_gnss() > 1000)
    process_int_gnss();

	systime = systime + 1;
	bins_status.bins_ok = bins_status.mti_ok || bins_status.stat_pres_ok || bins_status.full_pres_ok;

	if(bins_status.bins_self_test && (systime > 2000))
	  bins_status.bins_self_test = 0;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
uint32_t process_int_gnss()
{
  static NMEA nmea_int;
  static uint32_t prev_rec_time = 0;

  while(__HAL_UART_GET_FLAG(&uart_gnss, UART_FLAG_RXFNE))
  {
    if(nmea_int.parse(uart_gnss.Instance->RDR) == NMEA::NMEA_RECEIVED)
    {
      nmea_int.get_message(nmea_string);
      BaseType_t yield_req;
      prev_rec_time = systime;
      vTaskNotifyGiveFromISR( (TaskHandle_t)task_gnss, &yield_req );
      portYIELD_FROM_ISR(yield_req);
    }
  }
  return systime - prev_rec_time;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
uint32_t process_ext_gnss()
{
  static NMEA nmea_ext;
  static uint32_t prev_rec_time = 0;

  while(__HAL_UART_GET_FLAG(&uart_422, UART_FLAG_RXFNE))
  {
    if(nmea_ext.parse(uart_422.Instance->RDR) == NMEA::NMEA_RECEIVED)
    {
      nmea_ext.get_message(nmea_string);
      BaseType_t yield_req;
      prev_rec_time = systime;
      vTaskNotifyGiveFromISR( (TaskHandle_t)task_gnss, &yield_req );
      portYIELD_FROM_ISR(yield_req);
    }
  }
  return systime - prev_rec_time;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void process_serial()
{
  serial_io_state_t serial_state = serial_serve_io();
  if(serial_state.rxd)
  {
    BaseType_t yield_req;
    vTaskNotifyGiveFromISR( (TaskHandle_t)task_serial, &yield_req );
    portYIELD_FROM_ISR(yield_req);
  }

  // process MTI data
  if(serial_mode != SERIAL_MTI_BRIDGE)
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
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void init_int_gnss()
{
  enum{BUFSIZE = 16};
  uint8_t out_buf[BUFSIZE];

  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_set_rate(out_buf, 200), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GGA, 1), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GLL, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GSA, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_GSV, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_RMC, 0), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_VTG, 1), HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart_gnss, out_buf, ubx_msg_enable(out_buf, UBX_NMEA_ZDA, 5), HAL_MAX_DELAY);
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void init_ext_gnss()
{
  auto send_string = [](const char* str) {HAL_UART_Transmit(&uart_422, (uint8_t*)str, strlen(str),HAL_MAX_DELAY); };
  send_string("UNLOGALL\r\n");
  send_string("CONFIG SBAS ENABLE SDCM\r\n");
  send_string("GNGGA 0.05\r\n");
  send_string("GPVTG 0.05\r\n");
  send_string("GPHPR 0.05\r\n");
  send_string("GPZDA 1\r\n");
  send_string("SAVECONFIG");
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
