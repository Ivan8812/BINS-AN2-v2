#pragma once

#define RS422_LOG_OUT 1

#ifdef __cplusplus
extern "C"
{
#endif
  #include "FreeRTOS.h"
  #include "task.h"
  #include "timers.h"
  #include "main.h"
  #include "fdcan.h"
  #include "usart.h"
  #include "i2c.h"

  void app_run(void);

  void task_func_air_signals(void* arg);
  void task_func_rs422(void* arg);
  void task_func_idle(void* arg);
  void task_func_gnss(void* arg);
  void task_func_mti(void* arg);
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include "can.h"
#include "Packer.h"
#include "MTI_Uart.h"

//--------------------------------------
typedef union
{
  uint32_t raw;
  struct
  {
    uint32_t bins_ok:1;
    uint32_t bins_init_ok:1;
    uint32_t bins_self_test_ok:1;
    uint32_t imu_warm:1;
    uint32_t mti_ok:1;
    uint32_t gnss_ok:1;
    uint32_t stat_pres_ok:1;
    uint32_t full_pres_ok:1;
    uint32_t air_calibr_ok;
    uint32_t acc_calibr_ok;
    uint32_t gyr_calibr_ok;
    uint32_t mah_calibr_ok;
    uint32_t air_calibr_active;
    uint32_t acc_calibr_active;
    uint32_t gyr_calibr_active;
    uint32_t mag_calibr_active;
  };
} bins_status_t;
extern volatile bins_status_t bins_status;
//--------------------------------------
enum {RS422_PACKER=0x0, RS422_GNSS_BRIDGE=0x1, RS422_ECHO=0x2, RS422_MTI_BRIDGE=0x3};
extern volatile uint8_t rs422_mode;
//--------------------------------------
extern volatile uint32_t systime;
//--------------------------------------
extern volatile bool mti_bypass;
//--------------------------------------
extern Packer packer;
//--------------------------------------
extern MTI_Uart mti;
//--------------------------------------


//--------------------------------------
extern TaskHandle_t task_air_signals;
extern TaskHandle_t task_rs422;
extern TaskHandle_t task_idle;
extern TaskHandle_t task_gnss;
extern TaskHandle_t task_mti;
//--------------------------------------


//--------------------------------------
//extern bool stat_pres_err;
//extern bool full_pres_err;
//--------------------------------------

#endif //  __cplusplus
