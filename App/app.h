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
  void task_func_can(void* arg);
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include <can.h>
#include <Packer.h>
#include <MTI_Uart.h>
#include <build_time.h>
#include <can_messages.h>
#include <packets.h>

//--------------------------------------
// версия ПО
const uint8_t MajVersion = 0;
const uint8_t MinVersion = 1;
const uint8_t Build = 1;
//--------------------------------------



//--------------------------------------
typedef union
{
  uint32_t raw;
  struct
  {
    uint32_t bins_ok:1;  // 1 - бинс исправен
    uint32_t bins_init:1; // 1 - бинс в процессе инициализации
    uint32_t bins_self_test:1; // 1 - бинс в процессе самотестирования
    uint32_t imu_warm:1; // 1 - прогрев БЧЭ
    uint32_t mti_ok:1; // 1 - БЧЭ MTI-3 исправен
    uint32_t gnss_ok:1; // приемник ГНСС исправен
    uint32_t stat_pres_ok:1; // 1 - приемник статического давления исправен
    uint32_t full_pres_ok:1; // 1 - приемник полного давления исправен

    uint32_t air_calibr_ok:1;
    uint32_t acc_calibr_ok:1;
    uint32_t gyr_calibr_ok:1;
    uint32_t mag_calibr_ok:1;

    uint32_t air_calibr_active:1;
    uint32_t acc_calibr_active:1;
    uint32_t gyr_calibr_active:1;
    uint32_t mag_calibr_active:1;

    uint32_t bins_standalone:1; // 1 - бинс в независимом включении, 0 - бинс в составе ИСН
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
extern char nmea_string[];
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
