#pragma once 

#include "stdint.h"

#pragma pack(push, 1)


//---------------------------
// версия компилятора
// запрос версии - пустой пакет
#define PID_INFO          0x00
typedef struct
{
  char date[12];
  char time[9];
} opack_info_t;
//---------------------------


//---------------------------
// статистика работы пакетного драйвера
// запрос статистики - пустой пакет
#define PID_STAT          0x01
typedef struct
{
  uint32_t txd;
  uint32_t rxd;
  uint32_t tx_errors;
  uint32_t broken_cs;
  uint32_t broken_cntr;
} opack_stat_t;
//---------------------------


//---------------------------
// синхропакет, пустой, безусловно отправляется раз в секунду
#define PID_VERSION       0x0F
typedef struct
{
  uint32_t timestamp; // дата и время сборки в формате UNIX time
  uint8_t maj_ver;  //
  uint8_t min_ver;  // мажорная, минорная версии и номер билда. итоговая версия: <maj>.<min>.<build>
  uint8_t build;    //
} opack_version_t;
//---------------------------


//---------------------------
// задание режима работы рс422
#define PID_MODE          0x10                                                     // A5 10 00 01 01 B5 - перевод в PACK_MODE_GNSS_BRIDGE
enum{PACK_MODE_MAIN, PACK_MODE_GNSS_BRIDGE, PACK_MODE_PING, PACK_MODE_MTI_BRIDGE}; // A5 10 00 01 02 B6 - перевод в PACK_MODE_PING
typedef uint8_t ipack_mode_t;                                                      // A5 10 00 01 03 B7 - перевод в PACK_MODE_MTI_BRIDGE
//---------------------------



//---------------------------
#define PID_IMU           0x21
typedef struct
{
  uint32_t systime;
  uint32_t status;

  float roll1;
  float pitch1;
  float heading1;

  float mag_x;
  float mag_y;
  float mag_z;

  float gyr_x;
  float gyr_y;
  float gyr_z;

  float acc_x;
  float acc_y;
  float acc_z;
} opack_imu_t;
//---------------------------


//---------------------------
#define PID_AIR_SIGNALS   0x22
typedef struct
{
  uint32_t systime;
  float pres_stat;
  float pres_full;
  float temp_stat;
  float temp_full;
  float baroalt;
  float airspeed;
  float vertspeed;
} opack_air_t;
//---------------------------


//---------------------------
#define PID_GNSS_POS      0x23
typedef struct
{
  uint32_t systime;
  float timestamp;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  uint8_t quality;
} opack_gnss_pos_t;
//---------------------------


//---------------------------
#define PID_TEMP          0x24
typedef struct
{
  uint32_t systime;
  float temp;
} opack_temp_t;
//---------------------------


//---------------------------
#define PID_GNSS_ATT      0x25
typedef struct
{
  uint32_t systime;
  float timestamp;
  int32_t heading;
  int32_t pitch;
  int32_t roll;
  uint8_t quality;
} opack_gnss_att_t;
//---------------------------


//---------------------------
#define PID_GNSS_SPEED    0x26
typedef struct
{
  uint32_t systime;
  int32_t ground_speed;
  uint8_t quality;
} opack_gnss_speed_t;
//---------------------------


//---------------------------
// информация о принятом битом пакете или пакете,
// для которого не найден обработчик
#define PID_UNKNOWN       0xFF
#define PID_BROKEN        0xFE
typedef struct
{
  uint8_t pid;
  uint8_t len;
} opack_unknown_t;
//---------------------------



#pragma pack(pop)

