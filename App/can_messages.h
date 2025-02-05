#pragma once

#include "stdint.h"

#pragma pack(push, 1)


//----------------------------
typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t bins_ok:1;
    uint8_t valid:1;
    uint8_t test:1;
    uint8_t calibr:1;
  };
} can_msg_state_t;
//----------------------------


//----------------------------
// формат передачи вещественных значений из БИНС.
// в этом формате передается ориентация БИНС, координаты, скорости, давления
typedef struct
{
  float value;
  uint8_t cntr;
  can_msg_state_t state;
} can_val_t;
//--------------------------------
// углы в радианах
#define CAN_GROUP_ANG         0x10
#define CAN_VAL_ROLL          0x11
#define CAN_VAL_PITCH         0x12
#define CAN_VAL_HEADING       0x13
//------------------------------
// давление, Па
#define CAN_GROUP_PRES        0x20
#define CAN_VAL_PRES_STAT     0x21
#define CAN_VAL_PRES_FULL     0x22
#define CAN_VAL_PRES_DYN      0x23
//--------------------------------
// широта, долгота в градусах, высота - метры
#define CAN_GROUP_ATTITUDE    0x30
#define CAN_VAL_LONGITUDE     0x31
#define CAN_VAL_LATITUDE      0x32
#define CAN_VAL_BARO_HEIGHT   0x33
#define CAN_VAL_GNSS_HEIGHT   0x34
//--------------------------------
// скорость, м/с
#define CAN_GROUP_SPEED       0x40
#define CAN_VAL_IND_AIRSPEED  0x41
#define CAN_VAL_VERT_SPEED    0x42
//--------------------------------


//--------------------------------
// время и дата UTC (из гнсс)
typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  can_msg_state_t state;
} can_msg_time_t;
#define CAN_GROUP_TIME        0x100
#define CAN_MSG_TIME_DATE     0x101
//--------------------------------


//--------------------------------
// периодически отправляемые пустые сообщения
#define CAN_GROUP_HEARTBEAT   0x400
#define CAN_HEARTBEAT_INS     0x400
#define CAN_HEARTBEAT(N)	    (0x400 + ((N)&0x0F))
//--------------------------------


//--------------------------------
// состояние MTI (технологическое сообщение)
#define CAN_GROUP_MTI         0x410
#define CAN_MTI_STATUS        0x411
//--------------------------------
// слово состояния MTI
typedef uint32_t can_msg_mti_stat_t;
//--------------------------------

#pragma pack(pop)


