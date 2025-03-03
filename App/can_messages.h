#pragma once

#include "stdint.h"

#pragma pack(push, 1)


//----------------------------
typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t bins_ok:1; 	// Исправность БИНС 0: неисправна, 1: исправна
    uint8_t valid:1;   	// Достоверность данных 0: недостоверны, 1: достоверны
    uint8_t test:1;     // Признак тестовых данных. 0: не тестовые, 1 : тестовые
    uint8_t calibr:1;   // Признак калибровочных данных. 0: рабочие: 1: калибровочные
  };
} can_msg_state_t;
//----------------------------


//----------------------------
// формат передачи вещественных значений из БИНС.
// в этом формате передается ориентация БИНС, скорости, давления
typedef struct
{
  float value;          // Значение передаваемой величины     
  uint8_t cntr;         // Счётчик сообщений параметров данного типа
  can_msg_state_t state;// Структура сообщения
} can_val_t;
//--------------------------------
// углы в радианах
#define CAN_GROUP_ATTITUDE          0x10
#define CAN_VAL_BINS1_ROLL          0x11
#define CAN_VAL_BINS1_PITCH         0x12
#define CAN_VAL_BINS1_HEADING       0x13
#define CAN_VAL_BINS2_ROLL          0x19
#define CAN_VAL_BINS2_PITCH         0x1A
#define CAN_VAL_BINS2_HEADING       0x1B
//------------------------------
// давление, Па
#define CAN_GROUP_PRES              0x20
#define CAN_VAL_BINS1_PRES_STAT     0x21
#define CAN_VAL_BINS1_PRES_FULL     0x22
#define CAN_VAL_BINS1_PRES_DYN      0x23
#define CAN_VAL_BINS2_PRES_STAT     0x29
#define CAN_VAL_BINS2_PRES_FULL     0x2A
#define CAN_VAL_BINS2_PRES_DYN      0x2B
//--------------------------------
// скорость м/с, высота м
#define CAN_GROUP_AIR               0x40
#define CAN_VAL_BINS1_IND_AIRSPEED  0x41
#define CAN_VAL_BINS1_VERT_SPEED    0x42
#define CAN_VAL_BINS1_BARO_HEIGHT   0x43
#define CAN_VAL_BINS2_IND_AIRSPEED  0x49
#define CAN_VAL_BINS2_VERT_SPEED    0x4A
#define CAN_VAL_BINS2_BARO_HEIGHT   0x4B
//--------------------------------


//--------------------------------
typedef struct
{
  int32_t value;				/* значение параметра */
  uint8_t quality;			/* качество параметра - один из элементов перечисления CAN_GNSS_QUAL_xxxx */
  uint8_t cntr;					/* счетчик сообщений для контроля непрерывности передачи данных */
  can_msg_state_t state;		/* слово состояния */
} can_gnss_val_t;

enum
{
  CAN_GNSS_QUAL_INVALID,
  CAN_GNSS_QUAL_SINGLE,
  CAN_GNSS_QUAL_DIFF,
  CAN_GNSS_QUAL_PPS,
  CAN_GNSS_QUAL_RTK_INT,
  CAN_GNSS_QUAL_RTK_FLOAT,
  CAN_GNSS_QUAL_DEAD_RECKONING,
  CAN_GNSS_QUAL_MANUAL,
  CAN_GNSS_QUAL_SIMULATOR,
  CAN_GNSS_QUAL_SBAS,
};

// широта, долгота, курс в градусах ЦМР 1e-7, высота мм, скорость мм/с
#define CAN_GROUP_GNSS              0x30
#define CAN_MSG_BINS1_LONGITUDE     0x31
#define CAN_MSG_BINS1_LATITUDE      0x32
#define CAN_MSG_BINS1_TRUE_HEADING  0x33
#define CAN_MSG_BINS1_GNSS_HEIGHT   0x34
#define CAN_MSG_BINS1_GROUND_SPEED  0x35
#define CAN_MSG_BINS2_LONGITUDE     0x39
#define CAN_MSG_BINS2_LATITUDE      0x3A
#define CAN_MSG_BINS2_TRUE_HEADING  0x3B
#define CAN_MSG_BINS2_GNSS_HEIGHT   0x3C
#define CAN_MSG_BINS2_GROUND_SPEED  0x3D
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

#define CAN_GROUP_TIME              0x100
#define CAN_MSG_BINS1_TIME_DATE     0x101
#define CAN_MSG_BINS2_TIME_DATE     0x109
//--------------------------------


//--------------------------------
// параметры DOP ГНСС решения
typedef struct
{
  uint8_t hdop;
  uint8_t sats; // количество спутников, использованных в решении
  uint8_t cntr;
  can_msg_state_t state;
} can_gnss_dop_t;
#define CAN_GROUP_GNSS_DOP          0x110
#define CAN_MSG_BINS1_GNSS_DOP      0x111
#define CAN_MSG_BINS2_GNSS_DOP      0x119
//--------------------------------


//--------------------------------
// версия ПО
typedef struct
{
  uint32_t timestamp; // дата и время сборки в формате UNIX time
  uint8_t maj_ver;  //
  uint8_t min_ver;  // мажорная, минорная версии и номер билда. итоговая версия: <maj>.<min>.<build>
  uint8_t build;    //
} can_msg_version_t;
#define CAN_GROUP_VERSION           0x400
#define CAN_VERSION_MFI             0x400	  // версия ПО МФИ
#define CAN_VERSION_BINS1           0x401   //
#define CAN_VERSION_BINS2           0x402   // версии ПО бинсов


//--------------------------------


//--------------------------------
// состояние MTI (технологическое сообщение)
#define CAN_GROUP_MTI               0x410	/* Отладочная информация */
#define CAN_MTI_BINS1_STATUS        0x411	/* БИНС1 Отладочная информация */
#define CAN_MTI_BINS2_STATUS        0x419	/* БИНС2 Отладочная информация */
//--------------------------------
// слово состояния MTI
typedef uint32_t can_msg_mti_stat_t;
//--------------------------------

#pragma pack(pop)


