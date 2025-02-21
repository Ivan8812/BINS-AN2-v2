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
#define CAN_GROUP_ATTITUDE          0x10	/* Группа значений координат */
#define CAN_VAL_BINS1_ROLL          0x11	/* БИНС1 Крен */
#define CAN_VAL_BINS1_PITCH         0x12	/* БИНС1 Тангаж */
#define CAN_VAL_BINS1_HEADING       0x13	/* БИНС1 Курс */
#define CAN_VAL_BINS2_ROLL          0x19	/* БИНС2 Крен */
#define CAN_VAL_BINS2_PITCH         0x1A	/* БИНС2 Тангаж */
#define CAN_VAL_BINS2_HEADING       0x1B	/* БИНС2 Курс */
//------------------------------
// давление, Па
#define CAN_GROUP_PRES              0x20	/* Группа значений статического, полного, динамического давлении*/
#define CAN_VAL_BINS1_PRES_STAT     0x21	/* БИНС1 Статическое давление */
#define CAN_VAL_BINS1_PRES_FULL     0x22	/* БИНС1 Полное давление */
#define CAN_VAL_BINS1_PRES_DYN      0x23	/* БИНС1 Динамическое давление */
#define CAN_VAL_BINS2_PRES_STAT     0x29	/* БИНС2 Статическое давление */
#define CAN_VAL_BINS2_PRES_FULL     0x2A	/* БИНС2 Полное давление */
#define CAN_VAL_BINS2_PRES_DYN      0x2B	/* БИНС2 Динамическое давление */
//--------------------------------
// скорость, м/с
#define CAN_GROUP_SPEED             0x40	/* Группа параметроы скорости */
#define CAN_VAL_BINS1_IND_AIRSPEED  0x41	/* БИНС1 Приборная скорость */
#define CAN_VAL_BINS1_VERT_SPEED    0x42	/* БИНС1 Вертикальная скорость */
#define CAN_VAL_BINS2_IND_AIRSPEED  0x49	/* БИНС2 Приборная скорость */
#define CAN_VAL_BINS2_VERT_SPEED    0x4A	/* БИНС2 Вертикальная скорость */
//--------------------------------


//--------------------------------
typedef struct
{
  int32_t value;				/* значение параметра */
  uint8_t quality;				/* качество параметра - один из элементов перечисления CAN_POS_QUAL_xxxx */
  uint8_t cntr;					/* счетчик сообщений для контроля непрерывности передачи данных */
  can_msg_state_t state;		/* слово состояния */
} can_msg_position_t;

enum
{
  CAN_POS_QUAL_GNSS_INVALID,    /*                                 */
  CAN_POS_QUAL_GNSS_FIX,        /*                                 */
  CAN_POS_QUAL_GNSS_SBAS,       /*  признаки качества ГНСС решения */
  CAN_POS_QUAL_GNSS_RTK_FLOAT,  /*                                 */
  CAN_POS_QUAL_GNSS_RTK_FIXED,  /*                                 */
  CAN_POS_QUAL_STANDALONE = 0xFF, /* автономное решение */
};

// широта, долгота в градусах ЦМР 1e-7, высота - мм
#define CAN_GROUP_POSITION          0x30 	/* Группа параметров координат позиционирования */
#define CAN_MSG_BINS1_LONGITUDE     0x31	/* БИНС1 Долгота в градусах */
#define CAN_MSG_BINS1_LATITUDE      0x32	/* БИНС1 Широта в градусах */
#define CAN_MSG_BINS1_BARO_HEIGHT   0x33	/* БИНС1 Баровысота в миллиметрах	*/
#define CAN_MSG_BINS1_GNSS_HEIGHT   0x34	/* БИНС1 Высота эллипсоида в миллиметрах */ 
#define CAN_MSG_BINS2_LONGITUDE     0x39	/* БИНС2 Долгота */
#define CAN_MSG_BINS2_LATITUDE      0x3A	/* БИНС2 Широта */
#define CAN_MSG_BINS2_BARO_HEIGHT   0x3B	/* БИНС2 Баровысота в миллиметрах	*/
#define CAN_MSG_BINS2_GNSS_HEIGHT   0x3C	/* БИНС2 Высота эллипсоида в миллиметрах */
//--------------------------------


//--------------------------------
// время и дата UTC (из гнсс)
typedef struct
{
  uint16_t year;                    /* Год */
  uint8_t month;                    /* Месяц */
  uint8_t day;                      /* День */
  uint8_t hour;                     /* Час */
  uint8_t min;                      /* Минуты */
  uint8_t sec;                      /* Секунды */
  can_msg_state_t state;            /* слово состояния */
} can_msg_time_t;

#define CAN_GROUP_TIME              0x100 	/* Группа сообщений даты / времени */
#define CAN_MSG_BINS1_TIME_DATE     0x101	/* ЬИНС1 Сообщение: Дата время */
#define CAN_MSG_BINS2_TIME_DATE     0x109	/* ЬИНС2 Сообщение: Дата время */
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
#define CAN_VERSION_MFI             0x400	// версия ПО МФИ
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


