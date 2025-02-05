#ifndef INC_PACKETS_H_
#define INC_PACKETS_H_

#include "stdint.h"

#pragma pack(push, 1)


//---------------------------
// версия компилятора
// запрос версии - пустой пакет
#define PID_INFO	 				0x00
typedef struct
{
	char date[12];
	char time[9];
} opack_info_t;
//---------------------------


//---------------------------
// статистика работы пакетного драйвера
// запрос статистики - пустой пакет
#define PID_STAT	 				0x01
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
#define PID_SYNC					0x0F
//---------------------------


//---------------------------
// задание режима работы рс422
#define PID_MODE	 				0x10                                               // A5 10 00 01 01 B5 - перевод в PACK_MODE_PRINT
enum{PACK_MODE_MAIN, PACK_MODE_PRINT, PACK_MODE_PING, PACK_MODE_MTI_BRIDGE}; // A5 10 00 01 02 B6 - перевод в PACK_MODE_PING
typedef uint8_t ipack_mode_t;                                                // A5 10 00 01 03 B7 - перевод в PACK_MODE_MTI_BRIDGE
//---------------------------



//---------------------------
#define PID_IMU						0x21
typedef struct
{
	uint32_t systime;
	uint32_t status;

	float roll1;
	float pitch1;
	float heading1;

	float roll2;
	float pitch2;
	float heading2;

	float mag_x;
	float mag_y;
	float mag_z;

	float gyr_x;
	float gyr_y;
	float gyr_z;

	float acc_x;
	float acc_y;
	float acc_z;

	uint32_t ticks;
} opack_imu_t;
//---------------------------


//---------------------------
#define PID_AIR_SIGNALS		0x22
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
#define PID_GNSS					0x23
typedef struct
{
	uint32_t systime;
	uint32_t itow;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;
	uint32_t tacc;
	int32_t	nano;
	uint8_t fixtype;
	uint8_t flags;
	uint8_t numsv;
	int32_t lon;
	int32_t lat;
	int32_t height;
	int32_t hmsl;
	uint32_t hacc;
	uint32_t vacc;
	int32_t veln;
	int32_t vele;
	int32_t veld;
	int32_t gspeed;
	int32_t headmot;
	uint32_t sacc;
	uint32_t headacc;
	int32_t headveh;
	uint16_t gdop;
	uint16_t pdop;
	uint16_t tdop;
	uint16_t vdop;
	uint16_t hdop;
	uint16_t ndop;
	uint16_t edop;
} opack_gnss_t;
//---------------------------


//---------------------------
#define PID_TEMP					0x24
typedef struct
{
	uint32_t systime;
	float temp;
} opack_temp_t;
//---------------------------


//---------------------------
// передача текста (строки в стиле Си) в пакете
#define PID_TEXT					0x30
//---------------------------


//---------------------------
// информация о принятом битом пакете или пакете,
// для которого не найден обработчик
#define PID_UNKNOWN				0xFF
#define PID_BROKEN				0xFE
typedef struct
{
	uint8_t pid;
	uint8_t len;
} opack_unknown_t;
//---------------------------



#pragma pack(pop)
#endif /* INC_PACKETS_H_ */
