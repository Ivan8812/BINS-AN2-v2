#include "app.h"
#include "packets.h"


static void info_handler(const void* payload, const uint8_t len);
static void stat_handler(const void* payload, const uint8_t len);
static void mode_handler(const void* payload, const uint8_t len);

static const Packer::hnd_map_t handlers =
{
	{PID_INFO, info_handler},
	{PID_STAT, stat_handler},
	{PID_MODE, mode_handler}
};

Packer packer(handlers);

using namespace std;


//------------------------------------------------------------------------------
void task_func_serial(void *arg)
{
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    packer.parse_inbox();
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void info_handler(const void* payload, const uint8_t len)
{
	static const opack_info_t pack = {__DATE__, __TIME__ };
	packer.send(PID_INFO, (void*)&pack, sizeof(pack));
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void stat_handler(const void* payload, const uint8_t len)
{
	const Packer::stat_t& stat = packer.get_stat();
	packer.send(PID_STAT, &stat, sizeof(Packer::stat_t));
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void mode_handler(const void* payload, const uint8_t len)
{
	serial_mode = *((uint8_t*)payload) & 0x3;

	if(serial_mode == SERIAL_MTI_BRIDGE)
	{
		HAL_GPIO_WritePin(ANT_PWR_EN_GPIO_Port, ANT_PWR_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_SET);
	}
}
//------------------------------------------------------------------------------
