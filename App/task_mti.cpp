#include <app.h>
#include <MTI_Uart.h>
#include <map>
#include <optional>
#include <BE.h>
#include <AHRS.h>
#if RS422_LOG_OUT
#include <packets.h>
#endif


using namespace std;


#pragma pack(push, 1)
//-------------------------------------
using ube16_t = BE<uint16_t>;
using ube32_t = BE<uint32_t>;
using ibe32_t = BE<int32_t>;
using fbe32_t = BE<float>;
//-------------------------------------
using be_ang_t = struct
{
	fbe32_t roll;
	fbe32_t pitch;
	fbe32_t yaw;
};
//-------------------------------------
using be_gyr_t = struct
{
	fbe32_t x;
	fbe32_t y;
	fbe32_t z;
};
//-------------------------------------
using be_acc_t = struct
{
	fbe32_t x;
	fbe32_t y;
	fbe32_t z;
};
//-------------------------------------
using be_mag_t = struct
{
	fbe32_t x;
	fbe32_t y;
	fbe32_t z;
};
//-------------------------------------
using be_hdr_t = struct
{
	ube16_t data_id;
	uint8_t len;
};
//-------------------------------------
#pragma pack(pop)



//-------------------------------------
enum
{
	MTD_ANG  = 0x2034,
	MTD_STAT = 0xE020,
	MTD_CNTR = 0x1020,
	MTD_TIME = 0x1060,
	MTD_TEMP = 0x0810,
	MTD_ACC  = 0x4020,
	MTD_GYR  = 0x8020,
	MTD_MAG  = 0xC020,
};
//-------------------------------------
enum
{
	RATE_IMU  = 100,
	RATE_TEMP = 1,
};
//-------------------------------------

//-----------------------------------------------

#if USE_MTI_ICC
static void hnd_icc(void* data, const uint16_t len);
#endif
static void hnd_mtd(void* data, const uint16_t len);

initializer_list<MTI::cfg_t> mti_cfg =
{
	{MTD_STAT, RATE_IMU},
	{MTD_ANG,  RATE_IMU},

	{MTD_ACC,  RATE_IMU},
	{MTD_GYR,  RATE_IMU},
	{MTD_MAG,  RATE_IMU},

	{MTD_TEMP, RATE_TEMP},
};

static AHRS ahrs((double)RATE_IMU);
MTI_Uart mti(mti_cfg, hnd_mtd);


//------------------------------------------------------------------------------
void task_func_mti(void *argument)
{
  HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_SET);

  for(;;)
  {
    if(ulTaskNotifyTake(pdTRUE, 5000))
      mti.parse_inbox();
    else
    {
      bins_status.mti_ok = false;
      if(!mti_bypass)
      {
        HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_RESET);
        mti.reset();
        ahrs.reset();
        HAL_Delay(50);
        HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_SET);
      }
    }

  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
#if USE_MTI_ICC
void hnd_icc(void* data, const uint16_t len)
{
	uint8_t cmd = *((uint8_t*)data);
	void* payload_be = (void*)((uint32_t)data + sizeof(cmd));

	if(cmd == MTI_Uart::ICC_STOP)
	{
		using result_be_t = struct{fbe32_t ddt; uint8_t dim; uint8_t status;};
		result_be_t* result = ((result_be_t*)payload_be);
		can_icc_result_t pack;
		pack.ddt = result->ddt;
		pack.dim = result->dim;
		pack.status = result->status;
		can_send_dat(CAN_MSG_RESULT, &pack, sizeof(pack));
	}
	else if(cmd == MTI_Uart::ICC_GET_STATE)
	{
		can_icc_state_t state = *((uint8_t*)payload_be);
		can_send_dat(CAN_MSG_STATE, &state, sizeof(state));
	}
}
#endif
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void hnd_mtd(void *mtdata, const uint16_t len)
{
  using namespace Eigen;

  uint32_t status=0;
  uint32_t rec_time = systime;
  optional<be_ang_t> be_ang;
  optional<be_acc_t> be_acc;
  optional<be_gyr_t> be_gyr;
  optional<be_mag_t> be_mag;
  optional<fbe32_t> be_temp;

  uint32_t pos = 0;
  while(pos < len)
  {
    be_hdr_t *header = ((be_hdr_t*)((uint32_t)mtdata + pos));
    pos += sizeof(be_hdr_t);
    void *pack_data = (void*)((uint32_t)mtdata + pos);
    switch(header->data_id)
    {
    case MTD_STAT:
      status = *((ube32_t*)pack_data);
      break;
    case MTD_ANG:
      be_ang = *((be_ang_t*)pack_data);
      break;
    case MTD_ACC:
      be_acc = *((be_acc_t*)pack_data);
      break;
    case MTD_GYR:
      be_gyr = *((be_gyr_t*)pack_data);
      break;
    case MTD_MAG:
      be_mag = *((be_mag_t*)pack_data);
      break;
    case MTD_TEMP:
      be_temp = *((fbe32_t*)pack_data);
      break;
    default:
      break;
    }
    pos += header->len;
  }

  bins_status.mti_ok = ((status & 0x3) == 0x3);

  static uint32_t prev_status = 0x0;
  if(((prev_status & 0x3) == 0x1) && ((status & 0x3) == 0x3))
  {
    bins_status.bins_self_test_ok = true;
    mti.set_no_rotation(1);
    //ahrs.set_no_rotation(1);
  }
  prev_status = status;
  can_send_dat(CAN_MTI_STATUS, &status,  sizeof(status));

  if(be_ang)
  {
    can_send_val(CAN_VAL_ROLL, be_ang->roll, bins_status.mti_ok, false, false);
    can_send_val(CAN_VAL_PITCH, be_ang->pitch, bins_status.mti_ok, false, false);
    can_send_val(CAN_VAL_HEADING, be_ang->yaw, bins_status.mti_ok, false, false);
  }

#if RS422_LOG_OUT
  if(be_ang && be_acc && be_gyr && be_mag)
  {
    opack_imu_t pack;
    pack.systime = rec_time;
    pack.status = status;
    pack.roll1 = be_ang->roll;
    pack.pitch1 = be_ang->pitch;
    pack.heading1 = be_ang->yaw;
    pack.acc_x = be_acc->x;
    pack.acc_y = be_acc->y;
    pack.acc_z = be_acc->z;
    pack.gyr_x = be_gyr->x;
    pack.gyr_y = be_gyr->y;
    pack.gyr_z = be_gyr->z;
    pack.mag_x = be_mag->x;
    pack.mag_y = be_mag->y;
    pack.mag_z = be_mag->z;

    uint32_t t1 = DWT->CYCCNT;
    Vector3f acc({ pack.acc_x, pack.acc_y, pack.acc_z});
    Vector3f gyr({ pack.gyr_x, pack.gyr_y, pack.gyr_z});
    Vector3f mag({ pack.mag_x, pack.mag_y, pack.mag_z});
    acc /= -9.81f;
    //ahrs.step(acc, gyr, mag);
    auto eul = ahrs.current_euler();
    uint32_t t2 = DWT->CYCCNT;
    pack.roll2 = eul[0];
    pack.pitch2 = eul[1];
    pack.heading2 = eul[2];
    pack.ticks = t2-t1;

    //can_send_val(CAN_VAL_ROLL, eul[0], bins_status.mti_ok, false, false);
    //can_send_val(CAN_VAL_PITCH, eul[1], bins_status.mti_ok, false, false);
    //can_send_val(CAN_VAL_YAW, eul[2], bins_status.mti_ok, false, false);

    packer.send(PID_IMU, &pack, sizeof(pack));
  }
#endif

#if RS422_LOG_OUT
  if(be_temp)
  {
    opack_temp_t pack;
    pack.systime = rec_time;
    pack.temp = *be_temp;
    packer.send(PID_TEMP, &pack, sizeof(pack));
  }
#endif
}
//------------------------------------------------------------------------------

