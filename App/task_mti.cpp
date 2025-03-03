#include <app.h>
#include <MTI_Uart.h>
#include <map>
#include <optional>
#include <BE.h>
#include <AHRS.h>



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
	RATE_IMU  = 50,
	RATE_TEMP = 1,
};
//-------------------------------------

//-----------------------------------------------

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
    if(ulTaskNotifyTake(pdTRUE, 500))
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
void hnd_mtd(void *mtdata, const uint16_t len)
{
  using namespace Eigen;

  uint32_t status=0;
#if SERIAL_LOG_OUT
  uint32_t rec_time = systime;
#endif
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
    //bins_status.bins_self_test_ok = true;
    mti.set_no_rotation(1);
    //ahrs.set_no_rotation(1);
  }
  prev_status = status;

  if(bins_status.bins_standalone)
    can_send_dat(CAN_MTI_BINS2_STATUS, &status,  sizeof(status));
  else
    can_send_dat(CAN_MTI_BINS1_STATUS, &status,  sizeof(status));

  if(be_ang)
  {
    //be_ang->pitch = -be_ang->pitch;
    //be_ang->yaw = -be_ang->yaw;
    if(bins_status.bins_standalone)
    {
      can_send_val(CAN_VAL_BINS2_ROLL, be_ang->roll, bins_status.mti_ok, false, false);
      can_send_val(CAN_VAL_BINS2_PITCH, be_ang->pitch, bins_status.mti_ok, false, false);
      can_send_val(CAN_VAL_BINS2_HEADING, be_ang->yaw, bins_status.mti_ok, false, false);
    }
    else
    {
      can_send_val(CAN_VAL_BINS1_ROLL, be_ang->roll, bins_status.mti_ok, false, false);
      can_send_val(CAN_VAL_BINS1_PITCH, be_ang->pitch, bins_status.mti_ok, false, false);
      can_send_val(CAN_VAL_BINS1_HEADING, be_ang->yaw, bins_status.mti_ok, false, false);
    }
  }

#if SERIAL_LOG_OUT
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

    Vector3f acc({ pack.acc_x, pack.acc_y, pack.acc_z});
    Vector3f gyr({ pack.gyr_x, pack.gyr_y, pack.gyr_z});
    Vector3f mag({ pack.mag_x, pack.mag_y, pack.mag_z});
    acc /= -9.81f;
    //ahrs.step(acc, gyr, mag);
    //can_send_val(CAN_VAL_ROLL, eul[0], bins_status.mti_ok, false, false);
    //can_send_val(CAN_VAL_PITCH, eul[1], bins_status.mti_ok, false, false);
    //can_send_val(CAN_VAL_YAW, eul[2], bins_status.mti_ok, false, false);

    packer.send(PID_IMU, &pack, sizeof(pack));
  }
#endif

#if SERIAL_LOG_OUT
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

