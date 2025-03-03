#include <app.h>
#include <BMP390.h>
#include <air_signals.h>
#include <packets.h>

void on_timer(TimerHandle_t xTimer);


#define ADDR_FULL   (0x76<<1)
#define ADDR_STAT   (0x77<<1)


//------------------------------------------------------------------------------
template<const uint8_t Addr>
bool read_mem(uint8_t addr, void* dst, uint8_t n)
{
  return HAL_I2C_Mem_Read(&i2c_bmp, Addr, addr, 1, (uint8_t*)dst, n, 10) == HAL_OK ? true : false;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
template<const uint8_t Addr>
bool write_byte(uint8_t addr, uint8_t data)
{
  return HAL_I2C_Mem_Write(&i2c_bmp, Addr, addr, 1, &data, 1, 10) == HAL_OK ? true : false;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void task_func_air_signals(void *arg)
{
  BMP390 bmp390_full(read_mem<ADDR_FULL>, write_byte<ADDR_FULL>);
  BMP390 bmp390_stat(read_mem<ADDR_STAT>, write_byte<ADDR_STAT>);


  bmp390_full.init();
  bmp390_stat.init();
  TimerHandle_t timer = xTimerCreate("air timer", pdMS_TO_TICKS(100), pdTRUE, (void*)1, on_timer);
  xTimerStart(timer, 0);

  float prev_stat_pres = 0.0f;
  bins_status_t prev_bins_status = {.raw = bins_status.raw};
  struct
  {
    float stat_sum = 0.0f, full_sum = 0.0f;
    uint32_t stat_n = 0, full_n = 0;
    float offset = 0.0f;
  } calibr;
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    BMP390::out_t val_full, val_stat;
    bins_status.stat_pres_ok = bmp390_full.get(val_full) ? 1 : 0;
    bins_status.full_pres_ok = bmp390_stat.get(val_stat) ? 1 : 0;

    // self calibration
    if(bins_status.bins_self_test)
    {
      bins_status.air_calibr_active = 1;
      if(bins_status.stat_pres_ok)
      {
        calibr.stat_sum += val_stat.pres;
        calibr.stat_n++;
      }
      if(bins_status.stat_pres_ok)
      {
        calibr.full_sum += val_full.pres;
        calibr.full_n++;
      }
    }
    else if(prev_bins_status.bins_self_test)
    {
      if(calibr.full_n && calibr.stat_n)
      {
        calibr.offset = calibr.full_sum/(float)calibr.full_n - calibr.stat_sum/(float)calibr.stat_n;
        calibr.full_n = calibr.stat_n = 0;
        calibr.full_sum = calibr.stat_sum = 0.0f;
        bins_status.air_calibr_ok = 1;
      }
      bins_status.air_calibr_active = 0;
    }
    prev_bins_status.raw = bins_status.raw;

    // main calculations
    float alt = 0.0f, vspeed = 0.0f, ias = 0.0f;
    if(bins_status.stat_pres_ok)
    {
      alt = altitude(val_stat.pres);
      vspeed = vert_speed(val_stat.pres, (val_stat.pres-prev_stat_pres)*10.0f);
      prev_stat_pres = val_stat.pres;

      if(bins_status.full_pres_ok)
      {
        ias = air_speed(val_full.pres - val_stat.pres - calibr.offset)/3.6f;
      }
    }

    // send data out
    if(bins_status.bins_standalone)
    {
      can_send_val(CAN_VAL_BINS2_PRES_STAT, val_stat.pres, bins_status.stat_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS2_PRES_FULL, val_full.pres, bins_status.full_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS2_PRES_DYN, val_full.pres - val_stat.pres, bins_status.stat_pres_ok && bins_status.full_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS2_VERT_SPEED, vspeed, bins_status.stat_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS2_IND_AIRSPEED, ias, bins_status.stat_pres_ok && bins_status.full_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS2_BARO_HEIGHT, alt, bins_status.stat_pres_ok, false, false);
    }
    else
    {
      can_send_val(CAN_VAL_BINS1_PRES_STAT, val_stat.pres, bins_status.stat_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS1_PRES_FULL, val_full.pres, bins_status.full_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS1_PRES_DYN, val_full.pres - val_stat.pres, bins_status.stat_pres_ok && bins_status.full_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS1_VERT_SPEED, vspeed, bins_status.stat_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS1_IND_AIRSPEED, ias, bins_status.stat_pres_ok && bins_status.full_pres_ok, false, false);
      can_send_val(CAN_VAL_BINS1_BARO_HEIGHT, alt, bins_status.stat_pres_ok, false, false);
    }

#if SERIAL_LOG_OUT
    opack_air_t pack;
    pack.systime = systime;
    pack.pres_stat = val_stat.pres;
    pack.pres_full = val_full.pres;
    pack.temp_stat = val_stat.temp;
    pack.temp_full = val_full.temp;
    pack.baroalt = alt;
    pack.airspeed = ias;
    pack.vertspeed = vspeed;

    packer.send(PID_AIR_SIGNALS, &pack, sizeof(pack));
#endif
  }
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void on_timer(TimerHandle_t xTimer)
{
  /* USER CODE BEGIN on_tim_bmp */
  xTaskNotifyGive(task_air_signals);
  /* USER CODE END on_tim_bmp */
}
//------------------------------------------------------------------------------
