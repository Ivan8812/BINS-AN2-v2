#pragma once

#include <MTI.h>

extern "C"
{
#include "usart.h"
}


//-----------------------------------------------
class MTI_Uart final : public MTI
{
  enum { MID_ICC = 0x74 };
  static __UART_HandleTypeDef* const uart;
  CBuf<uint8_t, 8> buf_out;

  using MTI::serve_tim;
  bool write(void* data, uint32_t len);

public:
  using io_state_t = struct
  {
    bool rxd;
    bool txd;
  };

  MTI_Uart(std::initializer_list<cfg_t>& cfg, handler_t mtd_handler) : MTI(cfg, mtd_handler) {};
  MTI_Uart::io_state_t serve_io(uint32_t dt);

  enum{ICC_START, ICC_STOP, ICC_STORE, ICC_GET_STATE};
  bool icc(uint8_t cmd, handler_t hnd = nullptr);
};
//-----------------------------------------------
