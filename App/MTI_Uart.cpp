#include <MTI_Uart.h>
#include <app.h>

__UART_HandleTypeDef* const MTI_Uart::uart = &uart_mti;


//------------------------------------------------------------------------------
MTI_Uart::io_state_t MTI_Uart::serve_io(uint32_t dt)
{
  serve_tim(dt);

#if USE_MTI_ICC
  if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
  {
    FDCAN_RxHeaderTypeDef rx_hdr;
    uint8_t buf[8];
    if((HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_hdr, buf) == HAL_OK) &&  (rx_hdr.DataLength == FDCAN_DLC_BYTES_1))
      icc(buf[0], hnd_icc);
  }
#endif

  io_state_t state = {false,false};
  while(__HAL_UART_GET_FLAG(uart, UART_FLAG_RXFNE))
  {
    volatile uint32_t tmp = uart->Instance->RDR;
    state.rxd = true;
    if(!buf_in.full())
      buf_in.push(tmp & 0xFF);
  }
  if(!buf_out.empty())
    state.txd = true;
  while(__HAL_UART_GET_FLAG(uart, UART_FLAG_TXFNF) && !buf_out.empty())
    uart->Instance->TDR = (uint32_t)buf_out.pop();
  state.txd &= buf_out.empty();

  return state;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool MTI_Uart::write(void* data, uint32_t len)
{
  if(buf_out.avail() < len)
    return false;
  for(uint32_t i=0; i<len; i++)
    buf_out.push(((uint8_t*)data)[i]);
  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool MTI_Uart::icc(uint8_t cmd, handler_t hnd)
{
  switch(cmd)
  {
  case ICC_START:
  case ICC_STOP:
  case ICC_GET_STATE:
    return send_cmd(MID_ICC, &cmd, 1, hnd, 100);
  case ICC_STORE:
  {
    bool res = send_pack(MID_GOTOCONFIG) && send_cmd(MID_ICC, &cmd, 1, hnd, 100);
    if(res)
      reset();
    return res;
  }
  default:
    return false;
  }
  return false;
}
//------------------------------------------------------------------------------




