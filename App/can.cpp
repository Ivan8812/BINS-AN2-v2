#include <app.h>
#include <map>
#include <cstring>
#include <CBuf.h>


#define QUEUE_LEN         20
#define ENQUEUE_TIMEOUT   1

using namespace std;

static CBuf<can_queue_item_t, 8> queue;
static map<uint8_t, uint8_t> cntrs;


extern "C"
{
  void FDCAN1_IT0_IRQHandler(void);
}



//------------------------------------------------------------------------------
static bool send(can_queue_item_t& item)
{
  static FDCAN_TxHeaderTypeDef tx_hdr;
  tx_hdr.Identifier = item.id;
  tx_hdr.IdType = FDCAN_EXTENDED_ID;
  tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
  tx_hdr.DataLength = item.len;
  tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
  tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
  tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_hdr.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(&can, &tx_hdr, (uint8_t*)item.data) == HAL_OK;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool can_init()
{
  if(HAL_FDCAN_ConfigInterruptLines(&can, FDCAN_ILS_SMSG, FDCAN_INTERRUPT_LINE0) != HAL_OK)
    return false;

  if(HAL_FDCAN_ActivateNotification(&can, FDCAN_IT_TX_COMPLETE, 0x7) != HAL_OK)
    return false;

  if(HAL_FDCAN_Start(&can) != HAL_OK)
    return false;


  NVIC_EnableIRQ(FDCAN1_IT0_IRQn); // Разрешить прерывание в NVIC

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool can_send_dat(uint16_t id, const void* data, const uint8_t len)
{
  if(len > 8)
    return false;

  can_queue_item_t msg;
  msg.id = id;
  msg.len = len;
  memcpy(&msg.data, data, len);

  taskENTER_CRITICAL();
  if(!queue.full())
  {
    queue.push(msg);
    if(HAL_FDCAN_GetTxFifoFreeLevel(&can) == 3)
    {
      auto next_msg = queue.pop();
      send(next_msg);
    }
  }
  taskEXIT_CRITICAL();

  return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool can_send_val(uint16_t id, float val, bool valid, bool test, bool calibr)
{
  if(cntrs.contains(id))
    cntrs[id]++;
  else
    cntrs[id] = 0;

  can_val_t msg;
  msg.state.raw = 0x0;
  msg.state.bins_ok = bins_status.bins_ok;
  msg.state.valid = valid;
  msg.state.test = test;
  msg.state.calibr = calibr;
  msg.value = val;
  msg.cntr = cntrs[id];

  return can_send_dat(id, &msg, sizeof(msg));
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void FDCAN1_IT0_IRQHandler(void)
{
  // Чтение регистра прерываний FDCAN
  uint32_t irq_flags = can.Instance->IR;
  can.Instance->IR = irq_flags;

  // Проверяем установлен ли флаг Transmission Complete (TC)
  if (irq_flags & FDCAN_IR_TC)
  {
    //can.Instance->IR = FDCAN_IR_TC;
    if(!queue.empty())
    {
      auto msg = queue.pop();
      send(msg);
    }
  }
}
//------------------------------------------------------------------------------

