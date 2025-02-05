#include <app.h>
#include <can.h>
#include <map>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#include "fdcan.h"
#include "main.h"
#pragma GCC diagnostic pop

using namespace std;

static FDCAN_HandleTypeDef *fdcan = &hfdcan1;
static map<uint8_t, uint8_t> cntrs;


//------------------------------------------------------------------------------
void can_init()
{

#if USE_MTI_ICC
  FDCAN_FilterTypeDef filter_cfg;
  filter_cfg.IdType = FDCAN_STANDARD_ID;
  filter_cfg.FilterIndex = 0;
  filter_cfg.FilterType = FDCAN_FILTER_MASK;
  filter_cfg.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_cfg.FilterID1 = CAN_CMD_ICC;
  filter_cfg.FilterID2 = 0x7FF;
  filter_cfg.RxBufferIndex = 0;
  HAL_FDCAN_ConfigFilter(fdcan, &filter_cfg);
#endif

  HAL_FDCAN_Start(fdcan);
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool can_send_dat(uint16_t id, void* data, const uint8_t len)
{
	// check CAN state first of all
	uint32_t psr = (fdcan->Instance->PSR);
	if(psr & 0x80) // bus-off status
		HAL_FDCAN_Start(fdcan);

	if(len > 8)
		return false;

	if(HAL_FDCAN_GetTxFifoFreeLevel(fdcan) == 0)
		return false;

	FDCAN_TxHeaderTypeDef tx_hdr;
	tx_hdr.Identifier = id;
	tx_hdr.IdType = FDCAN_STANDARD_ID;
	tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
	tx_hdr.DataLength = len;
	tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
	tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
	tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_hdr.MessageMarker = 0;
	return (HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_hdr, (uint8_t*)data) == HAL_OK) ? true : false;
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




