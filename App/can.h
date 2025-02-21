#pragma once

#pragma pack(push, 1)
typedef struct
{
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
} can_queue_item_t;
#pragma pack(pop)


bool can_init(void);
bool can_send_dat(uint16_t id, const void* data, const uint8_t len);
bool can_send_val(uint16_t id, float val, bool valid, bool test, bool calibr);


