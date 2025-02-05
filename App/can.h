#pragma once

#include <can_messages.h>

void can_init(void);
bool can_send_dat(uint16_t id, void* data, const uint8_t len);
bool can_send_val(uint16_t id, float val, bool valid, bool test, bool calibr);


