#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include "CBuf.h"

class Packer
{
public:
	const uint8_t PackHeader = 0xA5;
	static const uint8_t PidUnknown = 0xFF;
	static const uint8_t PidBroken = 0xFE;

	using handler_t = std::function<void(const void*, const uint8_t)>;
	using hnd_map_t = std::map<uint8_t, handler_t>;
	using stat_t = struct
	{
		uint32_t txd;
		uint32_t rxd;
		uint32_t tx_errors;
		uint32_t broken_cs;
		uint32_t broken_cntr;
	};

	Packer(const hnd_map_t& hnd_map = {}) : hnd_map(hnd_map) {};

	void parse_inbox();

	bool inbox_full() {return buf_in.avail() ? false : true;}
	void inbox_push(const uint8_t x) {buf_in.push(x);}

	bool outbox_empty() {return buf_out.empty();}
	uint8_t outbox_pop() {return buf_out.pop();}

	bool send(const uint8_t pid, const void* payload = nullptr, const uint8_t len = 0);
	const stat_t& get_stat() {return stat;}

private:
	static const uint16_t MinPackSize = 5;

	Packer(const Packer &other);
	Packer& operator=(const Packer &rhs);
	Packer(Packer &&rhs);

	uint16_t pos = 0;
	uint8_t check_cntr = 0;
	uint8_t pack_cntr = 0;
	uint8_t payload[256] = {0,};
	stat_t stat = {0,0,0,0,0};

	const hnd_map_t hnd_map;

	CBuf<uint8_t, 9> buf_in;
	CBuf<uint8_t, 9> buf_out;
};

