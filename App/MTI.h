#pragma once

#define MTI_DEBUG 		0

#include <cstdint>
#include <functional>
#include <optional>
#include <initializer_list>
#include <CBuf.h>
#include <BE.h>


class MTI
{
public:
	using handler_t = std::function<void(void*, const uint16_t)>;
	using cfg_t = struct{uint16_t data_id; uint16_t rate;};

	MTI(std::initializer_list<cfg_t>& cfg, handler_t mtd_handler) : mtdata_cfg(cfg), mtd_handler(mtd_handler) {};
	virtual ~MTI() {}

	void reset();

		// !!! Function to fill data in buf_in must be defined in the derived class !!!
		// parse_inbox function parses only data contained in buf_in
	void parse_inbox();

	void serve_tim(uint32_t dt); // must be called periodically

	using stat_t = struct
	{
		uint32_t txd;
		uint32_t rxd;
		uint32_t tx_errors;
		uint32_t broken;
		uint32_t unknown;
		uint32_t oversize;
	};
	const stat_t& get_stat() { return stat; }

	bool set_no_rotation(uint16_t sec);

protected:
	enum
	{
		MID_RESET = 0x40,
		MID_WAKEUP = 0x3E,
		MID_WARNING = 0x43,
		MID_GOTOCONFIG = 0x30,
		MID_GOTOMEASURE = 0x10,
		MID_CONFIG = 0xC0,
		MID_MTDATA = 0x36,
		MID_SET_PROFILE = 0x64,
		MID_SET_NO_ROT = 0x22,
		MID_SET_ALIGN = 0xEC,
	};
	enum
	{
		PROF_GEN = 11,
		PROF_GEN_MAG = 13,
		PROF_HIGH_MAG = 51,
	};
	static const uint16_t MaxLen = 1000;
	static const uint16_t MaxPayload = 100;
	static const uint32_t MaxTimeout = 0xFFFFFFFF;

	CBuf<uint8_t, 10> buf_in;

	bool send_pack(uint8_t mid, const void* payload = nullptr, const uint16_t len=0);
	bool send_cmd(uint8_t mid, const void* payload = nullptr, uint16_t len=0, handler_t = nullptr, uint32_t timeout = MaxTimeout);
	bool set_profile(uint16_t prof);
	bool set_alignment(float qw, float qx, float qy, float qz);

private:
	MTI(const MTI &other) = delete;
	MTI& operator=(const MTI &rhs) = delete;
	MTI(MTI &&rhs) = delete;

	enum {PREAMBLE = 0xFA};

	enum {WAKEUP, CONFIG, RUN} mti_state = WAKEUP;
	enum {PACK_PREAMBLE, PACK_LENGTH, PACK_DATA, PACK_COMPLETE} pack_state = PACK_PREAMBLE;

#pragma pack(push, 1)
	using mtdata_cfg_t = struct { BE<uint16_t> data_id; BE<uint16_t> rate; };
#pragma pack(pop)

	const std::initializer_list<cfg_t> mtdata_cfg;
	const handler_t mtd_handler;

	stat_t stat = { 0, };
	std::optional<uint8_t> cmd_mid = std::nullopt;
	handler_t cmd_handler = nullptr;
	volatile uint32_t cmd_timeout = 0;

	void process_pack(uint8_t mid, const uint16_t len);
	virtual bool write(void* data, uint32_t len) = 0;
	virtual void on_warning() {}
};

