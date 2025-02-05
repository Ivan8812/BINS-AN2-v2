#include <MTI.h>
#include <array>

#if MTI_DEBUG
  #include "rs422_port.h"
	#define debug_print rs422_printf
#else
	static void debug_print(const char* fmt...) {}
#endif

using namespace std;



//------------------------------------------------------------------------------
void  MTI::reset()
{ 
	debug_print("\r\rMTI reset\r");
	if(mti_state != WAKEUP)
		send_pack(MID_RESET);
	pack_state = PACK_PREAMBLE;
	mti_state = WAKEUP;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void MTI::serve_tim(uint32_t dt)
{
	if(cmd_timeout && (cmd_mid || cmd_handler) && (cmd_timeout != MaxTimeout))
	{
		if(cmd_timeout > dt)
		{
			uint32_t t = cmd_timeout - dt;
			cmd_timeout = t;
		}
		else
		{
			cmd_mid = std::nullopt;
			cmd_handler = nullptr;
		}
	}
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool MTI::send_cmd(uint8_t mid, const void* payload, uint16_t len, handler_t handler, uint32_t timeout)
{
	if(cmd_mid || cmd_handler)
		return false;

	if(handler)
	{
		cmd_mid = mid;
		cmd_handler = handler;
		cmd_timeout = timeout;
	}

	if(!send_pack(mid, payload, len))
	{
		cmd_mid = nullopt;
		cmd_handler = nullptr;
		cmd_timeout = 0;
		return false;
	}

	return true;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool MTI::set_profile(uint16_t prof)
{
	BE<uint16_t> prof_be = prof;
	return send_pack(MID_SET_PROFILE, &prof_be, sizeof(uint16_t));
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool MTI::set_no_rotation(uint16_t sec)
{
	BE<uint16_t> sec_be = sec;
	return send_pack(MID_SET_NO_ROT, &sec_be, sizeof(uint16_t));
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void MTI::parse_inbox()
{
	static uint16_t data_len = 0;
	static uint16_t curr_len = 0;
	static uint8_t mid = 0x0;
	static uint8_t cs = 0x0;
	static uint16_t pos = 0;

	while (pos < buf_in.level())
	{
		if (pack_state == PACK_PREAMBLE)
		{
			if (buf_in[0] == PREAMBLE)
			{
				cs = 0x0;
				pos = 1;
				pack_state = PACK_LENGTH;
			}
			else
				buf_in.clear(1);
		}
		else
		{
			cs += buf_in[pos++];
			switch (pack_state)
			{
			case PACK_LENGTH:
				if ((pos == 4) && (buf_in[3] != 0xFF))
					data_len = buf_in[3];
				else if (pos == 6)
					data_len = (buf_in[4] << 8) + buf_in[5];
				else
					break;
				curr_len = 0;
				mid = buf_in[2];
				pack_state = (data_len && (data_len <= MaxLen)) ? PACK_DATA : PACK_COMPLETE;
				break;
			case PACK_DATA:
				if (++curr_len == data_len)
					pack_state = PACK_COMPLETE;
				break;
			case PACK_COMPLETE:
				if (data_len > MaxLen)
				{
					stat.oversize++;
					debug_print("ovsz%02X\r", mid);
					buf_in.clear(4);
				}
				if (cs)
				{
					stat.broken++;
					debug_print("brk%02X\r", mid);
					buf_in.clear(1);
				}
				else
				{
					stat.rxd++;
					debug_print("rx%02X\r", mid);
					buf_in.clear((data_len<255) ? 4 : 6);
					process_pack(mid, data_len);
					buf_in.clear(data_len+1);
				}
				[[fallthrough]];
			default:
				pos = 0;
				pack_state = PACK_PREAMBLE;
				break;
			}
		}
	}
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
bool MTI::send_pack(uint8_t mid, const void* payload, const uint16_t len)
{
	uint8_t buf[len+6];

	uint8_t cs = 0x0;
	uint16_t pos = 0;
	auto push_cs = [&cs, &pos, &buf, this](const uint8_t b) {buf[pos++] = b; cs -= b; };

	buf[pos++] = PREAMBLE;
	push_cs(0xFF);
	push_cs(mid);
	if (len < 255)
		push_cs((uint8_t)len);
	else
	{
		push_cs(0xFF);
		push_cs((uint8_t)(len >> 8));   // don't forget about BE
		push_cs((uint8_t)(len & 0xFF)); //
	}
	for (int i = 0; i < len; i++)
		push_cs(((uint8_t*)payload)[i]);
	buf[pos++] = cs;

	debug_print("tx%02X\r",mid);

	bool res = write(buf, pos);
	if(res)
		stat.txd++;
	else
		stat.tx_errors++;

	return res;
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void MTI::process_pack(uint8_t mid, const uint16_t len)
{
	if(mid == MID_WARNING)
		on_warning();

	if(cmd_mid && (mid == (cmd_mid.value()+1)) && cmd_handler)
	{
		cmd_mid = nullopt;
		handler_t handler(cmd_handler);
		cmd_handler = nullptr;
		uint8_t buf[len];
		for(uint16_t i=0; i<len; i++)
			buf[i] = buf_in[i];
		handler(buf, len);
	}
	else
	{
		switch (mti_state)
		{
		case WAKEUP:
			if(mid == MID_WAKEUP)
			{
				send_pack(MID_WAKEUP+1);
				mtdata_cfg_t cfg[mtdata_cfg.size()];
				mtdata_cfg_t* cfg_p = cfg;
				for(auto& mtdata_cfg_i: mtdata_cfg)
				{
					cfg_p->data_id = mtdata_cfg_i.data_id;
					cfg_p->rate = mtdata_cfg_i.rate;
					cfg_p++;
				}
				send_pack(MID_CONFIG, &cfg, sizeof(cfg));
				mti_state = CONFIG;
			}
			break;
		case CONFIG:
			if (mid == (MID_CONFIG + 1))
				set_profile(PROF_HIGH_MAG);
			else if (mid ==(MID_SET_PROFILE+1))
				send_pack(MID_GOTOMEASURE);
			else if(mid == (MID_GOTOMEASURE+1))
				mti_state = RUN;
			else
				reset();
			break;
		case RUN:
			if ((mid == MID_MTDATA) && mtd_handler)
			{
				static uint8_t mtdata[MaxLen] __attribute__((aligned(8)));
				buf_in.get(0, mtdata, len);
				mtd_handler(mtdata, len);
			}
			break;
		default:
			reset();
			break;
		}
	}
}
//------------------------------------------------------------------------------


