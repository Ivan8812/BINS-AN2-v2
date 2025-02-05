#include "Packer.h"

using namespace std;


//------------------------------------------------------------------------------
void Packer::parse_inbox()
{
	static uint16_t end_pos = MinPackSize;
	static uint8_t cs = 0;

	while(pos < buf_in.level())
	{
		if(pos == 0)
		{
			if(buf_in[0] == PackHeader)
			{
				cs = 0x0;
				end_pos = MinPackSize;
			}
			else
			{
				buf_in.clear(1);
				continue;
			}
		}
		else if(pos == 3)
			end_pos += buf_in[3];

		cs ^= buf_in[pos++];
		if(pos == end_pos)
		{
			buf_in.clear(1);
			uint8_t id = buf_in[0];
			uint8_t cntr = buf_in[1];
			uint8_t len = buf_in[2];
			if(cs)
			{ // ошибка контрольной суммы пакета
				stat.broken_cs++;
				send(PidBroken, &id, sizeof(id));
			}
			else
			{ // пакет успешно принят
				for(uint16_t i=0; i<len; i++)
					payload[i] = buf_in[i+3];
				buf_in.clear(len+4);
				if(cntr != check_cntr)
				{
					check_cntr = cntr;
					if(stat.rxd || stat.broken_cs)
						stat.broken_cntr++;
				}
				stat.rxd++;
				check_cntr++;
				if(hnd_map.find(id) == hnd_map.end())
					send(PidUnknown, &id, sizeof(id));
				else
					hnd_map.at(id)(payload, len);
			}
			pos = 0;
		}
	}
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool Packer::send(const uint8_t pid, const void* payload, const uint8_t len)
{
	bool res = false;

  if(buf_out.avail() >= ((uint32_t)len+5))
  {
    buf_out.push(PackHeader);
    buf_out.push(pid);
    buf_out.push(pack_cntr);
    buf_out.push(len);
    uint8_t cs = PackHeader ^ pid ^ len ^ pack_cntr;
    for(int i=0; i<len; i++)
    {
    	buf_out.push(((uint8_t*)payload)[i]);
      cs ^= ((uint8_t*)payload)[i];
    }
    buf_out.push(cs);
    pack_cntr++;
    stat.txd++;
    res = true;
  }
  else
  {
   	stat.tx_errors++;
  }

	return res;
}
//------------------------------------------------------------------------------

