#include <BMP390.h>

#define debug_print(...)


using namespace std;



//------------------------------------------------------------------------------
bool BMP390::init()
{
	debug_print("bmp390 init\r");
	uint8_t id;
	if(!func_rd(ADDR_ID, &id, sizeof(id)) || (id != ChipId))
		return false;

	#pragma pack(push, 1)
	struct
	{
		uint16_t t1;
		uint16_t t2;
		int8_t t3;
		int16_t p1;
		int16_t p2;
		int8_t p3;
		int8_t p4;
		uint16_t p5;
		uint16_t p6;
		int8_t p7;
		int8_t p8;
		int16_t p9;
		int8_t p10;
		int8_t p11;
	} raw_calib;
	#pragma pack(pop)

	if(!func_rd(ADDR_CALIB, &raw_calib, sizeof(raw_calib)))
		return false;

	calib.t1 = ((float)raw_calib.t1)/0x1p-8f;
	calib.t2 = ((float)raw_calib.t2)/0x1p30f;
	calib.t3 = ((float)raw_calib.t3)/0x1p48f;

	calib.p1 = ((float)raw_calib.p1-0x1p14f)/0x1p20f;
	calib.p2 = ((float)raw_calib.p2-0x1p14f)/0x1p29f;
	calib.p3 = ((float)raw_calib.p3)/0x1p32f;
	calib.p4 = ((float)raw_calib.p4)/0x1p37f;
	calib.p5 = ((float)raw_calib.p5)/0x1p-3f;
	calib.p6 = ((float)raw_calib.p6)/0x1p6f;
	calib.p7 = ((float)raw_calib.p7)/0x1p8f;
	calib.p8 = ((float)raw_calib.p8)/0x1p15f;
	calib.p9 = ((float)raw_calib.p9)/0x1p48f;
	calib.p10 = ((float)raw_calib.p10)/0x1p48f;
	calib.p11 = ((float)raw_calib.p11)/0x1p65f;

	return func_wr(ADDR_OSR, 0x1D) && func_wr(ADDR_IIR, 0x6) && func_wr(ADDR_PWR_CTRL, PwrCtrl);
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
float BMP390::calc_temp(uint32_t raw_temp)
{
	float t1 = (float)(raw_temp-calib.t1);
	float t2 = (float)(t1*calib.t2);
	calib.temp = t2 + t1*t1*calib.t3;
	return calib.temp;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
float BMP390::calc_pres(uint32_t raw_pres)
{
	float p1 = calib.p6*calib.temp;
	float p2 = calib.p7*calib.temp*calib.temp;
	float p3 = calib.p8*calib.temp*calib.temp*calib.temp;
	float out1 = calib.p5 + p1 + p2 + p3;
	p1 = calib.p2*calib.temp;
	p2 = calib.p3*calib.temp*calib.temp;
	p3 = calib.p4*calib.temp*calib.temp*calib.temp;
	float out2 = (float)raw_pres*(calib.p1 + p1 + p2 + p3);
	p1 = (float)raw_pres*(float)raw_pres;
	p2 = calib.p9 + calib.p10*calib.temp;
	p3 = p1*p2;
	float p4 = p3 + ((float)raw_pres*(float)raw_pres*(float)raw_pres)*calib.p11;
	return out1 + out2 + p4;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
bool BMP390::get(out_t& val)
{
	debug_print("bmp390 get\r");

	func_wr(ADDR_PWR_CTRL, PwrCtrl);

	uint8_t status;
	if(!func_rd(ADDR_STATUS, &status, 1))
		return false;

	debug_print("bmp390 status: 0x%02X\r",status);

	uint8_t data[6];
	if(((status & 0x60) != 0x60) || !func_rd(ADDR_DATA, data, sizeof(data)))
		return false;

	uint32_t raw_pres = data[0] + (data[1]<<8) + (data[2]<<16);
	uint32_t raw_temp = data[3] + (data[4]<<8) + (data[5]<<16);

	val.temp = calc_temp(raw_temp);
	val.pres = calc_pres(raw_pres);

	debug_print("bmp390 temp: %8.3f, pres: %10.2f\r", (double)val.temp, (double)val.pres);
	return true;
}
//------------------------------------------------------------------------------

