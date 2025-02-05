#pragma once

#define BMP_DEBUG 0

#include <stdint.h>
#include <functional>

class BMP390 final
{
public:
	using func_rd_t = std::function<bool(uint8_t addr, void* dst, uint8_t n)>;
	using func_wr_t = std::function<bool(uint8_t addr, uint8_t data)>;
	using out_t = struct
	{
		float temp;
		float pres;
	};

	BMP390(func_rd_t func_rd, func_wr_t func_wr) : func_rd(func_rd), func_wr(func_wr) {	};

	bool init();
	bool get(out_t& val);

private:
	static const uint8_t ChipId = 0x60;
	static const uint8_t PwrCtrl = 0x13;
	enum
	{
		ADDR_ID = 0x0,
		ADDR_STATUS = 0x03,
		ADDR_DATA = 0x04,
		ADDR_PWR_CTRL = 0x1B,
		ADDR_OSR = 0x1C,
		ADDR_IIR = 0x1F,
		ADDR_CALIB = 0x31,
	};

	using calib_t = struct
	{
		float t1;
		float t2;
		float t3;
		float p1;
		float p2;
		float p3;
		float p4;
		float p5;
		float p6;
		float p7;
		float p8;
		float p9;
		float p10;
		float p11;
		float temp;
	};

	calib_t calib = {0,};
	func_rd_t func_rd;
	func_wr_t func_wr;

	BMP390(const BMP390 &other) = delete;
	BMP390(BMP390 &&other) = delete;
	BMP390& operator=(const BMP390 &other) = delete;

	float calc_temp(uint32_t raw_temp);
	float calc_pres(uint32_t raw_pres);
};

