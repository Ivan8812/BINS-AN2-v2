#pragma once
#include <cstdint>

template<typename data_t = uint8_t, const int SizeFactor=8>
class CBuf
{
	static const uint32_t Len = 1 << SizeFactor;
	static const uint32_t Mask = Len-1;

	volatile uint32_t head=0;
	volatile uint32_t tail=0;
	data_t data[Len] = { 0x0 };
public:
	using block_t = struct { data_t* p; uint32_t n; };
	//----------------------------------------------
	bool empty() 
	{ 
		return head == tail; 
	}
	//----------------------------------------------
	bool full()
	{
		return level() == Mask;
	}
	//----------------------------------------------
	uint32_t level() 
	{ 
		return (tail - head) & Mask; 
	}
	//----------------------------------------------
	uint32_t avail() 
	{ 
		return Mask - level(); 
	}
	//----------------------------------------------
	void push(const data_t& x) 
	{ 
		data[tail] = x; 
		tail = ((tail + 1) & Mask); 
	}
	//----------------------------------------------
	const data_t pop() 
	{ 
		data_t tmp = data[head]; 
		head = ((head + 1) & Mask); 
		return tmp; 
	}
	//----------------------------------------------
	void clear(const uint32_t n) 
	{
		head = ((head + n) & Mask);
	}
	//----------------------------------------------
	void resize(const uint32_t n) 
	{ 
		tail = ((tail + n) & Mask); 
	}
	//----------------------------------------------
	data_t& operator[] (const uint32_t i)
	{ 
		return data[(head + i) & Mask]; 
	}
	//----------------------------------------------
	template<typename type_t>
	type_t get(const uint32_t pos)
	{ 
		type_t tmp = {};
		for (uint32_t i = 0; i < (sizeof(type_t) + sizeof(data_t) - 1) / sizeof(data_t); i++)
			((data_t*)&tmp)[i] = data[(head + pos + i) & Mask];
		return tmp;
	}
	//----------------------------------------------
	void get(const uint32_t pos, void* dst, const uint16_t len)
	{
		for (uint32_t i = 0; i < (len + sizeof(data_t) - 1) / sizeof(data_t); i++)
			((data_t*)dst)[i] = data[(head + pos + i) & Mask];
	}
};

