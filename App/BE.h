#pragma once

template<typename data_t>
class BE
{
	static const int N = sizeof(data_t);
	uint8_t d[N] = {};

	void swap(uint8_t* dst, const uint8_t* src)
	{
		for(int i=0; i<N; i++)
			dst[i] = src[N-i-1];
	}
public:
	BE(const data_t x)
	{
		swap(d, (uint8_t*)&x);
	}

	BE() {};

	BE& operator=(const data_t x)
	{
		swap(d, (uint8_t*)&x);
		return *this;
	}

	data_t value()
	{
		data_t x = {};
		swap((uint8_t*)&x, d);
		return x;
	}

	operator data_t() {return value();}

	uint8_t* data() const
	{
		return d;
	}
};
