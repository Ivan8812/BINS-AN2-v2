#pragma once

#include <math.h>
#include <stdint.h>


float altitude(float pres)
{
	static const float P0 = 101325.0f;
	static const float T0 = 288.15f;
	static const float L = 6.5e-3f;
	static const float R = 8.31446f;
	static const float G = 9.80665f;
	static const float M = 28.98e-3f;
	return T0/L*(1.0f-powf((pres/P0), (R*L/G/M)));
}

float air_speed(float pres)
{
	static const float Ks = 1225.057f;
	static const float P0 = 101325.0f;
	float tmp = 5.0f*(powf((pres/P0) + 1.0f, 2.0f/7.0f) - 1.0f);
	if(tmp >= 0.0f)
		return Ks*sqrtf(tmp);
	else
		return -Ks*sqrtf(-tmp);
}

float vert_speed(float pres, float dpdt)
{
	static const float P0 = 101325.0f;
	static const float T0 = 288.15f;
	static const float L = 6.5e-3f;
	static const float R = 8.31446f;
	static const float G = 9.80665f;
	static const float M = 28.98e-3f;
	return -T0*powf(pres/P0, R*L/(G*M))*R*dpdt/(G*M*pres);
}

