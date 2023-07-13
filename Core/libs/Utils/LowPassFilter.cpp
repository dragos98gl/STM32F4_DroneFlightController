/*
 * LowPassFilter.cpp
 *
 *  Created on: Sep 26, 2023
 *      Author: Dragos
 */

#include <math.h>
#include "LowPassFilter.hpp"
#include "utils_functions.hpp"

LowPassFilter::LowPassFilter(float wo, float Te, float u0)
{
	this->Te = Te;
	this->wo = drone::utils::functions::HzToRads(wo);
	this->u0(u0);

	getTf();
}

void LowPassFilter::u0(float u0)
{
	u[0] = u0;
	u[1] = u0;
	y[0] = u0;
	y[1] = u0;
}

void LowPassFilter::getTf()
{
	b[0] = wo/(2/Te+wo);
	b[1] = wo/(2/Te+wo);

	a[0] = (wo-2/Te)/(2/Te+wo);
}

float LowPassFilter::lsim(float input)
{
  y[0] = y[1];

  u[0] = u[1];
  u[1] = input;

  y[1] = -a[0] * y[0] + b[1] * u[0] + b[0] * u[1];

  return y[1];
}
