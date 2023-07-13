/*
 * LowPassFilter.hpp
 *
 *  Created on: Sep 26, 2023
 *      Author: Dragos
 */

#ifndef LIBS_UTILS_LOWPASSFILTER_HPP_
#define LIBS_UTILS_LOWPASSFILTER_HPP_

class LowPassFilter
{
  private:
    float Te;

    float wo;

    float y[2];
    float u[2];

    void getTf();

  public:
    float a[1];
    float b[2];

    void u0(float u0);
    LowPassFilter(float wo, float Te, float u0 = 0);
    float lsim(float input);
};

#endif /* LIBS_UTILS_LOWPASSFILTER_HPP_ */
