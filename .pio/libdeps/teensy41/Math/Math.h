#ifndef INCLUDED_MATH_H
#define INCLUDED_MATH_H

extern const float gPI;

float convertRange( float in, float oldMin, float oldMax, float newMin, float newMax );
float convertRangePlusMinusPi( float deg );
bool isNear( float a, float b, float error );

#endif
