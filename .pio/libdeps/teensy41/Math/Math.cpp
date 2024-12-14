#include "Math.h"
#include "mbed.h"

const float gPI = 3.1415926536f;

float convertRange( float in, float oldMin, float oldMax, float newMin, float newMax ){
    return ( in - oldMin ) * ( newMax - newMin ) / ( oldMax - oldMin ) + newMin;
}

float convertRangePlusMinusPi( float deg ){
    while ( ( deg > gPI ) || ( deg < -gPI ) ){
        while ( deg > gPI ){
            deg -= 2.0f * gPI;
        }
        while ( deg < -gPI ){
            deg += 2.0f * gPI;
        }
    }
    
    return deg;
}

bool isNear( float a, float b, float error ){
    return ( abs( a - b ) < error );
}
