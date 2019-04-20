/*
** uNav
** File description:
** This is a motors control project in development by Officine Robotiche.
*/

#pragma once
#define M_PI_F       3.14159265358979323846264338328f      /* pi */
#define LPF_ALPHA(dt, fc)       (dt / (dt + 1.0f / (2.0f * M_PI_F * fc)))

static inline float fboundf(float min, float max, float value){
    if(value > max){
        return max;
    } else if(value < min){
        return min;
    }
    return value;
}


