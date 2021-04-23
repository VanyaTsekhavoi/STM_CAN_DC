#ifndef DC_MOTOR_H__
#define DC_MOTOR_H__

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CLIP(a, l, h) MAX(MIN((a), (h)), (l))
#define CLIPS(a, h) CLIP((a), -(h), (h))

typedef struct
{
    float instantVoltage;
    float instantCurrent;
    float instantVelocity;
    float instantAngle;
} dc_motor_t;

void dc_motor_init(void);
void dc_motor_poll(void);
void velocity(int64_t angle, int64_t lastAngle);
void angle(double sin, double cos);

#endif
