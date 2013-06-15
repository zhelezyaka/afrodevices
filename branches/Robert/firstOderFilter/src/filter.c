/*
 * filter.c
 *
 *  Created on: 25.11.2012
 *      Author: robert
 *
 */

#include <stdint.h>
#include "board.h"
#include "filter.h"


void accelFilterStep(int16_t acc[3])
{
	static int _init = 0;
	float val;

	if (!_init)
	{
		_init = 1;
		initFirstOrderFilter();
	}
	else
	{
		val = acc[0];
		acc[0] = (int16_t)firstOrderFilter(val, &firstOrderFilters[ACCEL_X_LOWPASS]);
		val = acc[1];
		acc[1] = (int16_t)firstOrderFilter(val, &firstOrderFilters[ACCEL_Y_LOWPASS]);
		val = acc[2];
		acc[2] = (int16_t)firstOrderFilter(val, &firstOrderFilters[ACCEL_Z_LOWPASS]);

	}
}

///////////////////////////////////////////////////////////////////////////////
// filter copyright john ihlein
//
// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////

#define ACCEL_X_LOWPASS_TAU         0.05f
#define ACCEL_X_LOWPASS_SAMPLE_TIME 0.002f
#define ACCEL_X_LOWPASS_A           (2.0f * ACCEL_X_LOWPASS_TAU / ACCEL_X_LOWPASS_SAMPLE_TIME)
#define ACCEL_X_LOWPASS_GX1         (1.0f / (1.0f + ACCEL_X_LOWPASS_A))
#define ACCEL_X_LOWPASS_GX2         (1.0f / (1.0f + ACCEL_X_LOWPASS_A))
#define ACCEL_X_LOWPASS_GX3    ((1.0f - ACCEL_X_LOWPASS_A) / (1.0f + ACCEL_X_LOWPASS_A))

///////////////////////////////////////

#define ACCEL_Y_LOWPASS_TAU         0.05f
#define ACCEL_Y_LOWPASS_SAMPLE_TIME 0.002f
#define ACCEL_Y_LOWPASS_A           (2.0f * ACCEL_Y_LOWPASS_TAU / ACCEL_Y_LOWPASS_SAMPLE_TIME)
#define ACCEL_Y_LOWPASS_GX1         (1.0f / (1.0f + ACCEL_Y_LOWPASS_A))
#define ACCEL_Y_LOWPASS_GX2         (1.0f / (1.0f + ACCEL_Y_LOWPASS_A))
#define ACCEL_Y_LOWPASS_GX3         ((1.0f - ACCEL_Y_LOWPASS_A) / (1.0f + ACCEL_Y_LOWPASS_A))

///////////////////////////////////////

#define ACCEL_Z_LOWPASS_TAU         0.05f
#define ACCEL_Z_LOWPASS_SAMPLE_TIME 0.002f
#define ACCEL_Z_LOWPASS_A           (2.0f * ACCEL_Z_LOWPASS_TAU / ACCEL_Z_LOWPASS_SAMPLE_TIME)
#define ACCEL_Z_LOWPASS_GX1         (1.0f / (1.0f + ACCEL_Z_LOWPASS_A))
#define ACCEL_Z_LOWPASS_GX2         (1.0f / (1.0f + ACCEL_Z_LOWPASS_A))
#define ACCEL_Z_LOWPASS_GX3         ((1.0f - ACCEL_Z_LOWPASS_A) / (1.0f + ACCEL_Z_LOWPASS_A))


///////////////////////////////////////////////////////////////////////////////

void initFirstOrderFilter()
{
	extern uint16_t acc_1G;

    firstOrderFilters[ACCEL_X_LOWPASS].gx1 = ACCEL_X_LOWPASS_GX1;
    firstOrderFilters[ACCEL_X_LOWPASS].gx2 = ACCEL_X_LOWPASS_GX2;
    firstOrderFilters[ACCEL_X_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL_X_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[ACCEL_Y_LOWPASS].gx1 = ACCEL_Y_LOWPASS_GX1;
	firstOrderFilters[ACCEL_Y_LOWPASS].gx2 = ACCEL_Y_LOWPASS_GX2;
	firstOrderFilters[ACCEL_Y_LOWPASS].gx3 = ACCEL_Y_LOWPASS_GX3;
	firstOrderFilters[ACCEL_Y_LOWPASS].previousInput  = 0.0f;
    firstOrderFilters[ACCEL_Y_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    firstOrderFilters[ACCEL_Z_LOWPASS].gx1 = ACCEL_Z_LOWPASS_GX1;
	firstOrderFilters[ACCEL_Z_LOWPASS].gx2 = ACCEL_Z_LOWPASS_GX2;
	firstOrderFilters[ACCEL_Z_LOWPASS].gx3 = ACCEL_Z_LOWPASS_GX3;
	firstOrderFilters[ACCEL_Z_LOWPASS].previousInput  = acc_1G;
    firstOrderFilters[ACCEL_Z_LOWPASS].previousOutput = acc_1G;
}

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////

