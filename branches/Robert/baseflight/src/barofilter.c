/*
 * barofilter.c
 *
 *  Created on: 27.01.2013
 *      Author: rob
 */

#include "stdint.h"
#include "barofilter.h"


typedef struct
{
	float q;	// process noise covariance
	float r;	// measurement noise covariance
	float x;	// value
	float p;	// estimation error covariance
} kalmanState_t;

// barometer
kalmanState_t kbaro;

static void initSimpleKalmanState(kalmanState_t* state, float q, float r, float p, float intial_value)
{
	state->q = q;
	state->r = r;
	state->p = p;
	state->x = intial_value;
}

void baroKalmanfilterStep(int32_t *baro)
{
	float k;	// kalman gain
	float measurement = *baro;
	kbaro.p = kbaro.p + kbaro.q;

	//measurement update
	k = kbaro.p / (kbaro.p + kbaro.r);
	kbaro.x = kbaro.x + k * (measurement - kbaro.x);
	kbaro.p = (1 - k) * kbaro.p;

	*baro = (int32_t) kbaro.x;
}

void initKalmanBaro()
{
//#define Q 0.01	// process noise covariance
//#define	R 2.0	// measurement noise covariance
//#define P 0.2		// estimation error covariance

//#define Q 0.0625	// process noise covariance
//#define	R 4.0	// measurement noise covariance
//#define P 0.67	// estimation error covariance

#define Q 4.0 		// process noise covariance
#define	R 0.625		// measurement noise covariance
#define P 0.42		// estimation error covariance
	// up and downs in meters ... :(
//#define Q 1e-5 	// process noise covariance
//#define	R 1e-1	// measurement noise covariance
//#define P 0.42	// estimation error covariance

	initSimpleKalmanState(&kbaro, Q, R, P, 0);

#undef Q
#undef R
#undef P
}
