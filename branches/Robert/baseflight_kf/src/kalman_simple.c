/*
 * kalman.c
 *
 *  Created on: 25.11.2012
 *      Author: rob
 */

#include "stdint.h"
#include "kalman_simple.h"

typedef struct
{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //value
	float p; //estimation error covariance
} kalman_state;

// accelerometer
kalman_state kax;
kalman_state kay;
kalman_state kaz;

// gyro
kalman_state kgx;
kalman_state kgy;
kalman_state kgz;

void kalman_init(kalman_state* state, float q, float r, float p, float intial_value)
{
	state->q = q;
	state->r = r;
	state->p = p;
	state->x = intial_value;
}

void kalman_update(kalman_state* state, float measurement)
{
	//prediction update
	//omit x = x
	float k; //kalman gain
	state->p = state->p + state->q;

	//measurement update
	k = state->p / (state->p + state->r);
	state->x = state->x + k * (measurement - state->x);
	state->p = (1 - k) * state->p;
}


void initKalmanAccel(float ax, float ay, float az)
{
// small jakub frame
#define Q 0.0625		// process noise covariance
#define	R 1.0		// measurement noise covariance
#define P 0.22		// estimation error covariance

	kalman_init(&kax, Q, R, P, ax);
	kalman_init(&kay, Q, R, P, ay);
	kalman_init(&kaz, Q, R, P, az);
#undef Q
#undef R
#undef P
}

void initKalmanGyro(float gx, float gy, float gz)
{

// small jakub frame
#define Q 1.0 	// process noise covariance
#define	R 0.0625	// measurement noise covariance
#define P 0.22	// estimation error covariance

	kalman_init(&kgx, Q, R, P, gx);
	kalman_init(&kgy, Q, R, P, gy);
	kalman_init(&kgz, Q, R, P, gz);
#undef Q
#undef R
#undef P
}

inline float kalman_filter_step(kalman_state* state, int16_t val)
{
	float measurement = (float) val;
	kalman_update(state, measurement);
	return (state->x + 0.5f);
}

void accKalmanFilterStep(int16_t accels[3])
{
	static int _init = 0;
	if (!_init)
	{
		_init = 1;
		initKalmanAccel(accels[0], accels[1], accels[2]);
	}
	else
	{
		accels[0] = kalman_filter_step(&kax, accels[0]);
		accels[1] = kalman_filter_step(&kay, accels[1]);
		accels[2] = kalman_filter_step(&kaz, accels[2]);
	}
}

void gyroKalmanFilterStep(int16_t gyros[3])
{
	static int _init = 0;
	if (!_init)
	{
		_init = 1;
		initKalmanGyro(gyros[0], gyros[1], gyros[2]);
	}
	else
	{
		gyros[0] = kalman_filter_step(&kgx, gyros[0]);
		gyros[1] = kalman_filter_step(&kgy, gyros[1]);
		gyros[2] = kalman_filter_step(&kgz, gyros[2]);
	}
}
