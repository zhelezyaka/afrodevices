/*
 * AlphaBetaFilter.c
 *
 *  Created on: 12.01.2013
 *      Author: rob
 */

/** A simple alpha-beta filter example by Adrian Boeing
 www.adrianboeing.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "AlphaBetaFilter.h"


typedef struct
{
	float alpha;	// alpha value (effects x, eg pos)
	float beta;		// beta value (effects v, eg vel)
	float xk_1;		// current x-estimate
	float vk_1;		// current v-estimate
} AlphaBeta;

	AlphaBeta ab_ax;
	AlphaBeta ab_ay;
	AlphaBeta ab_az;

	AlphaBeta ab_gx;
	AlphaBeta ab_gy;
	AlphaBeta ab_gz;

void InitializeAlphaBeta(float x_measured, float alpha, float beta, AlphaBeta* pab)
{
	pab->xk_1 = x_measured;
	pab->vk_1 = 0;
	pab->alpha = alpha;
	pab->beta = beta;
}

void AlphaBetaFilter(float x_measured, float dt, AlphaBeta* pab)
{
	float xk_1 = pab->xk_1;
	float vk_1 = pab->vk_1;
	float alpha = pab->alpha;
	float beta = pab->beta;

	float xk; // current system state (ie: position)
	float vk; // derivative of system state (ie: velocity)
	float rk; // residual error

	// update our (estimated) state 'x' from the system (ie pos = pos + vel (last).dt)
	xk = xk_1 + dt * vk_1;
	// update (estimated) velocity
	vk = vk_1;
	// what is our residual error (mesured - estimated)
	rk = x_measured - xk;
	// update our estimates given the residual error.
	xk = xk + alpha * rk;
	vk = vk + beta / dt * rk;
	// finished!

	// now all our "currents" become our "olds" for next time
	pab->vk_1 = vk;
	pab->xk_1 = xk;
}


void initAlphaBetafilter()
{
	// initialize the AB filters
	InitializeAlphaBeta(0, 0.5, 0.001, &ab_ax);	// x position
	InitializeAlphaBeta(0, 0.5, 0.001, &ab_ay);	// y position
	InitializeAlphaBeta(0, 0.5, 0.001, &ab_az);	// z position

	InitializeAlphaBeta(0, 0.65, 0.009, &ab_gx);	// x position
	InitializeAlphaBeta(0, 0.65, 0.009, &ab_gy);	// y position
	InitializeAlphaBeta(0, 0.65, 0.009, &ab_gz);	// z position
}

void callABfilter(int16_t *x_measured, float dt, AlphaBeta* pab)
{
	float m = *x_measured;
	AlphaBetaFilter(m, dt, pab);
	*x_measured = (int16_t) pab->xk_1;
}

void accelABfilterStep(int16_t acc[3], float dt)
{
	callABfilter(&acc[0], dt, &ab_ax);
	callABfilter(&acc[1], dt, &ab_ay);
	callABfilter(&acc[2], dt, &ab_az);
}

void gyroABfilterStep(int16_t gyros[3], float dt)
{
	callABfilter(&gyros[0], dt, &ab_gx);
	callABfilter(&gyros[1], dt, &ab_gy);
	callABfilter(&gyros[2], dt, &ab_gz);
}
