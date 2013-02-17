/*
 * rotationMatrix.c
 *
 *  Created on: 07.02.2013
 *      Author: rob
 */

#include "board.h"
#include "integrator.h"

float m[3][3];	// rotation matrix

typedef struct
{
	float velN;
	float velE;
	float velD;
	float posN;
	float posE;
	float alt;
	float agl;				// above ground level from range sensor

} navStruct_t;

navStruct_t navData;

void accIntegratorStep(float accel_ned[3], float dt)
{
	int i;
	float acc;
	float vel[3];

	// from m to cm
	for (i = 0; i < 3; i++)
	{
		acc = accel_ned[i] * 100.0f;
		accel_ned[i] = acc;
	}

	// new velocity
	for (i = 0; i < 3; i++)
	{
		if (fabs(accel_ned[i]) > 1.0)
			vel[i] = navData.velN + accel_ned[i] * dt;
		else
			vel[i] = 0.0;
	}
	// update position
	navData.posN += (navData.velN + vel[0]) * 0.5f * dt;
	navData.posE += (navData.velE + vel[1]) * 0.5f * dt;
	navData.alt -= (navData.velD + vel[2]) * 0.5f * dt;

	navData.agl -= (navData.velD + vel[2]) * 0.5f * dt;

	// update velocity
	navData.velN = vel[0];
	navData.velE = vel[1];
	navData.velD = vel[2];
}

void getPosition(int *x, int *y, int *z)
{
	*x = (int) navData.posN;
	*y = (int) navData.posE;
	*z = (int) navData.alt;
}

float getZVelocity()
{
	return navData.velD;
}
