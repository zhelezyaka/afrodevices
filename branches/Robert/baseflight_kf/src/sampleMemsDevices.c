/*
 * sampleMemsDevices.c
 *
 *  Created on: 14.10.2013
 *      Author & Copyright: robert bouwens
 */

#include "board.h"
#include "mw.h"
#include "kalman_simple.h"
#include "highpass_filter.h"

float acc_filtered[3] = {0.0f, 0.0f, 0.0f};
float samples_gyroADC[3] = {0.0f, 0.0f, 0.0f}, samples_accADC[3] = {0.0f, 0.0f, 0.0f};


void sampleMemsDevices()
{
	int i;
	int16_t sample_gyroADC[3] = { 0, 0, 0 };
	int16_t sample_accADC[3] = { 0, 0, 0 };
	bool haveACC = sensors(SENSOR_ACC);

	if (haveACC)
	{
		acc.read(sample_accADC);
		hp_filter(sample_accADC, acc_filtered);
	}
	gyro.read(sample_gyroADC);

	if (!calibratingG)
	{
		// filter is active when not calibrating
		if (haveACC)
		{
			accKalmanFilterStep(sample_accADC);
		}
		gyroKalmanFilterStep(sample_gyroADC);
	}

	// save the individual samples
	for (i = 0; i < 3; i++)
	{
		samples_gyroADC[i] += sample_gyroADC[i];
		samples_accADC[i] += sample_accADC[i];
	}
}
