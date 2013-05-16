/*
 * barofilter.c
 *
 *  Created on: 27.01.2013
 *      Author: rob
 */

#include <stdint.h>
#include "board.h"
#include "kalman.h"

#define	F_CUT_BARO     2.0f
float fc;

KalmanFilter baroFilter;


void init_kalman_2d(float measurement)
{
#define Q 1.2			// process noise covariance
#define	R 40.0			// measurement noise covariance
#define P 5.0			// estimation error covariance

	baroFilter = alloc_filter(2, 1);

	/* The train state is a 2d vector containing position and velocity.
	 Velocity is measured in position units per timestep units. */
	{
		const float values[] =
			{
				1.0, 1.0,
				0.0, 1.0
			};
		set_matrix(&baroFilter.state_transition, sizeof(values) / sizeof(values[0]), values);
	}

	/* We only observe position */
	{
		const float values[] =
			{
				1.0, 0.0
			};
		set_matrix(&baroFilter.observation_model, sizeof(values) / sizeof(values[0]), values);
	}

	/* The covariance matrices are blind guesses */
	set_identity_matrix(&baroFilter.process_noise_covariance);
	scale_matrix(&baroFilter.process_noise_covariance, Q);

	set_identity_matrix(&baroFilter.observation_noise_covariance);
	scale_matrix(&baroFilter.observation_noise_covariance, R);

	/* Our knowledge of the start position is incorrect and unconfident */
	{
		float values[] = { measurement };
		set_matrix(&baroFilter.state_estimate, sizeof(values) / sizeof(values[0]), values);
	}
	set_identity_matrix(&baroFilter.estimate_covariance);
	scale_matrix(&baroFilter.estimate_covariance, P);
}

void baroKalmanfilterStep(int32_t *baro)
{
	static int _init = 0;
	static float dTerm = 0;
	static uint32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dt = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;

	float m = *baro;
	m = dTerm + (dt / (fc + dt)) * (m - dTerm);
	dTerm = m;

	if (!_init)
	{
		_init = 1;
		init_kalman_2d(m);
		fc = 0.5f / (M_PI * F_CUT_BARO);
	}
	else
	{
		float values[] = { m };
		set_matrix(&baroFilter.observation, sizeof(values) / sizeof(values[0]), values);
		update(&baroFilter);
		m = baroFilter.state_estimate.data[0][0];
		*baro = (int32_t) m;
		/*
		 Output
		 */
//		if (liftoff == 0)
//		{
//			if (est[1] < -5.0)
//			{
//				liftoff = 1;
//				//uartPrint("Liftoff detected\r\n");
//			}
//		}
//		else
//		{
//			if (est[1] > 0)
//			{
//				//uartPrint("Apogee detected\r\n");
//			}
//		}
	}
}
