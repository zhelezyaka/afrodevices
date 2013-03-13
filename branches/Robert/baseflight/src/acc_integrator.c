/*
 * acc_integrator.c
 *
 *  Created on: Mar 8, 2013
 *      Author: robert
 */

#include "stdio.h"
#include "matrix.h"
#include "kalman.h"
#include "acc_integrator.h"

//#define Q 0.2			// process noise covariance
//#define	R 10.0			// measurement noise covariance
//#define P 0.22			// estimation error covariance

static KalmanFilter accFilter;

KalmanFilter alloc_filter_velocity3d(float P, float Q, float R)
{
	/* The state model has three dimensions:
	 x, x', x''
	 Each time step we can only observe position, not velocity and acceleration,
	 so the observation vector has only one dimension.
	 */
	KalmanFilter f = alloc_filter(3, 1);

	set_identity_matrix(&f.state_transition);
	set_seconds_per_timestep_3d(&f, 0.004);

	/* We observe (x) in each time step */
	{
		const float values[] = { 1.0, 0.0, 0.0 };
		set_matrix(&f.observation_model, sizeof(values) / sizeof(values[0]), values);
	}

	/* Noise in the world. */
	set_identity_matrix_value(&f.process_noise_covariance, Q);

	/* Noise in our observation */
	set_identity_matrix_value(&f.observation_noise_covariance, R);

	/* The start position is totally unknown, so give a high variance */
	scale_matrix(&f.state_estimate, 0.0);
	set_identity_matrix_value(&f.estimate_covariance, P);

	return f;
}

void set_seconds_per_timestep_3d(KalmanFilter *f, float seconds_per_timestep)
{
	/* unit_scaler accounts for the relation between position and
	 velocity units */
	float t2 = seconds_per_timestep * seconds_per_timestep;
	f->state_transition.data[0][1] = seconds_per_timestep;
	f->state_transition.data[0][2] = t2;
	f->state_transition.data[1][2] = seconds_per_timestep;
}

void init_integrator()
{
//	int i;
//	const float p = 0.002;
//	const float q = 0.2;
//	const float r = 0.22;

	const float p = 0.22;
	const float q = 0.2;
	const float r = 25.0;

	accFilter = alloc_filter_velocity3d(p, q, r);
	/* Test with time steps of the position gradually increasing */
//	for (i = 0; i < 10; ++i)
//	{
//		const float values[] =
//			{ i };
//		set_matrix(&f.observation, sizeof(values) / sizeof(values[0]), values);
//		update(&f);
//		/* Our prediction should be close to (10, 1) */
//		printf("estimated value: %f -\t", f.state_estimate.data[0][0]);
//		printf("estimated velocity: %f - \t", f.state_estimate.data[1][0]);
//		printf("estimated position: %f\n", f.state_estimate.data[2][0]);
//	}
//	free_filter(&accFilter);
}

float acc_integrator_step(float acc, float dt)
{
		const float values[] = { acc * 100.0 };
		set_matrix(&accFilter.observation, sizeof(values) / sizeof(values[0]), values);
		set_seconds_per_timestep_3d(&accFilter, dt);
		update(&accFilter);
		// the position will be returned
		return accFilter.state_estimate.data[2][0];
}
