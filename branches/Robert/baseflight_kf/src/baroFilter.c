/*
 * baroFilter.c
 *
 *  Created on: 21.07.2013
 *      Author: rob
 */

#include "stdint.h"
#include "stdbool.h"
#include "barofilter.h"

#include "kalman.h"

KalmanFilter baroFilter;

/* Test the example of a train moving along a 1-d track */
void initBarofilter()
{
	baroFilter = alloc_filter(2, 1);

	/* The baro state is a 2d vector containing position and climbrate.
	 climbrate is measured in position units per timestep units.
	 In our case the time between gathering the baro samples */

	const float values[] = { 1.0f, 1.0f, 0.0f, 1.0f};

	set_matrix(&baroFilter.state_transition, sizeof(values) / sizeof(values[0]), values);

	/* We only observe position */
	const float pos[] = { 1.0f, 0.0f};
	set_matrix(&baroFilter.observation_model, sizeof(pos) / sizeof(pos[0]), pos);

	/* The covariance matrices are blind guesses */
	set_identity_matrix(&baroFilter.process_noise_covariance);
	set_identity_matrix(&baroFilter.observation_noise_covariance);

	/* Our knowledge of the start position is incorrect and unconfident */
	const float deviation[] = { 10000.0f };
	set_matrix(&baroFilter.state_estimate, sizeof(deviation) / sizeof(deviation[0]), deviation);

	const float scale = 1000.0f * 1000.0f;
	set_identity_matrix(&baroFilter.estimate_covariance);
	scale_matrix(&baroFilter.estimate_covariance, scale);

	/* the prediction would be: */
	// printf("estimated position: %f\n", f.state_estimate.data[0][0]);
	// printf("estimated velocity: %f\n", f.state_estimate.data[1][0]);
}

void baroKalmanfilterStep(int32_t *baro)
{
	const float values[] = {*baro};

	set_matrix(&baroFilter.observation, sizeof(values) / sizeof(values[0]), values);
	update(&baroFilter);
	// the position will be returned
	// correct possible rounding issue
	*baro = (int32_t) (baroFilter.state_estimate.data[0][0] + 0.5f);
}

float getClimbRate()
{
	return baroFilter.state_estimate.data[1][0];
}

