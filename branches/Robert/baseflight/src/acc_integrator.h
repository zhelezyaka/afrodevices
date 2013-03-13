/*
 * acc_integrator.h
 *
 *  Created on: Mar 8, 2013
 *      Author: robert
 */

#ifndef ACC_INTEGRATOR_H_
#define ACC_INTEGRATOR_H_

KalmanFilter alloc_filter_velocity3d(float P, float Q, float R);
void set_seconds_per_timestep_3d(KalmanFilter *f, float seconds_per_timestep);
void init_integrator();
float acc_integrator_step(float acc, float dt);
#endif /* ACC_INTEGRATOR_H_ */
