/*
 * kalman.h
 *
 *  Created on: 25.11.2012
 *      Author: rob
 */

#ifndef KALMAN_H_
#define KALMAN_H_

void accKalmanFilterStep(int16_t acc[3]);
void gyroKalmanFilterStep(int16_t gyros[3]);



#endif /* KALMAN_H_ */
