/*
 * rotationMatrix.h
 *
 *  Created on: 07.02.2013
 *      Author: rob
 */

#ifndef ROTATIONMATRIX_H_
#define ROTATIONMATRIX_H_

void accIntegratorStep(float accel_ned[3], float dt);
void getPosition(int *x, int *y, int *z);
float getZPosition();
float getZVelocity();
void resetIntegrator();
void resetAltitude(int altHold);
float getNedZ();


#endif /* ROTATIONMATRIX_H_ */
