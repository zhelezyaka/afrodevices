/*
 * baroFilter.h
 *
 *  Created on: 21.07.2013
 *      Author: rob
 */

#ifndef BAROFILTER_H_
#define BAROFILTER_H_

void initBarofilter();
void baroKalmanfilterStep(int32_t *baro);
float getClimbRate();

#endif /* BAROFILTER_H_ */
