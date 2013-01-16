/*
 * AlphaBetaFilter.h
 *
 *  Created on: 12.01.2013
 *      Author: rob
 */

#ifndef ALPHABETAFILTER_H_
#define ALPHABETAFILTER_H_

void initAlphaBetafilter();
void accelABfilterStep(int16_t acc[3], float dt);
void gyroABfilterStep(int16_t gyros[3], float dt);


#endif /* ALPHABETAFILTER_H_ */
