/*
 * filter.h
 *
 *  Created on: 25.11.2012
 *      Author: rob
 */

#ifndef FILTER_H_
#define FILTER_H_

#define NUMBER_OF_FIRST_ORDER_FILTERS 3

#define ACCEL_X_LOWPASS 0
#define ACCEL_Y_LOWPASS 1
#define ACCEL_Z_LOWPASS 2

///////////////////////////////////////////////////////////////////////////////

typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

///////////////////////////////////////////////////////////////////////////////

void initFirstOrderFilter();

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters);

///////////////////////////////////////////////////////////////////////////////

void accelFilterStep(int16_t acc[3]);

#endif /* FILTER_H_ */
