/*
 * highpass_filter.h
 *
 *  Created on: 21.09.2013
 *      Author: rob
 */

#ifndef HIGHPASS_FILTER_H_
#define HIGHPASS_FILTER_H_

// Type of accelerometer used/detected
typedef enum hp_filter {
    ACC_X = 0,
    ACC_Y,
    ACC_Z,
    ACC_LAST
} hp_filter_t;

void init_hp_filters();

void init_hp_filters();
void hp_filter(int16_t acc[3], float acc_filtered[3]);

#endif /* HIGHPASS_FILTER_H_ */
