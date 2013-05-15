/****************************************************************************
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file position_estimator_inav_params.c
 * 
 * Parameters for position_estimator_inav
 */

#include "position_estimator_inav_params.h"

/**
Here is my first test results for altitude estimation on PX4FMU. Calibration procedure is not implemented yet and I have selected acceleration transform parameters by hands. It the test board was moved vertically not preserving orientation carefully.
Kalman gain used:

K = [ 0.013364   0        ]
    | 0.005619   0.000074 |
    [ 0          0.9629   ]
*/
/* Kalman Filter covariances */
/* gps measurement noise standard deviation */

int INAV_USE_GPS = 0;

float INAV_K_ALT_00 = 0.013364f;
float INAV_K_ALT_01 = 0.0f;
float INAV_K_ALT_10 = 0.005619f;
float INAV_K_ALT_11 = 0.000074f;
float INAV_K_ALT_20 = 0.0f;
float INAV_K_ALT_21 = 0.9629f;

int INAV_ACC_OFFS_0 = 0;
int INAV_ACC_OFFS_1 = 0;
int INAV_ACC_OFFS_2 = 0;

float INAV_ACC_T_00 = 0.0021f;
float INAV_ACC_T_01 = 0.0f;
float INAV_ACC_T_02 = 0.0f;
float INAV_ACC_T_10 = 0.0f;
float INAV_ACC_T_11 = 0.0021f;
float INAV_ACC_T_12 = 0.0f;
float INAV_ACC_T_20 = 0.0f;
float INAV_ACC_T_21 = 0.0f;
float INAV_ACC_T_22 = 0.0021f;

int parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->k_alt_00 = INAV_K_ALT_00;
	h->k_alt_01 = INAV_K_ALT_01;
	h->k_alt_10 = INAV_K_ALT_10;
	h->k_alt_11 = INAV_K_ALT_11;
	h->k_alt_20 = INAV_K_ALT_20;
	h->k_alt_21 = INAV_K_ALT_21;

	h->acc_offs_0 = INAV_ACC_OFFS_0;
	h->acc_offs_1 = INAV_ACC_OFFS_1;
	h->acc_offs_2 = INAV_ACC_OFFS_2;

	h->acc_t_00 = INAV_ACC_T_00;
	h->acc_t_01 = INAV_ACC_T_01;
	h->acc_t_02 = INAV_ACC_T_02;
	h->acc_t_10 = INAV_ACC_T_10;
	h->acc_t_11 = INAV_ACC_T_11;
	h->acc_t_12 = INAV_ACC_T_12;
	h->acc_t_20 = INAV_ACC_T_20;
	h->acc_t_21 = INAV_ACC_T_21;
	h->acc_t_22 = INAV_ACC_T_22;
	return 1;
}

int parameters_update(struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p)
{
	h->k_alt_00 = p->k[0][0];
	h->k_alt_01 = (p->k[0][1]);
	h->k_alt_10 = (p->k[1][0]);
	h->k_alt_11 = (p->k[1][1]);
	h->k_alt_20 = (p->k[2][0]);
	h->k_alt_21 = (p->k[2][1]);

	h->acc_offs_0 = (p->acc_offs[0]);
	h->acc_offs_1 = (p->acc_offs[1]);
	h->acc_offs_2 = (p->acc_offs[2]);

	h->acc_t_00 = (p->acc_T[0][0]);
	h->acc_t_01 = (p->acc_T[0][1]);
	h->acc_t_02 = (p->acc_T[0][2]);
	h->acc_t_10 = (p->acc_T[1][0]);
	h->acc_t_11 = (p->acc_T[1][1]);
	h->acc_t_12 = (p->acc_T[1][2]);
	h->acc_t_20 = (p->acc_T[2][0]);
	h->acc_t_21 = (p->acc_T[2][1]);
	h->acc_t_22 = (p->acc_T[2][2]);
	return 1;
}
