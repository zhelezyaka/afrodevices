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
 * @file position_estimator_inav_params.h
 * 
 * Parameters for Position Estimator
 */
#include "stdint.h"

struct position_estimator_inav_params
{
	int		 use_gps;
	float	k[3][2];
	int16_t	acc_offs[3];
	float	acc_T[3][3];
};

struct position_estimator_inav_param_handles
{
	int use_gps;

	float k_alt_00;
	float k_alt_01;
	float k_alt_10;
	float k_alt_11;
	float k_alt_20;
	float k_alt_21;

	int acc_offs_0;
	int acc_offs_1;
	int acc_offs_2;

	float acc_t_00;
	float acc_t_01;
	float acc_t_02;
	float acc_t_10;
	float acc_t_11;
	float acc_t_12;
	float acc_t_20;
	float acc_t_21;
	float acc_t_22;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct position_estimator_inav_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p);
