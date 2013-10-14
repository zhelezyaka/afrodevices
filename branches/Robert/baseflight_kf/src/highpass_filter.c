/*
 * hp.c
 *
 *  Created on: 20.09.2013
 *      Author: robert
 *
 *
 * recursive single pole high-pass filter
 *
 * reference: The Scientist and Engineer's Guide to Digital Processing
 *
 */

#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "board.h"
#include "mw.h"
#include "highpass_filter.h"

#define fc_cuttoff_acc_hp	2.3f
#define sampling_rate		1000.0f

/* data for highpass filter */
typedef struct highpass
{
	float cutoff;
	float A0, A1, B1;
	float in, out;
} *highpass_t;

struct highpass filter_instance[ACC_LAST];
/*
 * Prepare processing.
 */
bool highpass_setup(float cutoff, float rate, highpass_t highp)
{
	const float pi = M_PI;
	// cutoff frequency should make sense
	if (cutoff > rate / 2.0f || cutoff < 0.1f || cutoff > 10.0f)
	{
		return false;
	}

	highp->B1 = exp((-2.0f * pi * (cutoff / rate)));
	highp->A0 = (1.0f + highp->B1) / 2.0f;
	highp->A1 = (-1.0 * (1.0 + highp->B1)) / 2.0f;
	highp->in = 0.0f;
	highp->out = 0.0f;

	return true;
}

void init_hp_filters()
{
	int n;
	bool ok;

	for (n = 0; n < ACC_LAST; n++)
	{
		highpass_setup(fc_cuttoff_acc_hp, sampling_rate, &filter_instance[n]);
//		ok = highpass_setup(cfg.accz_hp, sampling_rate, &filter_instance[n]);
//		if (!ok)
//		{
//			// fallback to a usefull value
//			highpass_setup(fc_cuttoff_acc_hp, sampling_rate, &filter_instance[n]);
//		}
	}
}

float filter_action(highpass_t instance, float measurement, float dTime)
{
	instance->out = instance->A0 * measurement + instance-> A1 * instance->in + instance->B1 * instance->out;
	instance->in = measurement;
	return instance->out;
}

void hp_filter(int16_t acc[3], float acc_filtered[3])
{
	int axis = 0;

	acc_filtered[axis] = (float)acc[axis] - filter_action(&filter_instance[axis], acc[axis], 0.001f);

	axis++;
	acc_filtered[axis] = (float)acc[axis] - filter_action(&filter_instance[axis], acc[axis], 0.001f);

	axis++;
	acc_filtered[axis] = (float)acc[axis] - filter_action(&filter_instance[axis], acc[axis], 0.001f);
}
