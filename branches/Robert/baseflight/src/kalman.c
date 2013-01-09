/*
 * kalman.c
 *
 *  Created on: 25.11.2012
 *      Author: robert
 */

#include "stdint.h"
#include "kalman.h"

/*
http://hicode.wordpress.com/2011/10/21/1-d-kalman-filter-for-smoothing-gps-accelerometer-signals/
            //http://en.wikipedia.org/wiki/Kalman_filter
            //run kalman filtering
            //x_k = Ax_{k-1} + Bu_k + w_k
            //z_k = Hx_k+v_k
            //time update
            //x_k = Ax_{k-1} + Uu_k
            //P_k = AP_{k-1}A^T + Q
            //measurement update
            //K_k = P_k H^T(HP_kH^T + R)^T
            //x_k = x_k + K_k(z_k - Hx_k)
            //P_k = (I - K_kH)P_k

			a = 1.0;
			h = 1.0;
			p = 0.1;
			q = 0.001;
			r = 10;
 */

typedef struct {
	float q;	// process noise covariance
	float r;	// measurement noise covariance
	float x;	// value
	float p;	// estimation error covariance
} kalmanState_t;

// accelerometer
kalmanState_t kax;
kalmanState_t kay;
kalmanState_t kaz;

// gyro
kalmanState_t kgx;
kalmanState_t kgy;
kalmanState_t kgz;

// barometer
kalmanState_t kbaro;


static void initKalmanState(kalmanState_t* state, float q, float r, float p, float intial_value) {
	state->q = q;
	state->r = r;
	state->p = p;
	state->x = intial_value;
}

static void kalmanUpdate(kalmanState_t* state, int16_t *pvalue) {
/*
                // the measured value
                // is z

                // time update - prediction
                x = A * x;
                P = A * P * A + Q;

                // measurement update - correction
                K = P * H / (H * P * H + R);
                x = x + K * (z - H * x);
                P = (1 - K * H) * P;

                // the estimated value
                // is x

                // A and H are set to 1.0 and therefore are 'optimized'
 */
	float k;	// kalman gain
	float measurement = *pvalue;
	state->p = state->p + state->q;

	//measurement update
	k = state->p / (state->p + state->r);
	state->x = state->x + k * (measurement - state->x);
	state->p = (1 - k) * state->p;

	measurement = state->x;
	*pvalue = (int16_t) measurement;
}

static void initKalmanGyro()
{
	// real bad on my small jakub frame
//#define Q 0.0625 // process noise covariance
//#define	R 4.0	// measurement noise covariance
//#define P 0.47	// estimation error covariance

	// small jakub frame
//#define Q 1.0 	// process noise covariance
//#define	R 0.01	// measurement noise covariance
//#define P 0.22	// estimation error covariance

#define Q 3.0 	// process noise covariance
#define	R 0.125	// measurement noise covariance
#define P 0.42	// estimation error covariance	<-- rise to 0.6 is to twitchy - or lower to 0.22 for much more fun

	initKalmanState(&kgx, Q, R, P, 0);
	initKalmanState(&kgy, Q, R, P, 0);
	initKalmanState(&kgz, Q, R, P, 0);

#undef Q
#undef R
#undef P
}


static void initKalmanAccel() {
	// small jakub frame
#define Q 0.0625	// process noise covariance
#define	R 1.0		// measurement noise covariance
#define P 0.22		// estimation error covariance

//#define Q 0.0625 // process noise covariance
//#define	R 4.0	// measurement noise covariance
//#define P 0.47	// estimation error covariance

	initKalmanState(&kax, Q, R, P, 0);
	initKalmanState(&kay, Q, R, P, 0);
	initKalmanState(&kaz, Q, R, P, 0);

#undef Q
#undef R
#undef P
}

static void initKalmanBaro() {
//#define Q 0.01	// process noise covariance
//#define	R 2.0	// measurement noise covariance
//#define P 0.2	// estimation error covariance

#define Q 0.0625 // process noise covariance
#define	R 4.0	// measurement noise covariance
#define P 0.47	// estimation error covariance

	initKalmanState(&kbaro, Q, R, P, 0);
	initKalmanState(&kbaro, Q, R, P, 0);
	initKalmanState(&kbaro, Q, R, P, 0);

#undef Q
#undef R
#undef P
}

void accelKalmanfilterStep(int16_t acc[3]) {
	kalmanUpdate(&kax, &acc[0]);
	kalmanUpdate(&kay, &acc[1]);
	kalmanUpdate(&kaz, &acc[2]);
}

void gyroKalmanfilterStep(int16_t gyros[3]) {
	kalmanUpdate(&kgx, &gyros[0]);
	kalmanUpdate(&kgy, &gyros[1]);
	kalmanUpdate(&kgz, &gyros[2]);
}

void baroKalmanfilterStep(int32_t *baro) {
	float k;	// kalman gain
	float measurement = *baro;
	kbaro.p = kbaro.p + kbaro.q;

	//measurement update
	k = kbaro.p / (kbaro.p + kbaro.r);
	kbaro.x = kbaro.x + k * (measurement - kbaro.x);
	kbaro.p = (1 - k) * kbaro.p;

	*baro = (int32_t)kbaro.x;
}

void initKalmanfilters() {
	initKalmanAccel();
	initKalmanGyro();
	initKalmanBaro();
}
