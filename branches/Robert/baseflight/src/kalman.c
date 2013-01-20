/*
 * kalman.c
 *
 *  Created on: 25.11.2012
 *      Author: robert
 *
 *      acts more as a filter front end for testing :)
 */

#include <stdint.h>
#include "board.h"
#include "kalman.h"
#include "kalman1D.h"
#include "AlphaBetaFilter.h"

extern uint16_t cycleTime;

// select filter type
#define k1d
//#undef k1d
//#define AlphaBeta
//#undef alphaBeta


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

// barometer
kalmanState_t kbaro;

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

#ifdef k1d
	// accelerometer
	kalman1D_t kax;
	kalman1D_t kay;
	kalman1D_t kaz;

	// gyro
	kalman1D_t kgx;
	kalman1D_t kgy;
	kalman1D_t kgz;

#else
	// accelerometer
	kalmanState_t kax;
	kalmanState_t kay;
	kalmanState_t kaz;

	// gyro
	kalmanState_t kgx;
	kalmanState_t kgy;
	kalmanState_t kgz;

#endif


static void initSimpleKalmanState(kalmanState_t* state, float q, float r, float p, float intial_value) {
	state->q = q;
	state->r = r;
	state->p = p;
	state->x = intial_value;
}

static void kalmanSimpleUpdate(kalmanState_t* state, int16_t *pvalue) {
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

	// working the larger jakub frame
#define Q 4.0 	// process noise covariance
#define	R 0.625	// measurement noise covariance
#define P 0.42	// estimation error covariance	<-- rise to 0.6 is to twitchy - or lower to 0.22 for much more fun

#ifdef k1d
	initKalman1D(&kgx, Q, R, P, 0);
	initKalman1D(&kgy, Q, R, P, 0);
	initKalman1D(&kgz, Q, R, P, 0);
#elif defined(AlphaBeta)
	initAlphaBetafilter();
#else
	initSimpleKalmanState(&kgx, Q, R, P, 0);
	initSimpleKalmanState(&kgy, Q, R, P, 0);
	initSimpleKalmanState(&kgz, Q, R, P, 0);
#endif

#undef Q
#undef R
#undef P
}


static void initKalmanAccel() {
	// small jakub frame
#define Q 0.0625		// process noise covariance
#define	R 1.0		// measurement noise covariance
#define P 0.22		// estimation error covariance

//#define Q 0.0625		// process noise covariance
//#define	R 4.0			// measurement noise covariance
//#define P 0.47			// estimation error covariance

//#define Q 0.625			// process noise covariance
//#define	R 4.0			// measurement noise covariance
//#define P 0.47			// estimation error covariance

#ifdef k1d
	initKalman1D(&kax, Q, R, P, 0);
	initKalman1D(&kay, Q, R, P, 0);
	initKalman1D(&kaz, Q, R, P, 0);
#elif defined(AlphaBeta)
	initAlphaBetafilter();
#else
	initSimpleKalmanState(&kax, Q, R, P, 0);
	initSimpleKalmanState(&kay, Q, R, P, 0);
	initSimpleKalmanState(&kaz, Q, R, P, 0);
#endif

#undef Q
#undef R
#undef P
}

static void initKalmanBaro() {
//#define Q 0.01	// process noise covariance
//#define	R 2.0	// measurement noise covariance
//#define P 0.2		// estimation error covariance

//#define Q 0.0625	// process noise covariance
//#define	R 4.0	// measurement noise covariance
//#define P 0.67	// estimation error covariance

#define Q 4.0 		// process noise covariance
#define	R 0.625		// measurement noise covariance
#define P 0.42		// estimation error covariance

	// up and downs in meters ... :(
//#define Q 1e-5 	// process noise covariance
//#define	R 1e-1	// measurement noise covariance
//#define P 0.42	// estimation error covariance

	initSimpleKalmanState(&kbaro, Q, R, P, 0);

#undef Q
#undef R
#undef P
}

void accelKalmanfilterStep(int16_t acc[3]) {
#ifdef k1d
	static int32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dT = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;
	kalman1DUpdate(&kax, &acc[0], dT);
	kalman1DUpdate(&kay, &acc[1], dT);
	kalman1DUpdate(&kaz, &acc[2], dT);
#elif defined(AlphaBeta)
	static uint32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dT = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;
	accelABfilterStep(acc, dT);
#else
	kalmanSimpleUpdate(&kax, &acc[0]);
	kalmanSimpleUpdate(&kay, &acc[1]);
	kalmanSimpleUpdate(&kaz, &acc[2]);
#endif


}

void gyroKalmanfilterStep(int16_t gyros[3]) {
#ifdef k1d
	static uint32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dT = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;
	kalman1DUpdate(&kgx, &gyros[0], dT);
	kalman1DUpdate(&kgy, &gyros[1], dT);
	kalman1DUpdate(&kgz, &gyros[2], dT);
#elif defined(AlphaBeta)
	static uint32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dT = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;
	 gyroABfilterStep(gyros, dT);
#else
	kalmanSimpleUpdate(&kgx, &gyros[0]);
	kalmanSimpleUpdate(&kgy, &gyros[1]);
	kalmanSimpleUpdate(&kgz, &gyros[2]);
#endif
}

void initKalmanfilters() {
	initKalmanAccel();
	initKalmanGyro();
	initKalmanBaro();
}
