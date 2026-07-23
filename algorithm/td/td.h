#ifndef TD_H_
#define TD_H_

#include <stdbool.h>

struct td_estimator {
	float x_est; /* position */
	float v_est; /* velocity */
	float r;     /* velocity factor (max acceleration) */
	float h;     /* filter facto / estimated step length, 2 ~ 5 * dt */
	float dt;    /* in second, sampling period */

	bool initialized;
};

/**
 * @brief Execute one-step TD update with angle unwrapping
 *
 * Thif function first calculate position difference
 * x_err = x_est - x_meas, and estimated the acceleration based on x_err
 * a = func(x_err, v_est, r, h), finally it updates the estimated position and velocity
 * x_est <- x_est + v_est * dt
 * v_est <- v_est + a * dt
 *
 * @param td pointer to tracking differentiator context
 * @param x_meas measured position, bounded in [-PI, PI]
 *
 * @return Estimated smooth velocity
 */
float td_update(struct td_estimator *td, float x_meas);

#endif /* TD_H_ */
