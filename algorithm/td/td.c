#include "td.h"
#include "arm_math.h"

#define TD_PI 3.14159265358979323846f
#define TD_TWO_PI 6.28318530717958647692f

__ALWAYS_INLINE
static float signf(float x)
{
	return (float)(x > 0.0f) - (float)(x < 0.0f);
}

__ALWAYS_INLINE
float wrap_in_pi(float x)
{
	if (x > TD_PI) {
		return x - TD_TWO_PI;
	} else if (x < -TD_PI) {
		return x + TD_TWO_PI;
	}

	return x;
}

/**
 * @brief Han's non-linear discrete optimal control synthesizer (fast han)
 *
 * @param x_err position error (x_est - x_meas)
 * @param v_est current velocity estimation
 * @param r acceleration gain
 * @param h filtering factor
 *
 * @return Optimal synthesis control signal (acceleration)
 */
__ALWAYS_INLINE
static float fhan(float x_err, float v_est, float r, float h)
{
	float d = r * h * h;

	float a0 = h * v_est;
	float y = x_err + a0;

	/* a1 = sqrt(d * (d + 8 * |y|)) */
	float a1;
	float a1_sq = d * (d + 8.0f * fabsf(y));
	arm_sqrt_f32(a1_sq, &a1);

	float a2 = a0 + signf(y) * (a1 - d) * 0.5f;
	float sy = (signf(y + d) - signf(y - d)) * 0.5f;
	float a = (a0 + y - a2) * sy + a2;
	float sa = (signf(a + d) - signf(a - d)) * 0.5f;

	return -r * (a / d - signf(a)) * sa - r * signf(a);
}

float td_update(struct td_estimator *td, float x_meas)
{
	if (td == NULL) {
		return NAN;
	}

	if (td->initialized == false) {
		td->x_est = x_meas;
		td->initialized = true;
		return td->v_est;
	}

	float x_err = td->x_est - x_meas;
	x_err = wrap_in_pi(x_err);

	/* synthesize non-linear acceleration */
	float a = fhan(x_err, td->v_est, td->r, td->h);

	/* state update */
	td->x_est += td->dt * td->v_est;
	td->x_est = wrap_in_pi(td->x_est);
	td->v_est += td->dt * a;

	return td->v_est;
}
