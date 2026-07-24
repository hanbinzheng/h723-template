#include "pid.h"
#include <math.h>
#include <stddef.h>

__ALWAYS_INLINE
static float limit_x_min_max(float x, float min, float max)
{
	if (x > max) {
		return max;
	} else if (x < min) {
		return min;
	} else {
		return x;
	}
}

float pid_calculate(struct pid_info *pid, float ref, float meas)
{
	if (pid == NULL) {
		return 0.0f;
	}

	pid->ref = ref;
	pid->meas = meas;
	pid->err = ref - meas;

	pid->p_out = pid->kp * pid->err;
	pid->d_out = pid->kd * (pid->err - pid->last_err);
	pid->ff_out = pid->k_b * ref + pid->k_j * (ref - pid->last_ref);

	pid->i_out += pid->ki * pid->err;
	pid->i_out = limit_x_min_max(pid->i_out, -pid->i_limit, pid->i_limit);

	float raw_out = pid->p_out + pid->i_out + pid->d_out + pid->ff_out;
	pid->output = limit_x_min_max(raw_out, -pid->out_limit, pid->out_limit);

	pid->last_err = pid->err;
	pid->last_ref = ref;

	return pid->output;
}
