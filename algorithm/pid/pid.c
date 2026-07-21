#include "pid.h"
#include <math.h>
#include <stddef.h>

#define SIGN(x) ((x > 0.0f) ? 1.0f : -1.0f)

__ALWAYS_INLINE
static float limit(float x, float min, float max)
{
	if (x > max) {
		return max;
	} else if (x < min) {
		return min;
	} else {
		return x;
	}
}

__ITCM_FUNC
float pid_calculate(struct pid_info *pid, float ref, float meas)
{
	if (pid == NULL) {
		return 0.0f;
	}

	pid->ref = ref;
	pid->meas = meas;
	pid->error = ref - meas;

	pid->p_out = pid->kp * pid->error;

	pid->i_out += pid->ki * pid->error;
	pid->i_out = limit(pid->i_out, -pid->i_limit, pid->i_limit);

	pid->d_out = pid->kd * (pid->error - pid->last_error);

	pid->output = pid->p_out + pid->i_out + pid->d_out;
	pid->output += SIGN(ref) * pid->ff * (ref - pid->last_ref) + pid->linear * ref;

	pid->output = limit(pid->output, -pid->out_limit, pid->out_limit);

	pid->last_error = pid->error;
	pid->last_ref = ref;

	return pid->output;
}
