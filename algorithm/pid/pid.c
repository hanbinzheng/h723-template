#include "pid.h"
#include <stddef.h>

static inline float limit(float x, float min, float max)
{
	if (x > max) {
		return max;
	} else if (x < min) {
		return min;
	} else {
		return x;
	}
}

float pid_calculate(struct pid_info *pid, float reference, float measure)
{
	if (pid == NULL) {
		return 0.0f;
	}

	pid->reference = reference;
	pid->measure = measure;
	pid->error = reference - measure;

	pid->p_out = pid->kp * pid->error;

	pid->i_out += pid->ki * pid->error;
	pid->i_out = limit(pid->i_out, -pid->i_limit, pid->i_limit);

	pid->d_out = pid->kd * (pid->error - pid->last_error);

	pid->output = pid->p_out + pid->i_out + pid->d_out;
	pid->output = limit(pid->output, -pid->out_limit, pid->out_limit);

	pid->last_error = pid->error;

	return pid->output;
}
