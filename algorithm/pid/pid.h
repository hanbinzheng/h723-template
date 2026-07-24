#ifndef PID_H_
#define PID_H_
#include <stdint.h>

struct pid_info {
	float kp;
	float ki;
	float kd;
	float i_limit;
	float out_limit;

	/* ff_out = k_b * ref + k_j * (ref - last_ref) */
	float k_b; /* B * w */
	float k_j; /* J * d w / dt */

	float ref;
	float last_ref;
	float meas;
	float err;
	float last_err;
	float output;

	// for debug
	float p_out;
	float i_out;
	float d_out;
	float ff_out;
};

float pid_calculate(struct pid_info *pid, float ref, float meas);

#endif /* PID_H_ */
