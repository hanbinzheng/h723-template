#ifndef PID_H_
#define PID_H_
#include <stdint.h>

struct pid_info {
	float kp;
	float ki;
	float kd;
	float i_limit;
	float out_limit;

	/* output += linear * ref + ff * (ref - last_ref) */
	float linear;
	float ff;

	float ref;
	float last_ref;
	float meas;
	float error;
	float last_error;
	float output;

	// for debug
	float p_out;
	float i_out;
	float d_out;
};

float pid_calculate(struct pid_info *pid, float ref, float meas);

#endif /* PID_H_ */
