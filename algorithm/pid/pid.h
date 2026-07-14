#ifndef PID_H_
#define PID_H_
#include <stdint.h>

struct pid_info {
	float kp;
	float ki;
	float kd;
	float i_limit;
	float out_limit;

	float reference;
	float measure;
	float error;
	float last_error;
	float output;

	// for debug
	float p_out;
	float i_out;
	float d_out;
};

float pid_calculate(struct pid_info *pid, float reference, float measure);

#endif /* PID_H_ */
