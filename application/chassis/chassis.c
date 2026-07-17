#include "chassis.h"
#include "motor.h"
#include "sbus.h"

#define VEL_MAX 0.5f	   /* m / s */
#define WHEEL_RADIUS 0.04f /* 40 mm */

#define EPSILON 0.002f

#ifndef PI
#define PI 3.141592653589793f
#endif
#ifndef HALF_PI
#define HALF_PI 1.5707963267948966f
#endif

static enum sbus_sw state = SBUS_SW_UP; /* default */
static float scale = 0;
static void slope_v(float val)
{
	if (val > 0) {
		scale += EPSILON;
	} else if (val < 0) {
		scale -= EPSILON;
	} else {
		scale *= 0.9;
	}

	if (scale > 1.0f) {
		scale = 1.0f;
	} else if (scale < -1.0f) {
		scale = -1.0f;
	}
}

void chassis_task(void)
{
	float w = 0.0f;

	/* check whether safe and update state machine */
	const struct sbus_data *sbus = sbus_get_data();
	if (sbus->safe == SBUS_UNSAFE || sbus->sw2 == SBUS_SW_UP) {
		gm6020_set_pos(0, 0, 0, 0);
		m3508_set_vel(0, 0, 0, 0);
		return;
	} else {
		state = sbus->sw3;
		slope_v(sbus->ls_x);
		w = scale * VEL_MAX / WHEEL_RADIUS;
	}

	/* set command */
	if (state == SBUS_SW_UP) {
		gm6020_set_pos(0, 0, 0, 0);
		m3508_set_vel(-w, -w, w, w);
	} else if (state == SBUS_SW_DOWN) {
		gm6020_set_pos(-HALF_PI, HALF_PI, -HALF_PI, HALF_PI);
		m3508_set_vel(w, -w, -w, w);
	}
}
