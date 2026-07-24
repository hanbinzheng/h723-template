#include "chassis.h"
#include "motor.h"
#include "sbus.h"

#define VEL_MAX 0.5f	   /* m / s */
#define WHEEL_RADIUS 0.04f /* 40 mm */

#define EPSILON 0.002f

#define GM6020_1_POS (5546 - 3331) * 0.0007669903939428206f
#define GM6020_2_POS (1555 - 3517) * 0.0007669903939428206f
#define GM6020_3_POS (711 - 6978 + 8192) * 0.0007669903939428206f
#define GM6020_4_POS (725 - 2780) * 0.0007669903939428206f

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
		gm6020_set_pos(GM6020_4_POS, GM6020_3_POS, GM6020_2_POS, GM6020_1_POS);
		m3508_set_vel(w, -w, -w, w);
	}
}
