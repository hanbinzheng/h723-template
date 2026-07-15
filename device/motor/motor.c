#include "motor.h"
#include "bsp_fdcan.h"
#include "pid.h"
#include "sbus.h"
#include "vofa.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define DJI_MOTOR_POS_RANGE 8192
#define DJI_MOTOR_POS_HALF_RANGE 4096

#define DJI_RPM_TO_RADS(value) ((float)(value) * 0.10471975511965977f)	 /* 2 * PI / 60 */
#define DJI_POS_TO_RADS(value) ((float)(value) * 0.0007669903939428206f) /* 2 * PI / RANGE */

/* GM6020 */
/* current: -16384 ~ 16384 <-> -3 ~ 3 */
#define DJI_GM6020_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 5461.333333f)) /* 16384 / 3 */
#define DJI_GM6020_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 0.0001831054687f) /* 3 / 16384 */
/* voltage: -24 ~ 24 <-> -25000 ~ 25000 */
#define DJI_GM6020_VOLTAGE_FLOAT_TO_INT(value) ((int16_t)((value) * 1041.66667f)) /* 25000 / 24 */

/* M3508 */
#define DJI_M3508_REDUCTION_RATE 19.203208556149733f /* 3591 / 187 */
/* current: -20 ~ 20 <-> 16384 ~ 16384 */
#define DJI_M3508_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 0.001220703125f) /* 20 / 16384 */
#define DJI_M3508_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 819.2f))	 /* 16384 / 20 */

struct dji_motor_inst {
	enum dji_motor_type type;

	int16_t raw_pos; /* 0 ~ 8191 */
	int16_t raw_vel;
	int16_t raw_eff;

	int16_t offset; /* 0 ~ 8191 */
	float pos;	/* -PI ~ PI */
	float vel;
	float eff;

	struct pid_info pid_v2e; /* velocity to effort (current or voltage) */
	struct pid_info pid_p2v; /* position to veolcity */

	struct can_rx_inst *can_rx_inst;
	uint16_t cmd; /* final comand */
};

/* static function */
static void motor_callback(struct can_rx_inst *can_inst, uint8_t *buff);
static void set_single_vel(struct dji_motor_inst *motor, float ref);
static void set_single_pos(struct dji_motor_inst *motor, float ref);

__ALWAYS_INLINE
static void raw_pos_into_pos(struct dji_motor_inst *motor)
{
	int16_t tmp = motor->raw_pos - motor->offset;

	/* scale the value into -4096 ~ 4096 */
	if (tmp < -DJI_MOTOR_POS_HALF_RANGE) {
		tmp += DJI_MOTOR_POS_RANGE;
	} else if (tmp > DJI_MOTOR_POS_HALF_RANGE) {
		tmp -= DJI_MOTOR_POS_RANGE;
	}

	motor->pos = DJI_POS_TO_RADS(tmp);
}

/* static variables */
static struct can_tx_inst *can_1 = NULL;
static struct can_tx_inst *can_2 = NULL;

struct dji_motor_inst gm6020_1 = {
    .type = DJI_GM6020,
    .offset = 3475,
    .pid_v2e =
	{.kp = 0.4f, .ki = 0.014f, .kd = 0.0f, .i_limit = 0.5f, .out_limit = 20.0f, .linear = 0.5f},
    .pid_p2v = {.kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 20.0f},
};

struct dji_motor_inst gm6020_2 = {
    .type = DJI_GM6020,
    .offset = 3420,
    .pid_v2e =
	{.kp = 0.6f, .ki = 0.1f, .kd = 0.0f, .i_limit = 1.2f, .out_limit = 20.0f, .linear = 0.4f},
    .pid_p2v = {.kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 20.0f},
};

struct dji_motor_inst gm6020_3 = {
    .type = DJI_GM6020,
    .offset = 6985,
    .pid_v2e =
	{.kp = 1.1f, .ki = 0.1f, .kd = 0.0f, .i_limit = 2.0f, .out_limit = 20.0f, .linear = 0.4f},
    .pid_p2v = {.kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 20.0f},
};

struct dji_motor_inst gm6020_4 = {
    .type = DJI_GM6020,
    .offset = 5975,
    .pid_v2e =
	{.kp = 1.0f, .ki = 0.1f, .kd = 0.0f, .i_limit = 1.2f, .out_limit = 20.0f, .linear = 0.4f},
    .pid_p2v = {.kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 20.0f},
};

struct dji_motor_inst m3508_5 = {
    .type = DJI_M3508,
    .pid_v2e = {.kp = 0.05f, .ki = 0.0003f, .kd = 0.0f, .i_limit = 1.0f, .out_limit = 20.0f},
};

struct dji_motor_inst m3508_6 = {
    .type = DJI_M3508,
    .pid_v2e = {.kp = 0.035f, .ki = 0.0002f, .kd = 0.0f, .i_limit = 1.0f, .out_limit = 20.0f},
};

struct dji_motor_inst m3508_7 = {
    .type = DJI_M3508,
    .pid_v2e = {.kp = 0.05f, .ki = 0.00003f, .kd = 0.0f, .i_limit = 1.0f, .out_limit = 20.0f},
};

struct dji_motor_inst m3508_8 = {
    .type = DJI_M3508,
    .pid_v2e = {.kp = 0.05f, .ki = 0.0001f, .kd = 0.0f, .i_limit = 0.5f, .out_limit = 20.0f},
};

struct dji_motor_inst *motor = &gm6020_4;

/* function definition */
void motor_init()
{
	/* initialize gm6020 1 */
	struct can_rx_config can_gm6020_1 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan2,
	    .id = 0x205, /* 0x204 + 1 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	gm6020_1.can_rx_inst = can_register_rx(&can_gm6020_1);
	can_set_user_data(gm6020_1.can_rx_inst, &gm6020_1);

	/* initialize gm6020 4 */
	struct can_rx_config can_gm6020_4 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan2,
	    .id = 0x208, /* 0x204 + 4 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	gm6020_4.can_rx_inst = can_register_rx(&can_gm6020_4);
	can_set_user_data(gm6020_4.can_rx_inst, &gm6020_4);

	/* initialize gm6020 2 */
	struct can_rx_config can_gm6020_2 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan1,
	    .id = 0x206, /* 0x204 + 2 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	gm6020_2.can_rx_inst = can_register_rx(&can_gm6020_2);
	can_set_user_data(gm6020_2.can_rx_inst, &gm6020_2);

	/* initialize gm6020 3 */
	struct can_rx_config can_gm6020_3 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan1,
	    .id = 0x207, /* 0x204 + 3 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	gm6020_3.can_rx_inst = can_register_rx(&can_gm6020_3);
	can_set_user_data(gm6020_3.can_rx_inst, &gm6020_3);

	/* initialize m3508 1 */
	struct can_rx_config can_m3508_5 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan1,
	    .id = 0x205, /* 0x200 + 5 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_5.can_rx_inst = can_register_rx(&can_m3508_5);
	can_set_user_data(m3508_5.can_rx_inst, &m3508_5);

	/* initialize m3508 4 */
	struct can_rx_config can_m3508_8 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan1,
	    .id = 0x208, /* 0x200 + 8 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_8.can_rx_inst = can_register_rx(&can_m3508_8);
	can_set_user_data(m3508_8.can_rx_inst, &m3508_8);

	/* initialize m3508 2 */
	struct can_rx_config can_m3508_6 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan2,
	    .id = 0x206, /* 0x200 + 6 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_6.can_rx_inst = can_register_rx(&can_m3508_6);
	can_set_user_data(m3508_6.can_rx_inst, &m3508_6);

	/* initialize m3508 3 */
	struct can_rx_config can_m3508_7 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan2,
	    .id = 0x207, /* 0x200 + 7 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_7.can_rx_inst = can_register_rx(&can_m3508_7);
	can_set_user_data(m3508_7.can_rx_inst, &m3508_7);

	/* register tx instance */
	struct can_tx_config can_config_1 = {
	    .hfdcan = &hfdcan1,
	    .id = 0x1FF,
	    .type = CAN_STANDARD,
	};
	can_1 = can_register_tx(&can_config_1);

	struct can_tx_config can_config_2 = {
	    .hfdcan = &hfdcan2,
	    .id = 0x1FF,
	    .type = CAN_STANDARD,
	};
	can_2 = can_register_tx(&can_config_2);
}

void m3508_set_vel(float front_left, float back_left, float back_right, float front_right)
{
	set_single_vel(&m3508_5, front_left * DJI_M3508_REDUCTION_RATE);
	set_single_vel(&m3508_6, back_left * DJI_M3508_REDUCTION_RATE);
	set_single_vel(&m3508_7, back_right * DJI_M3508_REDUCTION_RATE);
	set_single_vel(&m3508_8, front_right * DJI_M3508_REDUCTION_RATE);
}

void gm6020_set_vel(float front_left, float back_left, float back_right, float front_right)
{
	set_single_vel(&gm6020_2, front_left);
	set_single_vel(&gm6020_1, back_left);
	set_single_vel(&gm6020_4, back_right);
	set_single_vel(&gm6020_3, front_right);
}

void gm6020_set_pos(float front_left, float back_left, float back_right, float front_right)
{
	set_single_pos(&gm6020_2, front_left);
	set_single_pos(&gm6020_1, back_left);
	set_single_pos(&gm6020_4, back_right);
	set_single_pos(&gm6020_3, front_right);
}

void motor_set_command()
{
	/* transmit buffer for can1 and can2 */
	uint8_t buff1[8], buff2[8];

	buff1[0] = (m3508_5.cmd >> 8) & 0xFF; /* first motor */
	buff1[1] = m3508_5.cmd & 0xFF;
	buff1[2] = (gm6020_2.cmd >> 8) & 0xFF; /* second motor */
	buff1[3] = gm6020_2.cmd & 0xFF;
	buff1[4] = (gm6020_3.cmd >> 8) & 0xFF; /* third motor */
	buff1[5] = gm6020_3.cmd & 0xFF;
	buff1[6] = (m3508_8.cmd >> 8) & 0xFF; /* last motor */
	buff1[7] = m3508_8.cmd & 0xFF;

	buff2[0] = (gm6020_1.cmd >> 8) & 0xFF; /* first motor */
	buff2[1] = gm6020_1.cmd & 0xFF;
	buff2[2] = (m3508_6.cmd >> 8) & 0xFF; /* second motor */
	buff2[3] = m3508_6.cmd & 0xFF;
	buff2[4] = (m3508_7.cmd >> 8) & 0xFF; /* third motor */
	buff2[5] = m3508_7.cmd & 0xFF;
	buff2[6] = (gm6020_4.cmd >> 8) & 0xFF; /* last motor */
	buff2[7] = gm6020_4.cmd & 0xFF;

	can_transmit(can_1, buff1);
	can_transmit(can_2, buff2);
}

static void motor_callback(struct can_rx_inst *can_inst, uint8_t *buff)
{
	if (can_inst != NULL) {
		struct dji_motor_inst *motor = can_get_user_data(can_inst);
		if (motor != NULL) {
			motor->raw_pos = (int16_t)((buff[0] << 8) | buff[1]);
			motor->raw_vel = (int16_t)((buff[2] << 8) | buff[3]);
			motor->raw_eff = (int16_t)((buff[4] << 8) | buff[5]);

			/* get pos, vel and effort */
			raw_pos_into_pos(motor);
			motor->vel = DJI_RPM_TO_RADS(motor->raw_vel);
			if (motor->type == DJI_GM6020) {
				motor->eff = DJI_GM6020_CURRENT_INT_TO_FLOAT(motor->raw_eff);
			} else if (motor->type == DJI_M3508) {
				motor->eff = DJI_M3508_CURRENT_INT_TO_FLOAT(motor->raw_eff);
			}
		}
	}
}

static void set_single_vel(struct dji_motor_inst *motor, float ref)
{
	float cmd = pid_calculate(&(motor->pid_v2e), ref, motor->vel);
	switch (motor->type) {
	case DJI_M2006:
		break;
	case DJI_M3508:
		motor->cmd = DJI_M3508_CURRENT_FLOAT_TO_INT(cmd);
		break;
	case DJI_GM6020:
		motor->cmd = DJI_GM6020_VOLTAGE_FLOAT_TO_INT(cmd);
	}
}

static void set_single_pos(struct dji_motor_inst *motor, float ref)
{
	/* update the reference: the difference should within PI */
	if (ref - motor->pos > PI) {
		ref -= 2 * PI;
	} else if (ref - motor->pos < -PI) {
		ref += 2 * PI;
	}

	float cmd = pid_calculate(&(motor->pid_p2v), ref, motor->pos);
	set_single_vel(motor, cmd);
}
