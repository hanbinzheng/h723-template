#include "motor.h"
#include "bsp_fdcan.h"
#include "pid.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

/* for position and velocity */
#define RPM_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 60.0f)
#define ANGLE_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 8192.0f)

/* GM6020 */
#define GM6020_CURRENT_FLOAT_TO_INT(value)                                                         \
	((int16_t)((value) * 16384.0f / 3.0f)) /* -3A~0~3A, -16384~0~16384 */
#define GM6020_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 3.0f / 16384.0f)
#define GM6020_VOLTAGE_FLOAT_TO_INT(value)                                                         \
	((int16_t)((value) * 25000.0f / 24.0f)) /* -24v~0~24v, -25000~0~25000 */

/* M3508 */
#define M3508_CURRENT_FLOAT_TO_INT(value)                                                          \
	((int16_t)((value) * 16384.0f / 20.0f)) /* -20A~0~20A, -16384~0~16384 */
#define M3508_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 20.0f / 16384.0f)
#define M3508_REDUCE_RATE 19.203208556149733f /* 3591 / 187 */

struct dji_motor_inst {
	enum dji_motor_type type;

	int16_t raw_pos;
	int16_t raw_vel;
	int16_t raw_eff;

	float pos;
	float vel;
	float eff;

	struct pid_info pid;

	struct can_rx_inst *can_rx_inst;

	uint16_t cnt;
};

static struct dji_motor_inst gm6020_1 = {
    .type = DJI_GM6020,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst gm6020_2 = {
    .type = DJI_GM6020,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst gm6020_3 = {
    .type = DJI_GM6020,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst gm6020_4 = {
    .type = DJI_GM6020,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst m3508_1 = {
    .type = DJI_M3508,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst m3508_2 = {
    .type = DJI_M3508,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst m3508_3 = {
    .type = DJI_M3508,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

static struct dji_motor_inst m3508_4 = {
    .type = DJI_M3508,
    .pid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f},
};

void motor_callback(struct can_rx_inst *inst, uint8_t *buff);

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
	struct can_rx_config can_m3508_1 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan1,
	    .id = 0x201, /* 0x200 + 1 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_1.can_rx_inst = can_register_rx(&can_m3508_1);
	can_set_user_data(m3508_1.can_rx_inst, &m3508_1);

	/* initialize m3508 4 */
	struct can_rx_config can_m3508_4 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan1,
	    .id = 0x204, /* 0x200 + 4 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_4.can_rx_inst = can_register_rx(&can_m3508_4);
	can_set_user_data(m3508_4.can_rx_inst, &m3508_4);

	/* initialize m3508 2 */
	struct can_rx_config can_m3508_2 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan2,
	    .id = 0x202, /* 0x200 + 2 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_2.can_rx_inst = can_register_rx(&can_m3508_2);
	can_set_user_data(m3508_2.can_rx_inst, &m3508_2);

	/* initialize m3508 3 */
	struct can_rx_config can_m3508_3 = {
	    .callback = motor_callback,
	    .hfdcan = &hfdcan2,
	    .id = 0x203, /* 0x200 + 3 */
	    .mask = 0x7FF,
	    .type = CAN_STANDARD,
	};
	m3508_3.can_rx_inst = can_register_rx(&can_m3508_3);
	can_set_user_data(m3508_3.can_rx_inst, &m3508_3);
}

void motor_callback(struct can_rx_inst *can_inst, uint8_t *buff)
{
	if (can_inst != NULL) {
		struct dji_motor_inst *motor = can_get_user_data(can_inst);
		if (motor != NULL) {
			motor->raw_pos = (int16_t)((buff[0] << 8) | buff[1]);
			motor->raw_vel = (int16_t)((buff[2] << 8) | buff[3]);
			motor->raw_eff = (int16_t)((buff[4] << 8) | buff[5]);
			motor->pos = ANGLE_TO_RADS(motor->raw_pos);
			motor->vel = RPM_TO_RADS(motor->raw_vel);
			motor->cnt++;

			if (motor->type == DJI_GM6020) {
				motor->eff = GM6020_CURRENT_FLOAT_TO_INT(motor->raw_eff);
			} else if (motor->type == DJI_M3508) {
				motor->eff = M3508_CURRENT_INT_TO_FLOAT(motor->raw_eff);
			}
		}
	}
}