#ifndef DJI_MOTOR_H_
#define DJI_MOTOR_H_

#include "bsp_fdcan.h"

#define DJI_MOTOR_INST_MAX 16

struct dji_motor_inst;

struct dji_motor_info {
	float pos; /* in radius, 0 ~ 2 pi */
	float vel; /* in rad */
	float eff; /* current or voltage */
};

enum dji_motor_type {
	DJI_M2006 = 0,
	DJI_M3508,
	DJI_GM6020,
};

enum dji_motor_freq {
	FREQ_125_HZ = 8, /* 125 * 8 = 1000 */
	FREQ_250_HZ = 4,
	FREQ_500_Hz = 2,
	FREQ_1000_HZ = 1,
};

// enum dji_motor_id {
// 	DJI_MOTOR_ID_1 = 1,
// 	DJI_MOTOR_ID_2,
// 	DJI_MOTOR_ID_3,
// 	DJI_MOTOR_ID_4,
// 	DJI_MOTOR_ID_5,
// 	DJI_MOTOR_ID_6,
// 	DJI_MOTOR_ID_7,
// 	DJI_MOTOR_ID_8, /* not for GM6020 */
// };

// struct motor_config {
// 	enum dji_motor_type type;
// 	enum dji_motor_freq freq;

// 	struct can_rx_inst *rx_inst;
// 	struct can_tx_inst *tx_inst;
// };

void motor_init(void);

void gm6020_set_pos(float front_left, float front_right, float back_left, float back_right);
void gm6020_set_vel(float front_left, float front_right, float back_left, float back_right);
void gm6020_set_pos(float front_left, float front_right, float back_left, float back_right);

/* this function should be executed in a 1000hz loop */
void motor_set_command(void);

#endif /* DJI_MOTOR_H_ */