#ifndef F103_H_
#define F103_H_

#include "bsp_usart.h"

enum linear_actuator_state {
	LINEAR_ACTUATOR_STOP = 0x00, /* 0b 0000 0000 */
	LINEAR_ACTUATOR_UP = 0xF0,   /* 0b 1111 0000 */
	LINEAR_ACTUATOR_DOWN = 0x0F, /* 0b 0000 1111 */
};

enum esc_state {
	ESC_STOP = 0x00, /* 0b 0000 0000 */
	ESC_RUN = 0xF0,	 /* 0b 1111 0000 */
};

struct rc_cmd {
	enum esc_state esc_cmd;
	enum linear_actuator_state linear_actuator_cmd;
};

void f103_init(void);
HAL_StatusTypeDef f103_send(enum linear_actuator_state linear_actuator_cmd, enum esc_state esc_cmd);

#endif /* F103_H_ */
