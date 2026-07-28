#include "rust_clean.h"
#include "f103.h"
#include "sbus.h"

static enum esc_state esc = ESC_STOP;
static enum linear_actuator_state linear_actuator = LINEAR_ACTUATOR_STOP;

HAL_StatusTypeDef debug_rust_clean = HAL_BUSY;

void rust_clean_task(void)
{
	const struct sbus_data *sbus = sbus_get_data();
	if (sbus->safe == SBUS_UNSAFE || sbus->sw2 == SBUS_SW_UP) {
		esc = ESC_STOP;
		linear_actuator = LINEAR_ACTUATOR_STOP;
		f103_send(LINEAR_ACTUATOR_STOP, ESC_STOP);
	} else {
		if (sbus->rs_x > 0.0f) {
			linear_actuator = LINEAR_ACTUATOR_DOWN;
		} else if (sbus->rs_x == 0.0f) {
			linear_actuator = LINEAR_ACTUATOR_STOP;
		} else if (sbus->rs_x < 0.0f) {
			linear_actuator = LINEAR_ACTUATOR_UP;
		}

		if (sbus->sw4 == SBUS_SW_DOWN) {
			esc = ESC_RUN;
		} else {
			esc = ESC_STOP;
		}

		debug_rust_clean = f103_send(linear_actuator, esc);
	}
}
