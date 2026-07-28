#include "f103.h"
#include <string.h>

struct usart_inst *usart10 = NULL;
uint8_t f103_data[100];

static void callback(uint8_t *buff, uint16_t len)
{
	if (len < 100 && buff != NULL) {
		memcpy(f103_data, buff, len);
	}
}

void f103_init(void)
{
	struct usart_config config = {
	    .callback = callback,
	    .huart = &huart10,
	    .tx_mode = USART_TRANSMIT_IT,
	    .rx_mode = USART_RECEIVE_IDLE_DMA_CIRCULAR,
	};
	usart10 = usart_register(&config);
}

HAL_StatusTypeDef f103_send(enum linear_actuator_state linear_actuator_cmd, enum esc_state esc_cmd)
{
	uint8_t buff[2] = {(uint8_t)linear_actuator_cmd, esc_cmd};
	return usart_transmit(usart10, buff, 2);
}
