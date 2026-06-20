#include "bsp_usart.h"
#include <assert.h>

#define USART_TIMEOUT_MS HAL_MAX_DELAY

struct usart_inst {
	UART_HandleTypeDef *huart;
	enum usart_receive_mode rx_mode;
	enum usart_transmit_mode tx_mode;
	usart_callback callback;

	uint8_t *rx_buff;
};

static struct usart_inst usart_inst[USART_INST_MAX_NUM];
static uint8_t idx = 0;

struct usart_inst *usart_register(const struct usart_config *config)
{
	assert(config != NULL && config->huart != NULL);

	/* check repetition */
	for (uint8_t i = 0; i < idx; i++) {
		if (usart_inst[i].huart == config->huart) {
			return (usart_inst + i);
		}
	}

	struct usart_inst *inst = (usart_inst + idx);
	inst->huart = config->huart;
	inst->rx_mode = config->rx_mode;
	inst->tx_mode = config->tx_mode;
	inst->callback = config->callback;

	return inst;
}

HAL_StatusTypeDef usart_receive(struct usart_inst *inst, uint8_t *buff, uint16_t len)
{
	assert(inst != NULL && inst->huart != NULL);
	HAL_StatusTypeDef ret = HAL_ERROR;

	/* only the polling mode needs to receive manually */
	if (inst->rx_mode == USART_RECEIVE_POLLING) {
		ret = HAL_UART_Receive(inst->huart, buff, len, USART_TIMEOUT_MS);
	}

	return ret;
}

HAL_StatusTypeDef usart_transmit(struct usart_inst *inst, uint8_t *buff, uint16_t len)
{
	assert(inst != NULL && inst->huart != NULL);
	HAL_StatusTypeDef ret = HAL_ERROR;

	switch (inst->tx_mode) {
	case USART_TRANSMIT_NONE:
		break;
	case USART_TRANSMIT_POLLING:
		ret = HAL_UART_Transmit(inst->huart, buff, len, USART_TIMEOUT_MS);
		break;
	}

	return ret;
}