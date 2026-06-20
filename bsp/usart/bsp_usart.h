#ifndef BSP_USART_H_
#define BSP_USART_H_

#include "usart.h"
#include <stdint.h>

#define USART_INST_MAX_NUM 5
#define USART_TIMEOUT_MS HAL_MAX_DELAY
#define USART_BUFF_MAX_SIZE 128

struct usart_inst;
typedef void (*usart_callback)(uint8_t *rx_buff, uint16_t len);

enum usart_receive_mode {
	USART_RECEIVE_NONE = 0,
	USART_RECEIVE_POLLING,
	USART_RECEIVE_IT, /* FIFO */
			  // USART_RECEIVE_DMA, /* double DMA + IDLE interrupt */
};

enum usart_transmit_mode {
	USART_TRANSMIT_NONE = 0,
	USART_TRANSMIT_POLLING,
	USART_TRANSMIT_IT, /* FIFO */
			   // USART_TRANSMIT_DMA,
};

struct usart_config {
	UART_HandleTypeDef *huart;
	enum usart_receive_mode rx_mode;
	enum usart_transmit_mode tx_mode;
	uint16_t rx_len;
	uint8_t *rx_buff;	 /* not for polling mode */
	usart_callback callback; /* callback function for rx interrupt */
};

/**
 * @brief register a USART instance
 *
 * @note this should be called after the initialization of all peripherals.
 *
 * @param config USART instance configuration struct
 * @return pointer to the USART instance
 *
 */
struct usart_inst *usart_register(const struct usart_config *config);

/**
 * @brief receive data in polling mode
 *
 * blocking receive in polling mode
 *
 * @note this function is not for DMA
 *
 * @param inst pointer to the USART instance
 * @param buff receive buffer, only for polling mode
 * @param len number of bytes to receive, only for polling mode
 * @return whether successful
 */
HAL_StatusTypeDef usart_receive(struct usart_inst *inst, uint8_t *buff, uint16_t len);

/**
 * @brief transmit
 *
 * blocking time in polling mode: 100ms
 *
 * @param inst pointer to the USART instance
 * @param buff transmit buffer
 * @param len number of bytes to transmit in tx buffer
 * @return whether successful
 */
HAL_StatusTypeDef usart_transmit(struct usart_inst *inst, uint8_t *buff, uint16_t len);

#endif /* BSP_USART_H_ */