#include "bsp_usart.h"
#include <assert.h>

struct usart_inst {
	UART_HandleTypeDef *huart;
	enum usart_receive_mode rx_mode;
	enum usart_transmit_mode tx_mode;
	uint16_t rx_len;
	uint8_t *rx_buff;
	usart_callback callback;
};

static struct usart_inst usart_inst[USART_INST_MAX_NUM];
static uint8_t idx = 0;

struct usart_inst *usart_register(const struct usart_config *config)
{
	assert(config != NULL && config->huart != NULL && idx < USART_INST_MAX_NUM);

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
	inst->rx_len = config->rx_len;
	inst->callback = config->callback;

	switch (inst->rx_mode) {
	case USART_RECEIVE_NONE:
		break;
	case USART_RECEIVE_POLLING:
		break;
	case USART_RECEIVE_IT:
		inst->rx_buff = config->rx_buff;
		HAL_UART_Receive_IT(inst->huart, config->rx_buff, inst->rx_len);
		break;
	default:
		break;
	}

	idx++;
	return inst;
}

HAL_StatusTypeDef usart_receive(struct usart_inst *inst, uint8_t *buff, uint16_t len)
{
	assert(inst != NULL && inst->huart != NULL && buff != NULL);
	HAL_StatusTypeDef ret = HAL_ERROR;

	switch (inst->rx_mode) {
	case USART_RECEIVE_NONE:
		break;
	case USART_RECEIVE_POLLING:
		ret = HAL_UART_Receive(inst->huart, buff, len, USART_TIMEOUT_MS);
		break;
	case USART_RECEIVE_IT:
		inst->rx_buff = buff;
		inst->rx_len = len;
		ret = HAL_UART_Receive_IT(inst->huart, inst->rx_buff, inst->rx_len);
		break;
	default:
		break;
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
	case USART_TRANSMIT_IT:
		ret = HAL_UART_Transmit_IT(inst->huart, buff, len);
		break;
	default:
		break;
	}

	return ret;
}

/**
 * @brief UART Interrupt-Driven Transmission Flow Summary
 *
 * check stm32h7xx_hal_uart.c for details
 *
 * 1. User calls HAL_UART_Transmit_IT() to start transmission.
 *    - Stores user buffer pointer, size, and remaining count into UART handle.
 *    - Enables TXEIE (TX register empty) or TXFTIE (TX FIFO threshold) interrupt.
 *    - Returns immediately (non-blocking).
 *
 * 2. Hardware triggers TXE/TXFT interrupt when TX register/FIFO can accept data.
 *    - Calls huart->TxISR() to move next byte(s) from buffer to TX register/FIFO.
 *    - Decrements TxXferCount each time data is written.
 *    - When TxXferCount reaches 0: disables TXE/TXFT interrupt, enables TCIE (TC interrupt).
 *
 * 3. Hardware triggers TC (Transmission Complete) interrupt when the last bit
 *    (including stop bit) has fully left the TX pin.
 *    - Calls UART_EndTransmit_IT(), which:
 *      a) Disables TCIE
 *      b) Sets gState back to READY
 *      c) Calls the user callback (see below)
 *
 * 4. User-overridable callbacks (implement these in your code):
 *    - HAL_UART_TxCpltCallback()  -> Called after TC interrupt (all data fully sent).
 *    - HAL_UARTEx_TxFifoEmptyCallback() -> (FIFO mode only) Called when TX FIFO becomes empty,
 *                                           but last byte may still be shifting out.
 *                                           Early notification, NOT a replacement for TC callback.
 *
 * Interrupt sources in HAL_UART_IRQHandler() (in priority order):
 *   - TXE/TXFT  : TX register empty / FIFO threshold reached -> TxISR feeds next data.
 *   - TC        : Transmission complete (last bit sent)      -> HAL_UART_TxCpltCallback()
 *   - TXFE      : (FIFO mode) FIFO empty                     -> HAL_UARTEx_TxFifoEmptyCallback()
 *
 */

/**
 * @brief  UART TX Fifo empty callback.
 * @param  huart UART handle.
 * @retval None
 *
 * @note called in HAL_UART_IRQHandler(), check stm32h7xx_hal_uart.c for details.
 * @note TXFE (TX FIFO Full Empty) in our code hasn't been opened, so this will not be called
 */
/*
void HAL_UARTEx_TxFifoEmptyCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

/**
 * @brief Tx Transfer completed callback.
 * @param huart UART handle.
 * @retval None
 *
 * @note called in UART_DMATransmitCplt() and UART_EndTransmit_IT(),
 * 	check stm32h7xx_hal_uart.c for details
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	return;
}

/**
 * @brief UART Interrupt-Driven Reception Flow Summary
 *
 * check stm32h7xx_hal_uart.c for details
 *
 * 1. User calls HAL_UART_Receive_IT() to start reception ( call UART_Start_Receive_IT() )
 * - Stores user buffer pointer, total size, and remaining count (RxXferCount) into UART handle.
 * - Configures huart->RxISR function pointer according to WordLength and FIFO settings.
 * - Enables RXNEIE (RX register not empty) or RXFTIE (RX FIFO threshold) along with error
 * 	interrupts.
 * - Returns immediately (non-blocking).
 *
 * 2. Hardware triggers RXNE/RXFT interrupt when data is received from external device.
 * - HAL_UART_IRQHandler() detects the flag and calls huart->RxISR().
 * - huart->RxISR() moves received byte(s) from RDR register / RX FIFO to the user buffer.
 * - Increments user buffer pointer and decrements huart->RxXferCount each time data is read.
 *
 * 3. Reception Completes when huart->RxXferCount reaches 0.
 * - Unlike transmission, there is no specific "reception complete" hardware interrupt.
 * 	The end of transfer is determined inside RxISR when the count hits zero.
 * - The internal logic then:
 * 	a) Disables RXNEIE / RXFTIE and error interrupts.
 * 	b) Sets RxState back to READY.
 * 	c) Calls the user complete callback (see below).
 *
 * 4. User-overridable callbacks (implement these in your code):
 * - HAL_UART_RxCpltCallback()  -> Called after the last requested byte is received and stored.
 * - HAL_UARTEx_RxFifoFullCallback() -> (FIFO mode only) when RX FIFO becomes completely full.
 * - HAL_UART_ErrorCallback()   -> if any error (Overrun ORE, Noise NE, Frame FE, Parity PE) occurs.
 * - HAL_UARTEx_RxEventCallback() -> when an IDLE event occurs (if Reception till IDLE is selected).
 *
 * Interrupt sources in HAL_UART_IRQHandler() (in priority order):
 * - Error Flags: ORE (Overrun), FE (Frame), NE (Noise), PE (Parity) -> Disables Rx or triggers
 * 			ErrorCallback.
 * - RXNE/RXFT : RX register not empty / FIFO threshold reached   -> RxISR reads and stores data.
 * - IDLE : Idle line detected (if IDLEIE enabled) -> Triggers HAL_UARTEx_RxEventCallback().
 * - RXFF : (FIFO mode) FIFO full  -> HAL_UARTEx_RxFifoFullCallback().
 *
 */

/**
 * @brief  UART RX Fifo full callback.
 * @param  huart UART handle.
 * @retval None
 */
/*
void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

/**
 * @brief  Reception Event Callback (Rx event notification called after use of advanced reception
 * service).
 *
 * @note This callback function is for ToIdle
 *
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	return;
}

/**
 * @brief  Rx Transfer completed callback.
 *
 * @note This function is for HAL_UART_Transmit_IT without Idle
 * 	called by UART_RxISR_XBIT_xxxxxxxx
 * @param  huart UART handle.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	struct usart_inst *inst = NULL;

	/* TODO: use hash table instead of iteration */
	for (uint8_t i = 0; i < idx; i++) {
		inst = (usart_inst + i);
		if (inst->huart == huart && inst->callback != NULL) {
			inst->callback(inst->rx_buff, inst->rx_len);
		}
	}

	return;
}