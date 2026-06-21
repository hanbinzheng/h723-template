#include "bsp_usart.h"
#include <assert.h>

struct usart_inst {
	UART_HandleTypeDef *huart;
	enum usart_receive_mode rx_mode;
	enum usart_transmit_mode tx_mode;
	usart_callback callback;

	uint8_t *buff1; /* receive buffer or DMA buffer 1 */
	uint8_t *buff2; /* only for DMA buffer 2 */
	uint16_t len;	/* length of rx buffer */
};

static struct usart_inst usart_inst[USART_INST_MAX_NUM];
static uint8_t idle_buff[USART_INST_MAX_NUM * USART_BUFF_MAX_SIZE];
__NOCACHE_DMA static uint8_t dma_buff1[USART_INST_MAX_NUM * USART_BUFF_MAX_SIZE];
__NOCACHE_DMA static uint8_t dma_buff2[USART_INST_MAX_NUM * USART_BUFF_MAX_SIZE];
static uint8_t idx = 0;

/* TODO: use hash table instead of iteration */
__ALWAYS_INLINE
static struct usart_inst *get_usart_inst(UART_HandleTypeDef *huart)
{
	struct usart_inst *inst = NULL;

	for (uint8_t i = 0; i < idx; i++) {
		inst = (usart_inst + i);
		if (inst->huart == huart) {
			return inst;
		}
	}

	return inst; /* fail to find */
}

static void usart_idle_config(struct usart_inst *inst)
{
	inst->buff1 = (idle_buff + idx * USART_BUFF_MAX_SIZE);
	inst->buff2 = NULL;
	inst->len = USART_BUFF_MAX_SIZE;

	/* check stm32h7xx_hal_uart_ex.c and stm32h7xx_hal_uart.c for details */
	HAL_UARTEx_ReceiveToIdle_IT(inst->huart, inst->buff1, inst->len);
}

/* reference: https://zhuanlan.zhihu.com/p/720966722 */
static void usart_idle_dma_config(struct usart_inst *inst)
{
	inst->buff1 = (dma_buff1 + idx * USART_BUFF_MAX_SIZE);
	inst->buff2 = (dma_buff2 + idx * USART_BUFF_MAX_SIZE);
	inst->len = USART_BUFF_MAX_SIZE;
	UART_HandleTypeDef *huart = inst->huart;

	/* UART IDLE reception mode */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	huart->RxEventType = HAL_UART_RXEVENT_IDLE;
	huart->RxXferSize = inst->len;

	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); /* Enable DMA */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);     /* Enable IDLE interrupt */

	/* Configure DMA double buffer */
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR,
				   (uint32_t)inst->buff1, (uint32_t)inst->buff2, inst->len);
}

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
	inst->callback = config->callback;

	switch (inst->rx_mode) {
	case USART_RECEIVE_IT_IDLE:
		usart_idle_config(inst);
		break;
	case USART_RECEIVE_IT_IDLE_DMA:
		usart_idle_dma_config(inst);
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
	case USART_RECEIVE_POLLING:
		ret = HAL_UART_Receive(inst->huart, buff, len, USART_TIMEOUT_MS);
		break;
	case USART_RECEIVE_IT:
		inst->buff1 = buff;
		inst->len = len;
		ret = HAL_UART_Receive_IT(inst->huart, buff, len);
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
/* not in use
void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

/**
 * @brief UART Interrupt-Driven Advanced Uncertain-Length (ReceiveToIdle) Reception Flow Summary
 *
 * check stm32h7xx_hal_uart.c for details
 *
 * 1. User calls HAL_UARTEx_ReceiveToIdle_IT() to start advanced uncertain length listening.
 * - Marks huart->ReceptionType as HAL_UART_RECEPTION_TOIDLE.
 * - Internally calls UART_Start_Receive_IT() to hook the specific byte-by-byte
 * 	carrier (huart->RxISR).
 * - Explicitly sets the USART_CR1_IDLEIE bit to activate hardware bus idle detection.
 * - Size parameter here acts as a strict "safety upper bound protection fence" to
 *	avoid memory corruption.
 *
 * 2. Stage I: High-Speed Streaming Phase (Driven by RXNE Interrupt)
 * - Every single byte arriving at the RDR register forces HAL_UART_IRQHandler() to
	call huart->RxISR().
 * - The data is dumped into the internal buffer, and huart->RxXferCount decrements accordingly.
 *
 * 3. Stage II: Packet Termination Phase (Driven by IDLE or Buffer Full)
 * - Scenario A: The external device stops sending data. The bus remains idle for more
	than 1 byte time.
 * - HAL_UART_IRQHandler() detects the IDLE flag, calculates: Size - RxXferCount.
 * - It completely disables RXNEIE, PEIE, and IDLEIE to secure the current buffer.
 * - Marks RxEventType as HAL_UART_RXEVENT_IDLE and safely releases RxState to READY.
 * - Calls the user-overridable event callback: HAL_UARTEx_RxEventCallback(huart, actual_len).
 *
 * - Scenario B: The external sender continues blasting raw data without ever idling,
	hitting the buffer limit.
 * - huart->RxXferCount reaches 0 inside RxISR.
 * - The internal logic automatically shuts down interrupts,
	sets RxEventType to HAL_UART_RXEVENT_TC.
 * - Calls the legacy weak completion callback: HAL_UART_RxCpltCallback(huart).
 *
 * 4. User-overridable callbacks (implement these in your code):
 * - HAL_UART_RxCpltCallback() -> Called ONLY when the buffer limit (Size) is completely saturated.
 * - HAL_UARTEx_RxEventCallback() -> Called when a real total-bus idle event is
 * 	captured (Uncertain packets).
 * - HAL_UART_ErrorCallback() -> Triggered if ORE (Overrun), FE (Frame), or NE (Noise) occurs
 *
 * Interrupt sources in HAL_UART_IRQHandler() (in priority order):
 * - Error Flags: ORE(Overrun), FE(Frame), NE(Noise), PE(Parity) -> Halts Rx and invokes
 * 		ErrorCallback.
 * - RXNE/RXFT : RX register not empty / FIFO threshold reached -> RxISR shifts and
		stores data bytes.
 * - IDLE : Idle line detected (if IDLEIE enabled) -> Triggers HAL_UARTEx_RxEventCallback()
 * - RXFF : (FIFO mode) RX FIFO full  -> HAL_UARTEx_RxFifoFullCallback().
 */

static void usart_idle_callback(struct usart_inst *inst, uint16_t len)
{
	/* user callback to handle the data */
	if (inst->callback != NULL) {
		inst->callback(inst->buff1, len);
	}

	/* for next time idle interrupt */
	HAL_UARTEx_ReceiveToIdle_IT(inst->huart, inst->buff1, inst->len);
}

/* reference: https://zhuanlan.zhihu.com/p/720966722 */
static void usart_idle_dma_callback(struct usart_inst *inst, uint16_t len)
{
	UART_HandleTypeDef *huart = inst->huart;

	__HAL_DMA_DISABLE(huart->hdmarx); /* Disable DMA */

	/* Check DMA current buffer */
	if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
		/* Change DMA buffer and reset NDTR */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

		/* user callback function to handle data */
		if (inst->callback != NULL) {
			inst->callback(inst->buff1, len);
		}
	} else {
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

		if (inst->callback != NULL) {
			inst->callback(inst->buff2, len);
		}
	}

	__HAL_DMA_SET_COUNTER(huart->hdmarx, inst->len); /* reset length */
	__HAL_DMA_ENABLE(huart->hdmarx);		 /* Enable DMA */
}

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
	struct usart_inst *inst = get_usart_inst(huart);
	if (inst == NULL) {
		return;
	}

	switch (inst->rx_mode) {
	case USART_RECEIVE_IT_IDLE:
		usart_idle_callback(inst, Size);
		break;
	case USART_RECEIVE_IT_IDLE_DMA:
		usart_idle_dma_callback(inst, Size);
		break;
	default:
		return;
	}
}

/**
 * @brief UART Interrupt-Driven Standard Fixed-Length Reception Flow Summary
 *
 * check stm32h7xx_hal_uart.c for details
 *
 * 1. User calls HAL_UART_Receive_IT() to start reception ( calls UART_Start_Receive_IT() )
 * - Stores user buffer pointer, total size, and remaining count (RxXferCount) into UART handle.
 * - Configures huart->RxISR function pointer according to WordLength and FIFO settings.
 * - Enables RXNEIE (RX register not empty) or RXFTIE (RX FIFO threshold) along with
	error interrupts.
 * - Returns immediately (non-blocking).
 *
 * 2. Hardware triggers RXNE/RXFT interrupt when data is received from external device.
 * - HAL_UART_IRQHandler() detects the flag and calls huart->RxISR().
 * - huart->RxISR() moves received byte(s) from RDR register / RX FIFO to the user buffer.
 * - Increments user buffer pointer and decrements huart->RxXferCount each time data is read.
 *
 * 3. Reception Completes when huart->RxXferCount reaches 0.
 * - Unlike transmission, there is no specific "reception complete" hardware interrupt.
 * The end of transfer is determined inside RxISR when the count hits zero.
 * - The internal logic then:
 * a) Disables RXNEIE / RXFTIE and error interrupts.
 * b) Sets RxState back to READY.
 * c) Calls the user complete callback: HAL_UART_RxCpltCallback().
 */

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
	struct usart_inst *inst = get_usart_inst(huart);

	if (inst->callback != NULL) {
		inst->callback(inst->buff1, inst->len);
	}
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
 * @brief Tx Transfer completed callback.
 * @param huart UART handle.
 * @retval None
 *
 * @note called in UART_DMATransmitCplt() and UART_EndTransmit_IT(),
 * 	check stm32h7xx_hal_uart.c for details
 */
/* TODO: enable callback for Tx Complete Interrupt, this will be called after transmit IT finished
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

/**
 * @brief  UART TX Fifo empty callback.
 * @param  huart UART handle.
 * @retval None
 *
 * @note called in HAL_UART_IRQHandler(), check stm32h7xx_hal_uart.c for details.
 * @note TXFE (TX FIFO Full Empty) in our code hasn't been opened, so this will not be called
 */
/* not in use
void HAL_UARTEx_TxFifoEmptyCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/