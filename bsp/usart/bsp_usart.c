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

/* static variables */
static struct usart_inst usart_inst[USART_INST_MAX_NUM];
static uint8_t idle_buff[USART_INST_MAX_NUM * USART_BUFF_MAX_SIZE];
__NOCACHE_DMA static uint8_t dma_buff1[USART_INST_MAX_NUM * USART_BUFF_MAX_SIZE];
__NOCACHE_DMA static uint8_t dma_buff2[USART_INST_MAX_NUM * USART_BUFF_MAX_SIZE];
static uint8_t idx = 0;

/* static helper functions */
static void usart_idle_config(struct usart_inst *inst);
static void usart_idle_dma_config(struct usart_inst *inst);
static void usart_idle_callback(struct usart_inst *inst, uint16_t len);
static void usart_idle_dma_callback(struct usart_inst *inst, uint16_t len);

__ALWAYS_INLINE static struct usart_inst *get_usart_inst(UART_HandleTypeDef *huart)
{
	/* TODO: use hash table instead of iteration */
	struct usart_inst *inst = NULL;

	for (uint8_t i = 0; i < idx; i++) {
		inst = (usart_inst + i);
		if (inst->huart == huart) {
			return inst;
		}
	}

	return inst; /* fail to find */
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
	case USART_RECEIVE_IDLE:
		usart_idle_config(inst);
		break;
	case USART_RECEIVE_IDLE_DMA_CIRCULAR:
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
	call huart->RxISR(). If FIFO is enabled, use USART_CR3_RXFTIE.
 * - The data is dumped into the internal buffer, and huart->RxXferCount decrements accordingly.
 *
 * 3. Stage II: Packet Termination Phase (Driven by IDLE or Buffer Full)
 * - Scenario A: The external device stops sending data. The bus remains idle for more
 *	than 1 byte time.
 * - HAL_UART_IRQHandler() detects the IDLE flag,
 * 	calculates: nb_rx_data = RxXferSize - RxXferCount.
 * - It completely disables RXNEIE, PEIE, and IDLEIE to secure the current buffer.
 * - Marks RxEventType as HAL_UART_RXEVENT_IDLE and safely releases RxState to READY.
 * - Calls the user-overridable event callback: HAL_UARTEx_RxEventCallback(huart, nb_rx_data).
 *
 * - Scenario B: The external sender continues blasting raw data without ever idling,
 *	hitting the buffer limit.
 * - huart->RxXferCount reaches 0 inside RxISR.
 * - The internal logic automatically shuts down interrupts,
 *	sets RxEventType to HAL_UART_RXEVENT_TC.
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
 *		stores data bytes.
 * - IDLE : Idle line detected (if IDLEIE enabled) -> Triggers HAL_UARTEx_RxEventCallback()
 * - RXFF : (FIFO mode) RX FIFO full  -> HAL_UARTEx_RxFifoFullCallback().
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	struct usart_inst *inst = get_usart_inst(huart);
	if (inst == NULL) {
		return;
	}

	switch (inst->rx_mode) {
	case USART_RECEIVE_IDLE:
		usart_idle_callback(inst, Size);
		break;
	case USART_RECEIVE_IDLE_DMA_CIRCULAR:
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
 *	error interrupts.
 * - Returns immediately (non-blocking, calling UART_Start_Receive_IT()).
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
 * 	c) Calls the user complete callback: HAL_UART_RxCpltCallback().
 *
 * 4. User-overridable callbacks (implement these in your code):
 * - HAL_UART_RxCpltCallback()  -> Called after the last requested byte is received and stored.
 * - HAL_UARTEx_RxFifoFullCallback() -> (FIFO only, and need manually enable the interrupt ) when
 *	RX FIFO becomes completely full.
 * - HAL_UART_ErrorCallback()   -> if any error (Overrun ORE, Noise NE, Frame FE, Parity PE) occurs.
 * - HAL_UARTEx_RxEventCallback() -> when an IDLE event occurs (if Reception till IDLE is selected).
 *
 * Interrupt sources in HAL_UART_IRQHandler() (in priority order):
 * - Error Flags: ORE (Overrun), FE (Frame), NE (Noise), PE (Parity) -> Disables Rx or triggers
 * 			ErrorCallback.
 * - RXNE/RXFT : RX register not empty / FIFO threshold reached   -> RxISR reads and stores data.
 * - IDLE : Idle line detected (if IDLEIE enabled) -> Triggers HAL_UARTEx_RxEventCallback().
 * - RXFF : (FIFO mode) FIFO full  -> HAL_UARTEx_RxFifoFullCallback()
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	struct usart_inst *inst = get_usart_inst(huart);

	if (inst->callback != NULL) {
		inst->callback(inst->buff1, inst->len);
	}
}

/* reserved
void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

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
 * 	It Calls huart->TxISR() to:
 *    - Move next byte(s) from buffer to TX register/FIFO.
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

/* TODO: enable Tx Complete Interrupt, this will be called after transmission finished
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

/* reserved
void HAL_UARTEx_TxFifoEmptyCallback(UART_HandleTypeDef *huart)
{
	return;
}
*/

static void usart_idle_config(struct usart_inst *inst)
{
	inst->buff1 = (idle_buff + idx * USART_BUFF_MAX_SIZE);
	inst->buff2 = NULL;
	inst->len = USART_BUFF_MAX_SIZE;

	/* check stm32h7xx_hal_uart_ex.c and stm32h7xx_hal_uart.c for details */
	HAL_UARTEx_ReceiveToIdle_IT(inst->huart, inst->buff1, inst->len);
}

static void usart_idle_callback(struct usart_inst *inst, uint16_t len)
{
	/* user callback to handle the data */
	if (inst->callback != NULL) {
		inst->callback(inst->buff1, len);
	}

	/* for next time idle interrupt */
	HAL_UARTEx_ReceiveToIdle_IT(inst->huart, inst->buff1, inst->len);
}

/*
 * reference: https://zhuanlan.zhihu.com/p/720966722
 * check HAL_UARTEx_ReceiveToIdle_DMA() in stm32h7xx_hal_uart_ex.c
 */
static void usart_idle_dma_config(struct usart_inst *inst)
{
	inst->buff1 = (dma_buff1 + idx * USART_BUFF_MAX_SIZE);
	inst->buff2 = (dma_buff2 + idx * USART_BUFF_MAX_SIZE);
	inst->len = USART_BUFF_MAX_SIZE;
	UART_HandleTypeDef *huart = inst->huart;

	/* configure USART with IDLE & DMA mode, check stm32h7xx_hal_uart.c for details */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	huart->RxEventType = HAL_UART_RXEVENT_IDLE;
	huart->RxXferSize = inst->len;
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); /* enable usart DMA mode */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);     /* enable USART IDLE interrupt */

	/** configure DMA with double buffer
	 *
	 * HAL_DMAEx_MultiBufferStart():
	 * - 1. enable the double buffer mode
	 *	((DMA_Stream_TypeDef   *)hdma->Instance)->CR |= DMA_SxCR_DBM
	 * - 2. configure DMA stream destination address (second)
	 *	((DMA_Stream_TypeDef   *)hdma->Instance)->M1AR = SecondMemAddress;
	 *		DMA_SxM01R register, M0AR(Memory 0 Address)
	 * - 3. clear all flags of the interrupt clear flag register (IFCR)
	 *	*ifcRegister_Base = 0x3FUL << (hdma->StreamIndex & 0x1FU)
	 * - 4. call DMA_MultiBufferSetConfig() for single DMA configure (see below)
	 * - 5. clear something
	 *
	 * Tha main procedure is in  DMA_MultiBufferSetConfig(), which:
	 * - 1. configure DMA stream data length
	 *	((DMA_Stream_TypeDef   *)hdma->Instance)->NDTR = DataLength;
	 * - 2. configure DMA stream source address and destination address
	 *	((DMA_Stream_TypeDef   *)hdma->Instance)->PAR
	 *		DMA_SxPAR register, PAR(Peripheral Address)
	 *	((DMA_Stream_TypeDef   *)hdma->Instance)->M0AR
	 *		DMA_SxM0AR register, M0AR(Memory 0 Address)
	 */
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR,
				   (uint32_t)inst->buff1, (uint32_t)inst->buff2, inst->len);
}

/**
 * @brief UART Interrupt-Driven (IDLE + DMA) Uncertain-Length DMA Reception Flow Summary
 *
 * check stm32h7xx_hal_uart_ex.c for details
 *
 * 1. User calls HAL_UARTEx_ReceiveToIdle_DMA() (omitted) or configure it manually to
 *	start DMA-assisted uncertain-length reception
 *    - Note: DMA  Normal or Circular mode (user choice):
 *        a) Normal mode:   DMA stops after RxXferSize bytes are transferred.
 *        b) Circular mode: DMA wraps around and never stops (continuous streaming).
 *
 * 2. Hardware streams data via DMA (no CPU intervention for data movement).
 *    - DMA moves bytes from USART_RDR to user buffer autonomously.
 *    - DMA_NDTR (remaining count) decrements on each transferred byte.
 *    - CPU stays idle (or executes other tasks) during this phase.
 *
 * 3. Packet Termination Phase (Driven by IDLE interrupt or DMA completion):
 *    - Scenario A: Bus becomes idle before buffer fills up (Normal DMA mode).
 *      a) UART hardware asserts IDLE flag in USART_ISR.
 *      b) HAL_UART_IRQHandler() calls UART_EndRxTransfer().
 *      c) Calculates actual received length:
 *         nb_rx_data = RxXferSize - __HAL_DMA_GET_COUNTER(hdmarx)
 *      d) Disables DMAR (stops DMA), IDLEIE, PEIE, EIE.
 *      e) Calls HAL_DMA_Abort() to force-stop DMA (if needed).
 *      f) Sets RxState = READY and RxEventType = HAL_UART_RXEVENT_IDLE.
 *      g) Calls user callback: HAL_UARTEx_RxEventCallback(huart, nb_rx_data).
 *
 *    - Scenario B: Buffer reaches full capacity before bus goes idle (Normal DMA mode).
 *      a) DMA completes transfer (NDTR reaches 0), asserts TCIF (Transfer Complete) flag.
 *      b) DMA TCIF triggers HAL_UART_RxCpltCallback() via DMA interrupt.
 *      c) The DMA channel stops automatically (Normal mode).
 *      d) RxEventType = HAL_UART_RXEVENT_TC (Transfer Complete).
 *      e) HAL_UART_IRQHandler() disables IDLEIE to prevent further interrupts.
 *      f) User callback: HAL_UART_RxCpltCallback(huart).
 *
 *    - Scenario C: Circular DMA mode (continuous streaming).
 *      a) DMA wraps around and never stops (NDTR re-loads to RxXferSize).
 *      b) IDLE interrupt can still fire when bus goes idle.
 *      c) Callback is triggered regardless of DMA state:
 *         HAL_UARTEx_RxEventCallback(huart, nb_rx_data)
 *      d) DMA continues running, enabling "always listening" use cases.
 *
 * 5. Interrupt sources in HAL_UART_IRQHandler() (priority order):
 *    - Error Flags: ORE, FE, NE, PE -> UART_HandleRxError() -> ErrorCallback.
 *    - IDLE (if IDLEIE enabled) -> UART_EndRxTransfer() -> RxEventCallback(nb_rx_data).
 *    - RXNE/RXFT (Not used when DMAR is enabled) -> ignored.
 */
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
