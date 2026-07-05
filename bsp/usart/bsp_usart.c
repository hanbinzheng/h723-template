#include "bsp_usart.h"
#include <assert.h>

struct usart_inst {
	UART_HandleTypeDef *huart;
	enum usart_receive_mode rx_mode;
	enum usart_transmit_mode tx_mode;
	usart_rx_callback callback;

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

	return NULL; /* fail to find */
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	struct usart_inst *inst = get_usart_inst(huart);

	if (inst->callback != NULL && inst->rx_mode == USART_RECEIVE_IT) {
		inst->callback(inst->buff1, inst->len);
	}
}

/* reserved
void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart)
{
	return;
}
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

static void usart_idle_dma_callback(struct usart_inst *inst, uint16_t len)
{
	UART_HandleTypeDef *huart = inst->huart;

	__HAL_DMA_DISABLE(huart->hdmarx);

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
	__HAL_DMA_ENABLE(huart->hdmarx);
}
