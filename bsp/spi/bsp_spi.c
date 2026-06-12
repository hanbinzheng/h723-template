#include "bsp_spi.h"
#include <assert.h>

#define SPI_TIMEOUT_MS 100

struct spi_inst {
	SPI_HandleTypeDef *handle;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;

	/* enum spi_work_mode work_mode; */
};

static struct spi_inst spi_inst[SPI_INST_MAX_NUM];
static uint8_t idx = 0;

struct spi_inst *spi_register(const struct spi_config *config)
{
	assert(config != NULL && config->handle != NULL && idx < SPI_INST_MAX_NUM);

	/* check repetition */
	for (uint8_t i = 0; i < idx; i++) {
		if (spi_inst[i].handle == config->handle) {
			/* in our case, 2 different handler */
			return (spi_inst + i);
		}
	}

	struct spi_inst *inst = (spi_inst + idx);
	inst->handle = config->handle;
	inst->cs_pin = config->cs_pin;
	inst->cs_port = config->cs_port;
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
	idx++;

	return inst;
}

/* reserved
void spi_select(struct spi_inst *inst)
{
	assert(inst != NULL);
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
}

void spi_deselect(struct spi_inst *inst)
{
	assert(inst != NULL);
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
}
*/

HAL_StatusTypeDef spi_transmit(struct spi_inst *inst, uint8_t *buff, uint16_t len)
{
	assert(inst != NULL && buff != NULL && inst->handle != NULL);

	/* check stm32h7xx_hal_gpio.h/c for GPIO_TypeDef and HAL_GPIO_WritePin() */
	if (inst->cs_port == NULL || inst->cs_pin == 0) { /* GPIO_PIN_0 is (uint16_t)1 */
		return HAL_SPI_Transmit(inst->handle, buff, len, SPI_TIMEOUT_MS);
	}

	HAL_StatusTypeDef ret = HAL_ERROR; /* returen value */
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(inst->handle, buff, len, SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
	return ret;

	/* reserved modes: need manually select CS but not manually deselect CS */
	/* HAL_SPI_TransmitReceive_IT(inst->handle, tx_buff, rx_buff, len); */
	/* HAL_SPI_TransmitReceive_DMA(inst->handle, tx_buff, rx_buff, len); */
}

HAL_StatusTypeDef spi_receive(struct spi_inst *inst, uint8_t *buff, uint16_t len)
{
	assert(inst != NULL && buff != NULL && inst->handle != NULL);

	/* check stm32h7xx_hal_gpio.h/c for GPIO_TypeDef and HAL_GPIO_WritePin() */
	if (inst->cs_port == NULL || inst->cs_pin == 0) { /* GPIO_PIN_0 is (uint16_t)1 */
		return HAL_SPI_Receive(inst->handle, buff, len, SPI_TIMEOUT_MS);
	}

	HAL_StatusTypeDef ret = HAL_ERROR; /* returen value */
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
	ret = HAL_SPI_Receive(inst->handle, buff, len, SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
	return ret;

	/* reserved modes: need manually select CS but not manually deselect CS */
	/* HAL_SPI_TransmitReceive_IT(inst->handle, tx_buff, rx_buff, len); */
	/* HAL_SPI_TransmitReceive_DMA(inst->handle, tx_buff, rx_buff, len); */
}

HAL_StatusTypeDef spi_transceive(struct spi_inst *inst, uint8_t *tx_buff, uint8_t *rx_buff,
				 uint16_t len)
{
	assert(inst != NULL && tx_buff != NULL && rx_buff != NULL && inst->handle != NULL);

	/* check stm32h7xx_hal_gpio.h/c for GPIO_TypeDef and HAL_GPIO_WritePin() */
	if (inst->cs_port == NULL || inst->cs_pin == 0) { /* GPIO_PIN_0 is (uint16_t)1 */
		return HAL_SPI_TransmitReceive(inst->handle, tx_buff, rx_buff, len, SPI_TIMEOUT_MS);
	}

	HAL_StatusTypeDef ret = HAL_ERROR; /* returen value */
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive(inst->handle, tx_buff, rx_buff, len, SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
	return ret;

	/* reserved modes: need manually select CS but not manually deselect CS */
	/* HAL_SPI_TransmitReceive_IT(inst->handle, tx_buff, rx_buff, len); */
	/* HAL_SPI_TransmitReceive_DMA(inst->handle, tx_buff, rx_buff, len); */
}

/* reserved callback functions */
/* void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) */
/* void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) */