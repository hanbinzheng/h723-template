#ifndef BSP_SPI_H_
#define BSP_SPI_H_

#include "gpio.h"
#include "spi.h"
#include <stdint.h>

#define SPI_INST_MAX_NUM 2 /* only 2 in use */

/* only supports simple transmit and receive under block mode now */

struct spi_inst;

/* reserved: SPI transmit/receive mode enum
enum spi_work_mode {
	SPI_BLOCK_MODE = 0,
	SPI_IT_MODE,
	SPI_DMA_MODE,
};
*/

struct spi_config {
	SPI_HandleTypeDef *handle; /* hspi1, hspi2... */
	GPIO_TypeDef *cs_port;	   /* GPIOA, GPIOB... */
	uint16_t cs_pin;	   /* GPIO_PIN_1, GPIO_PIN_2... */

	/* enum spi_work_mode work_mode; */
};

/**
 * @brief register a SPI instance
 *
 * @note this should be called after the initialization of all peripherals.
 * specially, all MX_SPIX_Init() functions.
 * config->handle must be valid, and by default, the CS is not selected.
 *
 * @param config SPI instance configuration struct
 * @return pointer to the SPI instance
 *
 */
struct spi_inst *spi_register(const struct spi_config *config);

/**
 * @brief select the CS
 *
 * @param inst pointer to the SPI instance
 */
/* void spi_select(struct spi_inst *inst); */

/**
 * @brief release the CS
 *
 * @param inst pointer to the SPI instance
 */
/* void spi_deselect(struct spi_inst *inst); */

/**
 * @brief transmit in block mode
 *
 * blocking time: 100ms
 * CS will be asserted before transmission and deasserted after.
 * for invalid cs_port and cs_pin, CS operation is omitted
 *
 * @param inst pointer to the SPI instance
 * @param buff transmit buffer
 * @param len number of bytes to transmit in tx buffer
 * @return whether successful
 */
HAL_StatusTypeDef spi_transmit(struct spi_inst *inst, uint8_t *buff, uint16_t len);

/**
 * @brief receive data in blocking mode
 *
 * blocking timeout: 100ms
 * CS will be asserted before reception and deasserted after.
 * for invalid cs_port and cs_pin, CS operation is omitted.
 *
 * @param inst pointer to the SPI instance
 * @param buff receive buffer
 * @param len number of bytes to receive
 * @return whether successful
 */
HAL_StatusTypeDef spi_receive(struct spi_inst *inst, uint8_t *buff, uint16_t len);

/**
 * @brief transmit and receive simultaneously in blocking mode
 *
 * blocking timeout: 100ms
 * CS will be asserted before transaction and deasserted after.
 *
 * @param inst pointer to the SPI instance
 * @param tx_buff tx buffer (data to send)
 * @param rx_buff rx buffer (received data)
 * @param len number of bytes to transmit/receive
 * @return whether successful
 */
HAL_StatusTypeDef spi_transceive(struct spi_inst *inst, uint8_t *tx_buff, uint8_t *rx_buff,
				 uint16_t len);

#endif /* BSP_SPI_H_ */