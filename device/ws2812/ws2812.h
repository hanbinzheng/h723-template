#ifndef WS2812_H_
#define WS2812_H_

#include "bsp_spi.h"

/**
 * @brief register spi_inst for WS2812 control
 *
 * @param spi pointer to a SPI instance
 */
void ws2812_register(const struct spi_inst *spi);

/**
 * @brief control WS2812 RGB LED color via SPI
 *
 * this function uses SPI to send GRB data to WS2812 LED.
 *
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void ws2812_ctrl(uint8_t r, uint8_t g, uint8_t b);

#endif /* WS2812_H_ */
