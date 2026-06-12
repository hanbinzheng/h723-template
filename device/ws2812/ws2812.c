#include "ws2812.h"
#include "bsp_spi.h"

#define WS2812_LOW_LEVEL 0xC0
#define WS2812_HIGH_LEVEL 0xF0

static struct spi_inst *ws2812_spi;

void ws2812_register(const struct spi_inst *spi)
{
	ws2812_spi = (struct spi_inst *)spi;
}

void ws2812_ctrl(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t tx_buff[24];
	uint8_t res = 0;
	for (int i = 0; i < 8; i++) {
		tx_buff[7 - i] = (((g >> i) & 0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL) >> 1;
		tx_buff[15 - i] = (((r >> i) & 0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL) >> 1;
		tx_buff[23 - i] = (((b >> i) & 0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL) >> 1;
	}
	spi_transmit(ws2812_spi, tx_buff, 24);
	for (int i = 0; i < 100; i++) {
		spi_transmit(ws2812_spi, &res, 1);
	}
}
