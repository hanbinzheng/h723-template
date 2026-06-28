#include "sbus.h"
#include <assert.h>
#include <stddef.h> /* for NULL */

#define SBUS_SOF 0x0F
#define SBUS_EOF 0x00

/* check Futaba S-Bus.pdf */

#pragma pack(push, 1)
struct sbus_wire {
	/* start of frame, 1 byte, 0x0F */
	uint8_t sof; /* deviates from the manual */

	/* 16 channels, 11 bit each, 11 x 16 = 176 bit, 22 bytes */
	uint16_t ch0 : 11;
	uint16_t ch1 : 11;
	uint16_t ch2 : 11;
	uint16_t ch3 : 11;
	uint16_t ch4 : 11;
	uint16_t ch5 : 11;
	uint16_t ch6 : 11;
	uint16_t ch7 : 11;
	uint16_t ch8 : 11;
	uint16_t ch9 : 11;

	/* channel 10 ~ 15: not in use */
	uint16_t ch10 : 11;
	uint16_t ch11 : 11;
	uint16_t ch12 : 11;
	uint16_t ch13 : 11;
	uint16_t ch14 : 11;
	uint16_t ch15 : 11;

	/* flags: digital channels + frame lost + fail-safe activated */
	uint8_t ch16 : 1;	  /* bit7, digital channel 1 */
	uint8_t ch17 : 1;	  /* bit6, digital channel 2 */
	uint8_t frame_lost : 1;	  /* bit5: frame lost flag */
	uint8_t failsafe_act : 1; /* bit4: failure safe activated */
	uint8_t unused : 4;

	uint8_t eof; /* end of frame, 1 byte, 0x00 */
};
#pragma pack(pop)

struct sbus_host {
	uint8_t sof;

	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	int16_t ch6;
	int16_t ch7;
	int16_t ch8;

	int16_t ch9;
	int16_t ch10;
	int16_t ch11;
	int16_t ch12;
	int16_t ch13;
	int16_t ch14;
	int16_t ch15;

	uint8_t ch16;
	uint8_t ch17;
	uint8_t frame_lost;
	uint8_t fail_safe_act;

	uint8_t eof;
};

static struct sbus_host sbus_host;
static struct sbus_data sbus_data;

const struct sbus_data *sbus_get_data(void)
{
	return &sbus_data;
}

void sbus_update(uint8_t *buff)
{
	assert(buff != NULL);
	if (buff[0] != SBUS_SOF || buff[SBUS_FRAME_LENGTH - 1] != SBUS_EOF) {
		return;
	}

	struct sbus_wire *sbus_wire = (struct sbus_wire *)buff;

	/* interpret data to sbus_host */
	sbus_host.sof = sbus_wire->sof;
	sbus_host.ch0 = (int16_t)sbus_wire->ch0 - SBUS_CHANNEL_OFFSET;
	sbus_host.ch1 = (int16_t)sbus_wire->ch1 - SBUS_CHANNEL_OFFSET;
	sbus_host.ch2 = (int16_t)sbus_wire->ch2 - SBUS_CHANNEL_OFFSET;
	sbus_host.ch3 = (int16_t)sbus_wire->ch3 - SBUS_CHANNEL_OFFSET;
	sbus_host.ch4 = sbus_wire->ch4; /* ch4 ~ ch5: used as switch, no need to normalize */
	sbus_host.ch5 = sbus_wire->ch5;
	sbus_host.ch6 = sbus_wire->ch6;
	sbus_host.ch7 = sbus_wire->ch7;
	sbus_host.ch8 = (int16_t)sbus_wire->ch8 - SBUS_CHANNEL_OFFSET;
	sbus_host.ch9 = (int16_t)sbus_wire->ch9 - SBUS_CHANNEL_OFFSET;
	sbus_host.ch16 = sbus_wire->ch16;
	sbus_host.ch17 = sbus_wire->ch17;
	sbus_host.frame_lost = sbus_wire->frame_lost;
	sbus_host.fail_safe_act = sbus_wire->failsafe_act;
	sbus_host.eof = sbus_wire->eof;

	/* from sbus_host to sbus_data */
	sbus_data.ls_x = (float)sbus_host.ch2 / SBUS_CHANNEL_RANGE;
	sbus_data.ls_y = (float)sbus_host.ch3 / SBUS_CHANNEL_RANGE;
	sbus_data.rs_x = (float)sbus_host.ch1 / SBUS_CHANNEL_RANGE;
	sbus_data.rs_y = (float)sbus_host.ch0 / SBUS_CHANNEL_RANGE;
	sbus_data.sw1 = sbus_host.ch4;
	sbus_data.sw2 = sbus_host.ch5;
	sbus_data.sw3 = sbus_host.ch6;
	sbus_data.sw4 = sbus_host.ch7;
	sbus_data.wh1 = (float)sbus_host.ch8 / SBUS_CHANNEL_RANGE;
	sbus_data.wh2 = (float)sbus_host.ch9 / SBUS_CHANNEL_RANGE;
	sbus_data.safe = sbus_host.fail_safe_act || sbus_host.frame_lost;
}
