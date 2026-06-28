#include "dbus.h"
#include <assert.h>
#include <stddef.h> /* for NULL */

/* check DT7_DR16_User_Manual.pdf for details */

/* 364 ~ 1024 ~ 1684 */
#define DBUS_CHANNEL_MIN ((uint16_t)364)
#define DBUS_CHANNEL_MAX ((uint16_t)1684)
#define DBUS_CHANNEL_OFFSET ((uint16_t)1024)
#define DBUS_CHANNEL_RANGE 660.0f

#define DBUS_MOUSE_VALUE_RANGE 32767.0f

#pragma pack(push, 1)
struct dbus_wire {
	uint64_t ch0 : 11;
	uint64_t ch1 : 11;
	uint64_t ch2 : 11;
	uint64_t ch3 : 11;
	uint64_t s1 : 2;
	uint64_t s2 : 2;
	uint64_t mouse_x : 16;

	uint16_t mouse_y;
	uint16_t mouse_z;
	uint8_t mouse_l;
	uint8_t mouse_r;

	uint16_t keyboard;

	/* not documented in the manual, in reserved section */
	uint16_t wheel : 11;
	uint16_t unused : 5;
};
#pragma pack(pop)

struct dbus_host {
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;

	enum dbus_sw s1;
	enum dbus_sw s2;

	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;
	uint8_t mouse_r;

	uint16_t keyboard;
	int16_t wheel;
};

static struct dbus_host dbus_host;
static struct dbus_data dbus_data;

const struct dbus_data *dbus_get_data(void)
{
	return &dbus_data;
}

void dbus_update(uint8_t *buff)
{
	assert(buff != NULL);
	struct dbus_wire *dbus_wire = (struct dbus_wire *)buff;

	/* get the host data */
	dbus_host.ch0 = dbus_wire->ch0 - DBUS_CHANNEL_OFFSET;
	dbus_host.ch1 = dbus_wire->ch1 - DBUS_CHANNEL_OFFSET;
	dbus_host.ch2 = dbus_wire->ch2 - DBUS_CHANNEL_OFFSET;
	dbus_host.ch3 = dbus_wire->ch3 - DBUS_CHANNEL_OFFSET;

	dbus_host.s1 = dbus_wire->s1;
	dbus_host.s2 = dbus_wire->s2;

	dbus_host.mouse_x = (int16_t)dbus_wire->mouse_x;
	dbus_host.mouse_y = (int16_t)dbus_wire->mouse_y;
	dbus_host.mouse_z = (int16_t)dbus_wire->mouse_z;
	dbus_host.mouse_l = dbus_wire->mouse_l;
	dbus_host.mouse_r = dbus_wire->mouse_r;

	dbus_host.keyboard = dbus_wire->keyboard;

	dbus_host.wheel = dbus_wire->wheel - DBUS_CHANNEL_OFFSET;

	/* updata the dbus_data */
	dbus_data.ls_x = (float)dbus_host.ch3 / DBUS_CHANNEL_RANGE;
	dbus_data.ls_y = (float)dbus_host.ch2 / DBUS_CHANNEL_RANGE;
	dbus_data.rs_x = (float)dbus_host.ch1 / DBUS_CHANNEL_RANGE;
	dbus_data.rs_y = (float)dbus_host.ch0 / DBUS_CHANNEL_RANGE;

	dbus_data.sw_l = dbus_host.s2; /* hardware deviates from the manual */
	dbus_data.sw_r = dbus_host.s1;

	dbus_data.mouse_x = (float)dbus_host.mouse_x / DBUS_MOUSE_VALUE_RANGE;
	dbus_data.mouse_y = (float)dbus_host.mouse_y / DBUS_MOUSE_VALUE_RANGE;
	dbus_data.mouse_z = (float)dbus_host.mouse_z / DBUS_MOUSE_VALUE_RANGE;
	dbus_data.mouse_l = dbus_host.mouse_l;
	dbus_data.mouse_r = dbus_host.mouse_r;

	dbus_data.keyboard.key_code = dbus_host.keyboard;

	dbus_data.wheel = (float)dbus_host.wheel / DBUS_CHANNEL_RANGE;
}