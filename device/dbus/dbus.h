#ifndef DBUS_H_
#define DBUS_H_

#include <stdint.h>

#define DBUS_FRAME_LENGTH 18u /* 18 byte */

enum dbus_sw {
	DBUS_SW_UP = 1,
	DBUS_SW_DOWN,
	DBUS_SW_MID,
};

struct dbus_data {
	/* left and right stick, from -1.0 to 1.0 */
	float ls_x;
	float ls_y;
	float rs_x;
	float rs_y;

	/* left and right switch, UP/DOWN/MID */
	enum dbus_sw sw_l;
	enum dbus_sw sw_r;

	/* mouse info, from -1.0 to 1.0 */
	float mouse_x;
	float mouse_y;
	float mouse_z;

	/* mouse key, release: 0, pressed: 1 */
	uint8_t mouse_l;
	uint8_t mouse_r;

	/* keyboard, release: 0, pressed: 1 */
	union {
		uint16_t key_code;
		struct {
			uint16_t w : 1; /* bit00, LSB */
			uint16_t s : 1;
			uint16_t a : 1;
			uint16_t d : 1;
			uint16_t shift : 1;
			uint16_t ctrl : 1;
			uint16_t q : 1;
			uint16_t e : 1;
			uint16_t r : 1;
			uint16_t f : 1;
			uint16_t g : 1;
			uint16_t z : 1;
			uint16_t x : 1;
			uint16_t c : 1;
			uint16_t v : 1;
			uint16_t b : 1; /* bit15, MSB */
		};
	} keyboard;

	/* from -1.0 to 1.0 */
	float wheel;
};

/**
 * @brief get current dubs information
 *
 * @return pointer to current dbus info
 */
struct dbus_data *dbus_get_data(void);

/**
 * @brief update the dbus information
 *
 * @note this function is only for data interpretation and update of a single frame
 *	user should handle the packet coalescing manually.
 * @param buff raw reception buffer
 */
void dbus_update(uint8_t *buff);

#endif /* DBUS_H_ */