#ifndef SBUS_H_
#define SBUS_H_

#include <stdint.h>

#define SBUS_FRAME_LENGTH 25u /* 25 bytes */

/* 192 ~ 992 ~ 1792 */
#define SBUS_CHANNEL_MIN ((uint16_t)192)
#define SBUS_CHANNEL_MAX ((uint16_t)1792)
#define SBUS_CHANNEL_OFFSET ((uint16_t)992)
#define SBUS_CHANNEL_RANGE 800.0f

enum sbus_sw {
	SBUS_SW_UP = SBUS_CHANNEL_MIN,
	SBUS_SW_MID = SBUS_CHANNEL_OFFSET,
	SBUS_SW_DOWN = SBUS_CHANNEL_MAX,
};

enum sbus_safe {
	SBUS_UNSAFE = 0,
	SBUS_SAFE,
};

struct sbus_data {
	/* left and right stick, from -1.0 to 1.0 */
	float ls_x;
	float ls_y;
	float rs_x;
	float rs_y;

	/* 4 switchs */
	enum sbus_sw sw1; /* SWA-5, UP-MID-DOWN */
	enum sbus_sw sw2; /* SWB-6, UP-DOWN */
	enum sbus_sw sw3; /* SWC-7, UP-DOWN */
	enum sbus_sw sw4; /* SWD-8, UP-MID-DOWN */

	/* left and right wheels, from -1.0 to 1.0 */
	float wh1; /* VRA, left wheel */
	float wh2; /* VRB, right wheel */

	enum sbus_safe safe;
};

/**
 * @brief get current sbus data information
 *
 * @return pointer to current sbus data info
 */
const struct sbus_data *sbus_get_data(void);

/**
 * @brief update the sbus data information
 *
 * @note this function is only for data interpretation and update of a single frame
 *	user should handle the packet coalescing manually.
 * @param buff raw reception buffer
 */
void sbus_update(uint8_t *buff);

#endif /* SBUS_H_ */
