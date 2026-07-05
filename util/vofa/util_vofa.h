#ifndef UTIL_VOFA_H_
#define UTIL_VOFA_H_

#include "SEGGER_RTT.h"
#include <stdint.h>

/* RTT channel 1 */
#define VOFA_RTT_CHANNEL (1)

static const uint8_t _vofa_float_tail[4] = {0x00, 0x00, 0x80, 0x7F}; /* NAN */

enum vofa_state {
	VOFA_FAILURE = -1,
	VOFA_SUCCESS = 0,
};

/**
 * @brief Send arbitrary float parameters to VOFA+ with compile-time size resolution
 *
 * This macro leverages C99 compound literals to initialize an array of
 * floats at compile-time. It automatically computes the argument count
 * and writes the raw binary stream followed by the JustFloat tail to
 * the target RTT channel.
 *
 * @param ... List of float or numerical variables to plot on VOFA+
 */
#define vofa_send(...)                                                                             \
	do {                                                                                       \
		float _data[] = {__VA_ARGS__};                                                     \
		uint8_t _num = sizeof(_data) / sizeof(float);                                      \
		SEGGER_RTT_Write(VOFA_RTT_CHANNEL, _data, _num * sizeof(float));                   \
		SEGGER_RTT_Write(VOFA_RTT_CHANNEL, _vofa_float_tail, 4);                           \
	} while (0)

/**
 * @brief Initialize and register the VOFA+ telemetry up-buffer
 *
 * Binds the static RAM up-buffer to Channel 1 using the non-blocking SKIP mode
 * to guarantee control loop execution determinism.
 *
 * @return vofa_state VOFA_SUCCESS if setup succeeded, VOFA_FAILURE otherwise
 */
enum vofa_state vofa_init(void);

#endif /* UTIL_VOFA_H_ */
