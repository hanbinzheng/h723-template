#ifndef UTIL_VOFA_H_
#define UTIL_VOFA_H_

#include "SEGGER_RTT.h"
#include <math.h>
#include <stdint.h>

/* RTT channel 1 */
#define VOFA_RTT_CHANNEL (1)
#define VOFA_DECIMAL_DIGITS (6)

enum vofa_state {
	VOFA_FAILURE = -1,
	VOFA_SUCCESS = 0,
};

__ALWAYS_INLINE
static char *_fill_decimal_part(char *p, uint32_t val, uint16_t digits)
{
	p += digits;
	char *end = p;
	while (digits--) {
		uint32_t next = val / 10;
		*(--p) = (char)('0' + (val - next * 10)); /* avoid % operation */
		val = next;
	}
	return end; /* return next pointer */
}

__ALWAYS_INLINE
static char *_fill_integer_part(char *p, uint32_t val)
{
	if (val == 0) {
		*p++ = '0';
		return p;
	}

	/* store in reverse order */
	char *start = p;
	while (val > 0) {
		uint32_t next = val / 10;
		*p++ = (char)('0' + (val - next * 10));
		val = next;
	}
	char *end = p;

	/* reverse */
	p--;
	while (start < p) {
		char tmp = *start;
		*start++ = *p;
		*p-- = tmp;
	}

	return end; /* next position */
}

/**
 * @brief Send arbitrary float variables to VOFA+ with compile-time size resolution
 *
 * This macro leverages C99 compound literals to initialize an array of
 * floats at compile-time. It automatically computes the argument count
 * and writes the raw binary stream followed by the JustFloat tail to
 * the target RTT channel.
 *
 * @note the printing frequency should not be larger than 800 float / s.
 *	Otherwise, it will trigger inaccuracy either in vofa or in RTT transmission.
 *
 * @param ... List of float or numerical variables to plot on VOFA+
 */
#define vofa_send(...)                                                                             \
	do {                                                                                       \
		float _data[] = {__VA_ARGS__};                                                     \
		int _num = sizeof(_data) / sizeof(float);                                          \
		char _buff[_num * 20 + 1];                                                         \
		char *_p = _buff;                                                                  \
                                                                                                   \
		for (int _i = 0; _i < _num; _i++) {                                                \
			float _val = _data[_i];                                                    \
			float _abs_val = fabsf(_val);                                              \
                                                                                                   \
			if (signbit(_val)) {                                                       \
				*_p++ = '-';                                                       \
			}                                                                          \
                                                                                                   \
			uint32_t _integ = (uint32_t)_abs_val;                                      \
			uint32_t _frac = (uint32_t)((_abs_val - (float)_integ) * 1000000.0f);      \
                                                                                                   \
			_p = _fill_integer_part(_p, _integ);                                       \
			*_p++ = '.';                                                               \
			_p = _fill_decimal_part(_p, _frac, VOFA_DECIMAL_DIGITS);                   \
                                                                                                   \
			if (_i < _num - 1) {                                                       \
				*_p++ = ',';                                                       \
			}                                                                          \
		}                                                                                  \
                                                                                                   \
		*_p++ = '\n';                                                                      \
		SEGGER_RTT_Write(VOFA_RTT_CHANNEL, _buff, (unsigned)(_p - _buff));                 \
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
