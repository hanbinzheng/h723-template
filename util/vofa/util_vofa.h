#ifndef UTIL_VOFA_H_
#define UTIL_VOFA_H_

#include "SEGGER_RTT.h"
#include <stdint.h>

#define VOFA_RTT_CHANNEL (1)

static const uint8_t _vofa_float_tail[4] = {0x00, 0x00, 0x80, 0x7F};

enum vofa_state {
	VOFA_FAILURE = -1,
	VOFA_SUCCESS = 0,
};

#define vofa_send(...)                                                                             \
	do {                                                                                       \
		float _data[] = {__VA_ARGS__};                                                     \
		uint8_t _num = sizeof(_data) / sizeof(float);                                      \
		SEGGER_RTT_Write(VOFA_RTT_CHANNEL, _data, _num * sizeof(float));                   \
		SEGGER_RTT_Write(VOFA_RTT_CHANNEL, _vofa_float_tail, 4);                           \
	} while (0)

enum vofa_state vofa_init(void);

#endif /* UTIL_VOFA_H_ */
