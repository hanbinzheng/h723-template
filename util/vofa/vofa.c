#include "vofa.h"

#define VOFA_BUFFER_SIZE (2048)

static uint8_t vofa_up_buffer[VOFA_BUFFER_SIZE];

enum vofa_state vofa_init(void)
{
	/* SEGGER_RTT_ConfigUpBuffer(): if successful return 0, otherwise return -1 */
	return SEGGER_RTT_ConfigUpBuffer(VOFA_RTT_CHANNEL, "vofa", vofa_up_buffer, VOFA_BUFFER_SIZE,
					 SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}
