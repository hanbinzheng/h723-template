#ifndef LOG_H_
#define LOG_H_

#define LOG_RTT_CHANNEL (0)

#include "SEGGER_RTT.h"

#define LOG_INFO(format, ...)                                                                      \
	SEGGER_RTT_printf(LOG_RTT_CHANNEL, "[INFO] (%s:%d): " format "\r\n", __FILE_NAME__,        \
			  __LINE__, ##__VA_ARGS__)

#define LOG_DEBUG(format, ...)                                                                     \
	SEGGER_RTT_printf(LOG_RTT_CHANNEL,                                                         \
			  RTT_CTRL_TEXT_BRIGHT_YELLOW "[DEBUG][%s:%d]: " format RTT_CTRL_RESET     \
						      "\r\n",                                      \
			  __FILE_NAME__, __LINE__, ##__VA_ARGS__)

#define LOG_ERROR(format, ...)                                                                     \
	SEGGER_RTT_printf(LOG_RTT_CHANNEL,                                                         \
			  RTT_CTRL_TEXT_BRIGHT_RED "[ERROR][%s:%d]: " format RTT_CTRL_RESET        \
						   "\r\n",                                         \
			  __FILE_NAME__, __LINE__, ##__VA_ARGS__)

#endif /* LOG_H_ */