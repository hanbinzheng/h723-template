/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  	Wang Hongxi
 * @author  	modified by NeoZng
 * @author  	modified by Zheng Hanbin
 * @date    	2026/06/15
 * @brief
 ******************************************************************************
 */

#ifndef BSP_DWT_H_
#define BSP_DWT_H_

#include "main.h"
#include <stdint.h>

/**
 * @brief used to calculate code segment execution time in seconds
 *
 * @param dt: float
 * @param code: code segmentation to execute
 */
#define TIME_ELAPSE(dt, code)                                                                      \
	do {                                                                                       \
		float tstart = dwt_get_timeline_s();                                               \
		code;                                                                              \
		dt = dwt_get_timeline_s() - tstart;                                                \
	} while (0)

/**
 * @brief initialize DWT, input parameter is the CPU frequency, unit MHz
 *
 * @param cpu_freq_mhz C board: 168MHz, A board: 180MHz
 */
void dwt_init(uint32_t cpu_freq_mhz);

/**
 * @brief get the time interval between two calls in seconds
 *
 * @attention assume that within one overflow
 * @param cnt_last timestamp of the last call
 * @return float time interval, unit is seconds
 */
float dwt_get_delta_t(uint32_t *cnt_last);

/**
 * @brief get the time interval between two calls in seconds, high precision
 *
 * @attention assume that within one overflow
 * @param cnt_last timestamp of the last call
 * @return double time interval, unit is seconds
 */
double dwt_get_delta_t_64(uint32_t *cnt_last);

/**
 * @brief DWT update timeline function, will be called by the three timeline functions
 * @attention if timeline functions are not called for a long time, this function needs
 * to be called manually to update the timeline, otherwise CYCCNT overflow will cause
 * inaccurate timing and timeline
 */
void dwt_systime_update(void);

/**
 * @brief get current time in s/ms/us (since initialization)
 *
 * @return float timeline / uint16_t timeline
 */
float dwt_get_timeline_s(void);
float dwt_get_timeline_ms(void);
uint64_t dwt_get_timeline_us(void);

/**
 * @brief DWT delay function in s/ms
 *
 * @attention not affected by whether interrupts are enabled
 * @note: error: s(0.00045%), ms(0.39%)
 *
 * @param delay delay time in s/ms
 */
void dwt_delay_s(float delay_s);
void dwt_delay_ms(uint32_t delay_ms);

/**
 * @brief DWT delay function in us
 *
 * this function must be implemented inline to reduce error
 * @attention the delay_us should be within 1000 to guarantee precision
 * @note error: 10~20%, and for high precision case, please use tim_delay
 *
 * @param delay delay time in us
 */
__attribute__((always_inline)) static inline void dwt_delay_us(uint32_t delay_us,
							       uint32_t cpu_freq_mhz)
{
	uint32_t start_cnt = DWT->CYCCNT;
	uint32_t delay_cnt = delay_us * cpu_freq_mhz; /* cpu_freq_mhz = ticks per us */

	while ((uint32_t)(DWT->CYCCNT - start_cnt) < delay_cnt)
		; /* unsigned int can automatically handle a overflow */
}

#endif /* BSP_DWT_H_ */