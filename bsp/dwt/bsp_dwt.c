/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  	Wang Hongxi
 * @author  	modified by Neo with annotation
 * @author  	modified by Zheng Hanbin
 * @date    	2026/06/15
 * @brief
 ******************************************************************************
 */

#include "bsp_dwt.h"

struct dwt_time {
	uint32_t s;
	uint16_t ms;
	uint16_t us;
};

static struct dwt_time systime;
static uint32_t cyccnt_round_count;
static uint32_t cyccnt_last;
static uint64_t cyccnt64;
static uint32_t ticks_per_s, ticks_per_ms, ticks_per_us; /* avoid division in MCU */

/**
 * @brief private function used to check if DWT->CYCCNT register overflows,
 * and update the cyccnt_round_count
 *
 * @attention this function assumes the time interval between two calls does not
 * exceed one overflow
 *
 * @todo better solution: set up a separate task for DWT time update?
 * However, the original intention of using dwt is to ensure timing is not
 * affected by factors such as interrupts/tasks, so this implementation still
 * has its significance.
 *
 */
static void dwt_cnt_update(void)
{
	static volatile uint8_t bit_locker = 0;
	if (!bit_locker) {
		bit_locker = 1;
		volatile uint32_t cnt_now = DWT->CYCCNT;
		if (cnt_now < cyccnt_last)
			cyccnt_round_count++;

		cyccnt_last = DWT->CYCCNT;
		bit_locker = 0;
	}
}

/* see core_cm7.h for details */
void dwt_init(uint32_t cpu_freq_mhz)
{
	/* enable the access of DWT peripheral */
	/* check the struct: CoreDebug_Type, CoreDebug = 0xE000EDF0UL */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* xxx |= (1UL << 24U ) */

	/* clear the DWT CYCCNT register */
	/* check the struct DWT_Type, 0xE0001000UL->CYCCNT */
	DWT->CYCCNT = (uint32_t)0u;

	/* enable Cortex-M DWT CYCCNT register */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* xxx |= 0x1UL */

	ticks_per_s = cpu_freq_mhz * 1000000;
	ticks_per_ms = cpu_freq_mhz * 1000;
	ticks_per_us = cpu_freq_mhz;
	cyccnt_round_count = 0;

	dwt_cnt_update();
}

float dwt_get_delta_t(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	float dt = (cnt_now - *cnt_last) / (float)ticks_per_s;
	*cnt_last = cnt_now;

	dwt_cnt_update();

	return dt;
}

double dwt_get_delta_t_64(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	double dt = (cnt_now - *cnt_last) / (double)ticks_per_s;
	*cnt_last = cnt_now;

	dwt_cnt_update();

	return dt;
}

void dwt_systime_update(void)
{
	dwt_cnt_update();

	volatile uint32_t cnt_now = DWT->CYCCNT;
	uint64_t cnt_temp1, cnt_temp2, cnt_temp3;

	/* get total counts, cyccnt_round_count * UINT32_MAX: error of missing 1 */
	cyccnt64 = ((uint64_t)cyccnt_round_count << 32) + (uint64_t)cnt_now;

	cnt_temp1 = cyccnt64 / ticks_per_s;
	systime.s = cnt_temp1;

	cnt_temp2 = cyccnt64 - cnt_temp1 * ticks_per_s;
	systime.ms = cnt_temp2 / ticks_per_ms;

	cnt_temp3 = cnt_temp2 - systime.ms * ticks_per_ms;
	systime.us = cnt_temp3 / ticks_per_us;
}

float dwt_get_timeline_s(void)
{
	dwt_systime_update();

	float timeline = systime.s + systime.ms * 0.001f + systime.us * 0.000001f;

	return timeline;
}

float dwt_get_timeline_ms(void)
{
	dwt_systime_update();

	float timeline = systime.s * 1000 + systime.ms + systime.us * 0.001f;

	return timeline;
}

uint64_t dwt_get_timeline_us(void)
{
	dwt_systime_update();

	uint64_t timeline = systime.s * 1000000 + systime.ms * 1000 + systime.us;

	return timeline;
}

/* use get_timeline_us(), the error is negligible */
void dwt_delay_ms(uint32_t delay_ms)
{
	uint64_t start_us = dwt_get_timeline_us();
	uint64_t delay_us = delay_ms * 1000;

	while ((dwt_get_timeline_us() - start_us) < delay_us)
		;
}

/* use get_timeline_us(), the error is negligible */
void dwt_delay_s(float delay_s)
{
	uint64_t start_us = dwt_get_timeline_us();
	uint64_t delay_us = (uint64_t)(delay_s * 1000000);

	while ((dwt_get_timeline_us() - start_us) < delay_us)
		;
}
