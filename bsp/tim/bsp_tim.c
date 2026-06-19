#include "bsp_tim.h"
#include <assert.h>

struct tim_inst {
	TIM_HandleTypeDef *htim;
	enum tim_mode mode;
	uint32_t clk_freq;
	uint32_t channel;
};

static struct tim_inst tim_inst[TIM_INST_MAX_NUM];
static uint8_t idx = 0;

struct tim_inst *tim_register(const struct tim_config *config)
{
	assert(config != NULL && config->htim != NULL && idx < TIM_INST_MAX_NUM);

	/* check repetition */
	for (uint8_t i = 0; i < idx; i++) {
		struct tim_inst *tmp = (tim_inst + i);
		if (tmp->htim == config->htim && tmp->channel == config->channel) {
			return tmp;
		}
	}

	struct tim_inst *inst = (tim_inst + idx);
	inst->htim = config->htim;
	inst->mode = config->mode;
	inst->clk_freq = config->clk_freq;
	inst->channel = config->channel;

	switch (inst->mode) {
	case TIM_INTERRUPT_MODE:
		if (HAL_TIM_Base_Start_IT(inst->htim) != HAL_OK) {
			return NULL;
		}
		break;
	case TIM_PWM_MODE:
		if (HAL_TIM_Base_Start(inst->htim) != HAL_OK) {
			return NULL;
		}
		break;
	};
	idx++;

	return inst;
}

__ITCM_FUNC
HAL_StatusTypeDef tim_pwm_start(const struct tim_inst *inst)
{
	assert(inst != NULL);
	return HAL_TIM_PWM_Start(inst->htim, inst->channel);
}

__ITCM_FUNC
HAL_StatusTypeDef tim_pwm_stop(const struct tim_inst *inst)
{
	assert(inst != NULL);
	return HAL_TIM_PWM_Stop(inst->htim, inst->channel);
}

/* check stm32h7xx_hal_tim.h for details */
__ITCM_FUNC
HAL_StatusTypeDef tim_set_pwm(struct tim_inst *inst, uint16_t pwm_freq, float pwm_duty)
{
	assert(inst != NULL && inst->htim != NULL);
	TIM_HandleTypeDef *htim = inst->htim;
	uint32_t pwm_freq_max = inst->clk_freq / (htim->Init.Prescaler + 1);
	if (inst->mode != TIM_PWM_MODE || pwm_freq > pwm_freq_max) {
		return HAL_ERROR;
	}

	/* change htim->Init.Period and the auto-loader register htim->Instance->ARR */
	uint16_t arr = (uint16_t)((float)pwm_freq_max / (float)pwm_freq - 1.0f);
	__HAL_TIM_SET_AUTORELOAD(htim, arr);

	/* change pwm pusle, the capture & compare register CCR */
	pwm_duty = pwm_duty > 1.0f ? 1.0f : pwm_duty;
	pwm_duty = pwm_duty < 0.0f ? 0.0f : pwm_duty;
	__HAL_TIM_SET_COMPARE(htim, inst->channel, (uint16_t)(arr * pwm_duty));

	return HAL_OK;
}