#ifndef BSP_TIM_
#define BSP_TIM_

#include "tim.h"

#define TIM_INST_MAX_NUM 5
#define TIM_CHANNEL_NONE ((uint32_t)-1)

struct tim_inst;
typedef void (*tim_callback)(void);

enum tim_mode {
	TIM_INTERRUPT_MODE = 0,
	TIM_PWM_MODE,
};

struct tim_config {
	TIM_HandleTypeDef *htim;
	uint32_t channel; /* TIM_CHANNEL_1, TIM_CHANNEL_2... */
	enum tim_mode mode;
	uint32_t clk_freq;
	tim_callback callback;
};

/**
 * @brief register a TIM instance
 *
 * @note this should be called after the initialization of all peripherals.
 * specially, all MX_TIMX_Init() functions.
 *
 * @attention for PWM, you should manually open it by tim_pwm_start()
 *
 * @param config TIM instance configuration struct
 * @return pointer to the TIM instance
 *
 */
struct tim_inst *tim_register(const struct tim_config *config);

/**
 * @brief control on and off of PWM
 *
 * @param inst pointer to tim_inst
 * @return whether this operation is successful
 */
HAL_StatusTypeDef tim_pwm_start(const struct tim_inst *inst);
HAL_StatusTypeDef tim_pwm_stop(const struct tim_inst *inst);

/**
 * @brief change the setting of pwm in running time
 *
 * @note users should make sure you
 *
 * @param inst pointer to tim_inst
 * @param pwm_freq target pwm frequency
 * @param pwm_duty target pwm duty, in range [0, 1]
 * @return whether the modification is successful
 */
HAL_StatusTypeDef tim_set_pwm(struct tim_inst *inst, uint16_t pwm_freq, float pwm_duty);

#endif /* BSP_TIM_ */