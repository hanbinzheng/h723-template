#ifndef BSP_GPIO_H_
#define BSP_GPIO_H_

#include "gpio.h"
#include <stdint.h>

#define GPIO_INST_MAX_NUM 10

struct gpio_inst;

/* exit mode, reserved
	enum gpio_exit_mode {
		GPIO_EXTI_MODE_RISING,
		GPIO_EXTI_MODE_FALLING,
		GPIO_EXTI_MODE_RISING_FALLING,
		GPIO_EXTI_MODE_NONE,
	};
*/

struct gpio_config {
	/* type and macros in stm32h7xx_hal_gpio.h */
	GPIO_TypeDef *port;	      /* GPIOA, GPIOB... */
	uint16_t pin;		      /* GPIO_PIN_0, GPIO_PIN_2... */
	GPIO_PinState pin_init_state; /* GPIO_PIN_SET, GPIO_PIN_RESET */

	/* reserved */
	/* void (*callback)(struct gpio_inst *); */
	/* enum gpio_exit_mode exit_mode; */
};

/**
 * @brief register a GPIO instance
 *
 * @note this should be called after the initialization of all peripherals.
 * MX_GPIO_Init() in main function will initialize the GPIO clock,
 * and MX_XXX_Init() in main function will call HAL_GPIO_Init() finally.
 *
 * @param config GPIO instance configuration struct
 * @return pointer to the GPIO instance
 *
 */
struct gpio_inst *gpio_register(const struct gpio_config *config);

/**
 * @brief set GPIO pin
 *
 * @param inst pointer to the GPIO instance
 */
void gpio_set(struct gpio_inst *inst);

/**
 * @brief reset GPIO pin
 *
 * @param inst pointer to the GPIO instance
 */
void gpio_reset(struct gpio_inst *inst);

/**
 * @brief toggle GPIO pin
 *
 * @param inst pointer to the GPIO instance
 */
void gpio_toggle(struct gpio_inst *inst);

/**
 * @brief read GPIO pin state
 *
 * @param inst pointer to the GPIO instance
 * @return the GPIO pin state
 */
GPIO_PinState gpio_read(struct gpio_inst *inst);

#endif /* BSP_GPIO_H_ */