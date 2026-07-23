#include "bsp_gpio.h"
#include <assert.h>

struct gpio_inst {
	GPIO_TypeDef *port;
	uint16_t pin;
	GPIO_PinState pin_init_state;

	/* void (*callback)(struct gpio_inst *); */
	/* enum gpio_exit_mode exit_mode; */
};

static struct gpio_inst gpio_inst[GPIO_INST_MAX_NUM];
static uint8_t idx = 0;

struct gpio_inst *gpio_register(const struct gpio_config *config)
{
	assert(config != NULL && idx < GPIO_INST_MAX_NUM);

	/* check repetition */
	for (uint8_t i = 0; i < idx; i++) {
		struct gpio_inst tmp = gpio_inst[i];
		if (tmp.port == config->port && tmp.pin == config->pin) {
			return (gpio_inst + i);
		}
	}

	struct gpio_inst *inst = (gpio_inst + idx);
	inst->port = config->port;
	inst->pin = config->pin;
	inst->pin_init_state = config->pin_init_state;
	HAL_GPIO_WritePin(inst->port, inst->pin, inst->pin_init_state);
	/* inst->callback = config->callback */
	/* inst->exit_mode = config->exit_mode */
	idx++;

	return inst;
}

void gpio_set(struct gpio_inst *inst)
{
	assert(inst != NULL);
	HAL_GPIO_WritePin(inst->port, inst->pin, GPIO_PIN_SET);
}

void gpio_reset(struct gpio_inst *inst)
{
	assert(inst != NULL);
	HAL_GPIO_WritePin(inst->port, inst->pin, GPIO_PIN_RESET);
}

void gpio_toggle(struct gpio_inst *inst)
{
	assert(inst != NULL);
	HAL_GPIO_TogglePin(inst->port, inst->pin);
}

GPIO_PinState gpio_read(struct gpio_inst *inst)
{
	assert(inst != NULL);
	return HAL_GPIO_ReadPin(inst->port, inst->pin);
}

/* reserved callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	struct gpio_inst *inst;
	for (size_t i = 0; i < idx; i++) {
		inst = gpio_inst + i;
		if (inst->pin == GPIO_Pin && inst->callback != NULL) {
			inst->callback(inst);
			return;
		}
	}
}
*/
