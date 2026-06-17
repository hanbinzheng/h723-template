#ifndef BUZZER_H_
#define BUZZER_H_

#include "bsp_tim.h"

enum song {
	ODE_TO_JOY = 0,
	HA_JI_MI,
};

/**
 * @brief register spi_inst for buzzer control
 *
 * @param tim pointer to a tim instance
 */
void buzzer_register(const struct tim_inst *tim);

/**
 * @brief control buzzer
 *
 * the buzzer will be on with frequency, duration and volumn specified
 *
 * @note assumption: buzzer is initially off
 *
 * @param freq
 * @param duration duration, unit in ms
 * @param volumn the volumn of buzzer, [0, 1]
 */
void buzzer_ctrl(uint16_t freq, uint16_t duration, float volume);

/**
 * @brief play songs
 *
 * @param name the name of song
 */
void buzzer_play_song(enum song song);

#endif /* BUZZER_H_ */