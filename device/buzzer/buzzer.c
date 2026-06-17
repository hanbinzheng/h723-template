#include "buzzer.h"
#include "bsp_dwt.h"
#include <assert.h>

static struct tim_inst *buzzer_tim = NULL;

/* definition of frequencies */
#define NOTE_REST 0

/* C4 ~ G4 */
#define NOTE_C4 262 /* 1 */
#define NOTE_D4 294 /* 2 */
#define NOTE_E4 330 /* 3 */
#define NOTE_F4 349 /* 4 */
#define NOTE_G4 392 /* 5 */
#define NOTE_A4 440 /* 6 */
#define NOTE_B4 494 /* 7 */

/* C5 ~ B5 */
#define NOTE_C5 523 /* ·1 */
#define NOTE_D5 587 /* ·2 */
#define NOTE_E5 659 /* ·3 */
#define NOTE_F5 698 /* ·4 */
#define NOTE_G5 784 /* ·5 */
#define NOTE_A5 880 /* ·6 */
#define NOTE_B5 988 /* ·7 */

/* durations */
#define EIGHTH_NOTE 125	 /* 1/8, 125 ms */
#define QUARTER_NOTE 250 /* 1/4, 250 ms */
#define HALF_NOTE 500	 /* 1/2, 500 ms */
#define PHRASE_PAUSE 180 /* pause */

struct note {
	uint16_t freq;
	uint16_t duration;
	float volume;
};

void buzzer_register(const struct tim_inst *tim)
{
	assert(tim != NULL);
	buzzer_tim = (struct tim_inst *)tim;
}

void buzzer_ctrl(uint16_t freq, uint16_t duration, float volume)
{
	if (freq == NOTE_REST) {
		tim_pwm_stop(buzzer_tim);
		dwt_delay_ms(duration);
		return;
	}

	tim_set_pwm(buzzer_tim, freq, volume);
	tim_pwm_start(buzzer_tim);
	dwt_delay_ms(duration);
	tim_pwm_stop(buzzer_tim);
}

static const struct note ode_to_joy[] = {
    /* 3345 | 5432 | 1123 | 322 */
    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_F4, QUARTER_NOTE, 0.5f},
    {NOTE_G4, QUARTER_NOTE, 0.5f},

    {NOTE_G4, QUARTER_NOTE, 0.5f},
    {NOTE_F4, QUARTER_NOTE, 0.5f},
    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_D4, QUARTER_NOTE, 0.5f},

    {NOTE_C4, QUARTER_NOTE, 0.5f},
    {NOTE_C4, QUARTER_NOTE, 0.5f},
    {NOTE_D4, QUARTER_NOTE, 0.5f},
    {NOTE_E4, QUARTER_NOTE, 0.5f},

    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_D4, QUARTER_NOTE, 0.5f},
    {NOTE_D4, HALF_NOTE, 0.5f},

    /* 3345 | 5432 | 1123 | 211 */
    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_F4, QUARTER_NOTE, 0.5f},
    {NOTE_G4, QUARTER_NOTE, 0.5f},

    {NOTE_G4, QUARTER_NOTE, 0.5f},
    {NOTE_F4, QUARTER_NOTE, 0.5f},
    {NOTE_E4, QUARTER_NOTE, 0.5f},
    {NOTE_D4, QUARTER_NOTE, 0.5f},

    {NOTE_C4, QUARTER_NOTE, 0.5f},
    {NOTE_C4, QUARTER_NOTE, 0.5f},
    {NOTE_D4, QUARTER_NOTE, 0.5f},
    {NOTE_E4, QUARTER_NOTE, 0.5f},

    {NOTE_D4, QUARTER_NOTE, 0.5f},
    {NOTE_C4, HALF_NOTE, 0.5f},
    {NOTE_REST, 0, 0}, /* end */
};

static const struct note ha_ji_mi[] = {
    /* ·1 ·6 ·5 ·5 | ·3 ·5 ·3 ·1 */
    {NOTE_C5, EIGHTH_NOTE, 0.35f},
    {NOTE_A5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_REST, PHRASE_PAUSE, 0},

    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_C5, QUARTER_NOTE, 0.35f},
    {NOTE_REST, PHRASE_PAUSE, 0},

    /* 6 ·1 ·2 ·3 | ·3 ·5 ·3 ·1 */
    {NOTE_A4, EIGHTH_NOTE, 0.5f},
    {NOTE_C5, EIGHTH_NOTE, 0.35f},
    {NOTE_D5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_REST, PHRASE_PAUSE, 0},

    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_C5, QUARTER_NOTE, 0.35f},
    {NOTE_REST, PHRASE_PAUSE, 0},

    /* 6 ·1 ·3 ·2 | ·5 ·4 ·3 ·4 ·3 ·1 6 5 */
    {NOTE_A4, EIGHTH_NOTE, 0.5f},
    {NOTE_C5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_D5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_F5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_F5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_C5, EIGHTH_NOTE, 0.35f},
    {NOTE_A4, EIGHTH_NOTE, 0.5f},
    {NOTE_G4, QUARTER_NOTE, 0.5f},
    {NOTE_REST, PHRASE_PAUSE, 0},

    /* ·1 7 ·1 ·6 ·5 | ·6 ·7 ·6 ·5 ·4 ·3 ·2 ·3 ·1 5 */
    {NOTE_C5, EIGHTH_NOTE, 0.35f},
    {NOTE_B4, EIGHTH_NOTE, 0.5f},
    {NOTE_C5, EIGHTH_NOTE, 0.35f},
    {NOTE_A5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_A5, EIGHTH_NOTE, 0.35f},
    {NOTE_B4, EIGHTH_NOTE, 0.5f},
    {NOTE_A5, EIGHTH_NOTE, 0.35f},
    {NOTE_G5, EIGHTH_NOTE, 0.35f},
    {NOTE_F5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_D5, EIGHTH_NOTE, 0.35f},
    {NOTE_E5, EIGHTH_NOTE, 0.35f},
    {NOTE_C5, QUARTER_NOTE, 0.35f},
    {NOTE_G4, HALF_NOTE, 0.5f},

    {NOTE_REST, 0, 0} /* end */
};

void buzzer_play_song(enum song song)
{
	const struct note *notes = NULL;
	uint32_t len_song = 0;
	switch (song) {
	case ODE_TO_JOY:
		notes = ode_to_joy;
		len_song = sizeof(ode_to_joy) / sizeof(struct note);
		break;
	case HA_JI_MI:
		notes = ha_ji_mi;
		len_song = sizeof(ha_ji_mi) / sizeof(struct note);
		break;
	default:
		break;
	}

	for (uint32_t i = 0; i < len_song; ++i) {
		buzzer_ctrl(notes[i].freq, notes[i].duration, notes[i].volume);
		buzzer_ctrl(notes[i].freq, PHRASE_PAUSE, 0); /* to distinguish notes */
	}
}