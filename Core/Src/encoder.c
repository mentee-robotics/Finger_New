/*
 * Encoder.c
 *
 * Created on: Jun 13, 2023
 * Author: nir
 */

#include "encoder.h"

// Define the number of ticks per revolution
#define TICKS_PER_REV 4096

/*
 * Initialize an encoder.
 *
 * This function initializes an encoder with the given timer and calibration factor.
 * The counter, previous_counter and absolute_position are reset to 0.
 */
void encoder_init(Encoder* self, TIM_HandleTypeDef* htim, float calibration_factor) {
    self->htim = htim;
    self->counter = 0;
    self->previous_counter = 0;
    self->absolute_position = 0;
}

/*
 * Get the revolutions of an encoder.
 *
 * This function calculates and returns the revolutions of an encoder
 * by getting its absolute position and dividing it by the ticks per revolution.
 */
float get_revolution(Encoder* en) {
	get_absolute_position(en);
    return (float)en->absolute_position / TICKS_PER_REV;
}

/*
 * Get the absolute position of an encoder.
 *
 * This function calculates the absolute position of an encoder by
 * getting the current counter, calculating the difference from the
 * previous counter, and then updating the absolute position and previous counter.
 */
int32_t get_absolute_position(Encoder* en) {
    uint16_t current_counter = __HAL_TIM_GET_COUNTER(en->htim);
    int16_t diff = (int16_t)(current_counter - en->previous_counter);

    // Correct for counter overflow
    if (diff < -32000) {
        diff += 65536;
    }
    // Correct for counter underflow
    else if (diff > 32000) {
        diff -= 65536;
    }

    en->absolute_position += diff;
    en->previous_counter = current_counter;

    return en->absolute_position;
}
