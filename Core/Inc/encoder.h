/*
 * encoder.h
 *
 * Created on: Jun 13, 2023
 * Author: nir
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stdint.h"
#include "stm32g4xx_hal.h"

/* The Encoder structure encapsulates a timer, a counter,
 * a previous counter value, and the absolute position.
 */
typedef struct {
    TIM_HandleTypeDef* htim;         // The hardware timer associated with the encoder
    uint16_t counter;                // The current encoder count
    uint16_t previous_counter;       // The previous encoder count
    int32_t absolute_position;       // The absolute position
 } Encoder;

void encoder_init(Encoder* self, TIM_HandleTypeDef* htim, float calibration_factor);

// Declare the functions that will be used as methods
float get_revolution(Encoder* en);
int32_t get_absolute_position(Encoder* en);

#endif /* INC_ENCODER_H_ */
