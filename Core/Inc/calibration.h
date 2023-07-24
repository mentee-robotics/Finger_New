/*
 * calibration.h
 *
 *  Created on: Jun 20, 2023
 *      Author: nir
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "motor.h"
#include <stdbool.h>

// Enumeration for different calibration states
typedef enum {
    STATE_INIT,
    STATE_CALIBRATION,
    STATE_STABILITY_CHECK,
    STATE_BREAK
} Calibration_State;

// Calibration function
void calibration(MAX22200_StatusReg* stat_reg, Motor* motors[]);

// Global variables related to calibration
extern uint32_t counter_calibration[3];
extern uint32_t counter[3];
extern bool calibrated;
extern int motor_stable[3];

#endif /* INC_CALIBRATION_H_ */
