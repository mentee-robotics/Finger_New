/*
 * calibration.c
 *
 *  Created on: Jun 20, 2023
 *      Author: nir
 */
#include "calibration.h"

// Calibration function for motor control
void calibration(MAX22200_StatusReg* stat_reg, Motor* motors[]) {
    // Define and initialize counters and calibration variables
    uint32_t last_counter_value[3] = {0};
    uint32_t stable_start_time[3] = {0};
    uint32_t current_time = 0;
    Calibration_State state = STATE_INIT;  // Start at the initial state

    while (1) {
        switch (state) {
            case STATE_INIT:
                // Initial configuration for all motors
                for (int i = 0; i < 3; ++i) {
                    Motor_setDirection(i+1, SLEEP, stat_reg);
                    Set_VDRnCDR(&motors[i]->cfg1_, 0);//Go to CDR mode Current Drive
                    Set_VDRnCDR(&motors[i]->cfg2_, 0);//Go to CDR mode Current Drive
                    Set_HIT(&motors[i]->cfg1_, 100);
                    Set_HIT(&motors[i]->cfg2_, 100);
                    Set_HOLD(&motors[i]->cfg1_, 100);
                    Set_HOLD(&motors[i]->cfg2_, 100);
                    Motor_writeCfgRegister(&motors[i]->cfg1_, motors[i]->channel1_);
                    Motor_writeCfgRegister(&motors[i]->cfg2_, motors[i]->channel2_);
                    Motor_setDirection(i+1, DOWN, stat_reg);
                    last_counter_value[i] = counter[i];
                    motor_stable[i] = 0;
                }
//                HAL_Delay(3000);
//                for (int i = 0; i < 3; ++i) {
//                    Motor_setDirection(i+1, UP, stat_reg);
//                }
//                HAL_Delay(3000);
//               for (int i = 0; i < 3; ++i) {
//				   Motor_setDirection(i+1, DOWN, stat_reg);
//               }
			HAL_Delay(3000);
                state = STATE_CALIBRATION;  // Move to calibration state
                break;

            case STATE_CALIBRATION:
                // Calibration process
                current_time = HAL_GetTick();
                for (int i = 0; i < 3; ++i) {
                    if (!motor_stable[i]) {
                        if (counter[i] != last_counter_value[i]) {
                            // Counter value changed, update last value and start time
                            last_counter_value[i] = counter[i];
                            stable_start_time[i] = current_time;
                        } else if ((current_time - stable_start_time[i]) >= 250) {
                            // Counter has been stable for at least 1 seconds
                            counter_calibration[i] = counter[i];
                            motor_stable[i] = 1;
                            Motor_setDirection(i+1, BREAK, stat_reg);
                        }
                    }
                }
                // If all motors are stable(stopped), move to stability check state
                if (motor_stable[0] && motor_stable[1] && motor_stable[2]) {
                    state = STATE_STABILITY_CHECK;
                }
                break;

            case STATE_STABILITY_CHECK:
                // Check the stability of each motor and finalize calibration if stable
                HAL_Delay(500);
                calibrated = true;
                state = STATE_BREAK;  // Move to break state
                break;

            case STATE_BREAK:
                // Break state to stop all motors
                for (int i = 0; i < 3; ++i) {
                    Motor_setDirection(i+1, BREAK, stat_reg);
                }
                return;  // Exit function
        }
    }
}
