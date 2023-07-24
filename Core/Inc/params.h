/*
 * params.h
 *
 *  Created on: Jun 20, 2023
 *      Author: nir
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_
#include <stdbool.h>
#include "fdcan.h"

//encoder
uint32_t counter[3] = {0};
//calibration
uint32_t counter_calibration[3] = {0};
int motor_stable[3] = {0};
bool calibrated=false;
//watchdog
bool watchdog1=false;
bool watchdog2=false;
bool watchdog3=false;

//pid params
float kp = 0.1;
float ki = 0.002;
float kd = 0.0;
double outputs[3]={0};
int pwm_outputs[3]={0};
const double PI = 3.1415926535897932384626433832795028841971693993751058209;
int pos[3]={152,170,170};

//comm params


bool receiveNsendFlag = false;


//current sensing
uint32_t ADC_RawReading[2];

//debug
uint32_t loop_count=0;
#endif /* INC_PARAMS_H_ */
