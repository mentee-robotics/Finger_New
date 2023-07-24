/*
 * Motor.h
 *
 *  Created on: May 15, 2023
 *      Author: nir
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "max22200_driver.h"

typedef struct {
    uint8_t HFS        : 1;
    uint8_t HOLD       : 7;
    uint8_t TRGnSPI    : 1;
    uint8_t HIT        : 7;
    uint8_t HIT_T      : 8;
    uint8_t VDRnCDR    : 1;
    uint8_t HSnLS      : 1;
    uint8_t FREQ_CFG   : 2;
    uint8_t SRC        : 1;
    uint8_t OL_EN      : 1;
    uint8_t DPM_EN     : 1;
    uint8_t HHF_EN     : 1;
} CFG_CH_Register;

typedef struct {
    int channel1_;
    int channel2_;
    int m;
    CFG_CH_Register cfg1_;
    CFG_CH_Register cfg2_;
} Motor;

typedef enum{
    Current,
    Voltage
}MotorDriveMode;

typedef enum {
    SLEEP,
    DOWN,
    UP,
    BREAK
} MotorCommandType;

void MAX22000_init_cfg_reg(CFG_CH_Register *cfg1, CFG_CH_Register *cfg2);
void Motor_init(Motor* motor, CFG_CH_Register* cfg1, CFG_CH_Register* cfg2, int motorNumber);
void Motor_setMode(Motor* motor, MotorDriveMode mod);
void Motor_writeCfgRegister(CFG_CH_Register* config, int channel);
void Motor_setDirection(int MotorNum, MotorCommandType cmd, MAX22200_StatusReg* stsreg);
void Set_HFS(CFG_CH_Register* cfg, uint8_t HFS);
void Set_HOLD(CFG_CH_Register* cfg, uint8_t HOLD);
void Set_TRGnSPl(CFG_CH_Register* cfg, uint8_t TRGnSPl);
void Set_HIT(CFG_CH_Register* cfg, uint8_t HIT);
void Set_HIT_T(CFG_CH_Register* cfg, uint8_t HIT_T);
void Set_VDRnCDR(CFG_CH_Register* cfg, uint8_t VDRnCDR);
void Set_HSnLS(CFG_CH_Register* cfg, uint8_t HSnLS);
void Set_FREQ_CFG(CFG_CH_Register* cfg, uint8_t FREQ_CFG);
void Set_SRC(CFG_CH_Register* cfg, uint8_t SRC);
void Set_OL_EN(CFG_CH_Register* cfg, uint8_t OL_EN);
void Set_DPM_EN(CFG_CH_Register* cfg, uint8_t DPM_EN);
void Set_HHF_EN(CFG_CH_Register* cfg, uint8_t HHF_EN);

#endif /* MOTOR_H_ */
