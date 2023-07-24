#include "motor.h"
void MAX22000_init_cfg_reg(CFG_CH_Register *cfg1, CFG_CH_Register *cfg2) {
    *cfg1 = (CFG_CH_Register) {
        .HFS = 0,
        .HOLD = 1,
        .TRGnSPI = 0,
        .HIT = 1,
        .HIT_T = 1,
        .VDRnCDR = 1,
        .HSnLS = 0,
        .FREQ_CFG = 3,
        .SRC = 1,
        .OL_EN = 0,
        .DPM_EN = 0,
        .HHF_EN = 0
    };

    *cfg2 = (CFG_CH_Register) {
        .HFS = 0,
        .HOLD = 1,
        .TRGnSPI = 0,
        .HIT = 1,
        .HIT_T = 1,
        .VDRnCDR = 1,
        .HSnLS = 0,
        .FREQ_CFG = 3,
        .SRC = 1,
        .OL_EN = 0,
        .DPM_EN = 0,
        .HHF_EN = 0
    };
}

void Motor_init(Motor* motor, CFG_CH_Register* cfg1, CFG_CH_Register* cfg2, int motorNumber) {
    motor->m = motorNumber;
    motor->channel1_ = (motorNumber - 1) * 2 + 1;
    motor->channel2_ = (motorNumber - 1) * 2 + 2;
    motor->cfg1_ = *cfg1;
    motor->cfg2_ = *cfg2;

    Motor_writeCfgRegister(&motor->cfg1_, motor->channel1_);
    Motor_writeCfgRegister(&motor->cfg2_, motor->channel2_);
}

void Motor_setMode(Motor* motor, MotorDriveMode mod)
{
    switch (mod)
    {
    case Voltage:
    {
        Set_VDRnCDR(&motor->cfg1_, 1);
        Set_VDRnCDR(&motor->cfg2_, 1);
        break;
    }
    case Current:
    {
        Set_VDRnCDR(&motor->cfg1_, 0);
        Set_VDRnCDR(&motor->cfg2_, 0);
        break;
    }
    }
}

void Motor_writeCfgRegister(CFG_CH_Register* config, int channel)
{
    if ((channel > 8) || (channel < 1)) return;
    if (config->HFS > 1) return;
    if (config->HOLD > 127) return;
    if (config->TRGnSPI > 1) return;
    if (config->HIT > 127) return;
    if (config->HIT_T > 255) return;
    if (config->VDRnCDR > 1) return;
    if (config->HSnLS > 1) return;
    if (config->FREQ_CFG > 3) return;
    if (config->SRC > 1) return;
    if (config->OL_EN > 1) return;
    if (config->DPM_EN > 1) return;
    if (config->HHF_EN > 1) return;

    uint32_t data = 0;
    data |= ((config->HFS << 31) & BIT_CH_HFS);
    data |= ((config->HOLD << 24) & BIT_CH_HOLD);
    data |= ((config->TRGnSPI << 23) & BIT_CH_TRGnSPI);
    data |= ((config->HIT << 16) & BIT_CH_HIT);
    data |= ((config->HIT_T << 8) & BIT_CH_HIT_T);
    data |= ((config->VDRnCDR << 7) & BIT_CH_VDRnCDR);
    data |= ((config->HSnLS << 6) & BIT_CH_HSnLS);
    data |= ((config->FREQ_CFG << 4) & BIT_CH_FREQ_CFG);
    data |= ((config->SRC << 3) & BIT_CH_SRC);
    data |= ((config->OL_EN << 2) & BIT_CH_OL_EN);
    data |= ((config->DPM_EN << 1) & BIT_CH_DPM_EN);
    data |= ((config->HHF_EN << 0) & BIT_CH_HHF_EN);
    MAX22200_write_register(channel, data);
}

void Motor_setDirection(int MotorNum, MotorCommandType cmd, MAX22200_StatusReg* stsreg)
{
    switch (cmd)
    {
    case SLEEP:
        stsreg->ONCH[(MotorNum - 1) * 2] = 0;
        stsreg->ONCH[(MotorNum - 1) * 2 + 1] = 0;
        break;
    case DOWN:
        stsreg->ONCH[(MotorNum - 1) * 2] = 1;
        stsreg->ONCH[(MotorNum - 1) * 2 + 1] = 0;
        break;
    case UP:
        stsreg->ONCH[(MotorNum - 1) * 2] = 0;
        stsreg->ONCH[(MotorNum - 1) * 2 + 1] = 1;
        break;
    case BREAK:
        stsreg->ONCH[(MotorNum - 1) * 2] = 1;
        stsreg->ONCH[(MotorNum - 1) * 2 + 1] = 1;
        break;
    }
    uint32_t dir_status = buildStatusRegister(stsreg);
    MAX22200_write_register(MAX22200_STATUS, dir_status);
}

void Set_HFS(CFG_CH_Register* cfg, uint8_t HFS)
{
    if (HFS > 1) return;
    cfg->HFS = HFS;
}

void Set_HOLD(CFG_CH_Register* cfg, uint8_t HOLD)
{
    if (HOLD > 127) return;
    cfg->HOLD = HOLD;
}

void Set_TRGnSPl(CFG_CH_Register* cfg, uint8_t TRGnSPl)
{
    if (TRGnSPl > 1) return;
    cfg->TRGnSPI = TRGnSPl;
}

void Set_HIT(CFG_CH_Register* cfg, uint8_t HIT)
{
    if (HIT > 127) return;
    cfg->HIT = HIT;
}

void Set_HIT_T(CFG_CH_Register* cfg, uint8_t HIT_T)
{
    if (HIT_T > 255) return;
    cfg->HIT_T = HIT_T;
}

void Set_VDRnCDR(CFG_CH_Register* cfg, uint8_t VDRnCDR)
{
    if (VDRnCDR > 1) return;
    cfg->VDRnCDR = VDRnCDR;
}

void Set_HSnLS(CFG_CH_Register* cfg, uint8_t HSnLS)
{
    if (HSnLS > 1) return;
    cfg->HSnLS = HSnLS;
}

void Set_FREQ_CFG(CFG_CH_Register* cfg, uint8_t FREQ_CFG)
{
    if (FREQ_CFG > 3) return;
    cfg->FREQ_CFG = FREQ_CFG;
}

void Set_SRC(CFG_CH_Register* cfg, uint8_t SRC)
{
    if (SRC > 1) return;
    cfg->SRC = SRC;
}

void Set_OL_EN(CFG_CH_Register* cfg, uint8_t OL_EN)
{
    if (OL_EN > 1) return;
    cfg->OL_EN = OL_EN;
}

void Set_DPM_EN(CFG_CH_Register* cfg, uint8_t DPM_EN)
{
    if (DPM_EN > 1) return;
    cfg->DPM_EN = DPM_EN;
}

void Set_HHF_EN(CFG_CH_Register* cfg, uint8_t HHF_EN)
{
    if (HHF_EN > 1) return;
    cfg->HHF_EN = HHF_EN;
}
