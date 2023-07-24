#ifndef MAX22200_DRIVER_H
#define MAX22200_DRIVER_H

#include "max22200_registers.h"
#include <stdint.h>
#include <string.h>
#include "gpio.h"
#include "spi.h"

typedef struct {
    uint8_t ONCH[8];
    uint8_t M_OVT;
    uint8_t M_OCP;
    uint8_t M_OLF;
    uint8_t M_HHF;
    uint8_t M_DPM;
    uint8_t M_COMF;
    uint8_t M_UVM;
    uint8_t FREQM;
    uint8_t CM76[2];
    uint8_t CM54[2];
    uint8_t CM32[2];
    uint8_t CM10[2];
    uint8_t OVT;
    uint8_t OCP;
    uint8_t OLF;
    uint8_t HHF;
    uint8_t DPM;
    uint8_t COMER;
    uint8_t UVM;
    uint8_t ACTIVE;
} MAX22200_StatusReg;

typedef struct {
    uint8_t status;
    uint32_t result;
    uint8_t fault;
    uint32_t status_reg;
} MAX22200_status;


// Bit definitions

// Bit definitions for the Channels:

/* Half/Full-scale select                   */    #define BIT_CH_HFS       0x80000000
/* Bits for HOLD Current                    */    #define BIT_CH_HOLD      0x7f000000
/* Trigger select bit                       */    #define BIT_CH_TRGnSPI   0x00800000
/* Bits for HIT Current                     */    #define BIT_CH_HIT       0x007f0000
/* Bits for HIT Time                        */    #define BIT_CH_HIT_T     0x0000ff00
/* Current / Voltage drive select bit       */    #define BIT_CH_VDRnCDR   0x00000080
/* High- / Low- Side mode select bit        */    #define BIT_CH_HSnLS     0x00000040
/* Chopping Frequency configuration bits    */    #define BIT_CH_FREQ_CFG  0x00000030
/* Slew-rate control enable bit             */    #define BIT_CH_SRC       0x00000008
/* Open-load detection enable bit           */    #define BIT_CH_OL_EN     0x00000004
/* Detection of Plunger Movement enable bit */    #define BIT_CH_DPM_EN    0x00000002
/* HIT Current check enable bit             */    #define BIT_CH_HHF_EN    0x00000001


// CS commands
#define MAX22200_CS_LOW    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,  SPI_CS_Pin,  GPIO_PIN_RESET);
#define MAX22200_CS_HIGH   HAL_GPIO_WritePin(SPI_CS_GPIO_Port,  SPI_CS_Pin,  GPIO_PIN_SET);
#define MAX22200_CMD_LOW   HAL_GPIO_WritePin(SPI_MAX_CMD_GPIO_Port, SPI_MAX_CMD_Pin, GPIO_PIN_RESET);
#define MAX22200_CMD_HIGH  HAL_GPIO_WritePin(SPI_MAX_CMD_GPIO_Port, SPI_MAX_CMD_Pin, GPIO_PIN_SET);


#define MAX22200_ENABLE_LOW   HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, GPIO_PIN_RESET);
#define MAX22200_ENABLE_HIGH  HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, GPIO_PIN_SET);


// Trigger Pins
#define MAX22200_TRIGA_LOW   HAL_GPIO_WritePin(MAX_TRIGA_GPIO_Port, MAX_TRIGA_Pin, GPIO_PIN_RESET);
#define MAX22200_TRIGA_HIGH  HAL_GPIO_WritePin(MAX_TRIGA_GPIO_Port, MAX_TRIGA_Pin, GPIO_PIN_SET);
#define MAX22200_TRIGB_LOW   HAL_GPIO_WritePin(MAX_TRIGB_GPIO_Port, MAX_TRIGB_Pin, GPIO_PIN_RESET);
#define MAX22200_TRIGB_HIGH  HAL_GPIO_WritePin(MAX_TRIGB_GPIO_Port, MAX_TRIGB_Pin, GPIO_PIN_SET);

void MAX22200_init(MAX22200_status *stats);
void MAX22200_init_statreg(MAX22200_status* stats_,MAX22200_StatusReg* status_Reg);
uint8_t MAX22200_write_register(uint8_t reg_adr, uint32_t data);
uint32_t MAX22200_read_register(uint8_t reg_adr, uint8_t *status);

void MAX22200_print_all_registers();
void update_status_register(MAX22200_StatusReg *statusReg, uint8_t ONCH[], uint8_t M_OVT, uint8_t M_OCP, uint8_t M_OLF, uint8_t M_HHF, uint8_t M_DPM, uint8_t M_COMF, uint8_t M_UVM, uint8_t FREQM, uint8_t CM76[], uint8_t CM54[], uint8_t CM32[], uint8_t CM10[], uint8_t OVT, uint8_t OCP, uint8_t OLF, uint8_t HHF, uint8_t DPM, uint8_t COMER, uint8_t UVM, uint8_t ACTIVE);
void MAX22200_build_and_send_status_register(const MAX22200_StatusReg *statusReg);

#endif
