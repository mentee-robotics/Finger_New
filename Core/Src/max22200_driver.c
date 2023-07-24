/*
 * max22200.c
 *
 *  Created on: Jun 13, 2023
 *      Author: nir
 *
 *  This codebase is for a MAX22200 Motor driver. It deals with
 *  initializing various components, executing commands, switching between states,
 *  and handling errors for 3 motors.
 */

#include "max22200_driver.h"
#include <stdio.h>
#include <stdint.h>


uint8_t MAX22200_tx[16];  // SPI TX buffer
uint8_t MAX22200_rx[16];  // SPI RX buffer

// Initialize the MAX22200
void MAX22200_init(MAX22200_status *stats)
{
    // Enable the device
    MAX22200_ENABLE_HIGH;

    // Make sure Channels aren't triggered
    MAX22200_TRIGA_LOW;
    MAX22200_TRIGB_LOW;

    MAX22200_CS_HIGH;  // Make sure CS  pin is high
    MAX22200_CMD_LOW;  // Make sure CMD pin is low

    MAX22200_write_register(0x00, 0x01); // Set active bit, otherwise the chip doesn't work
    stats->status = MAX22200_write_register(0x00, 0x00000001);   // Set Chip Active / returns the status byte
    uint32_t data1= 0X0004AA01;
    uint32_t data2 = 0x7F6432B8;
    uint32_t data3 = 0x7F0032B8;
    //this sequence was added for handling bringup of the max according to datasheet sequence page 25 datasheet
    // Step 1: Read status register
    stats->result = MAX22200_read_register(MAX22200_STATUS, &stats->status);

    while (1) { // Main loop

       // Step 2: Write to status register
        stats->status = MAX22200_write_register(MAX22200_STATUS, data1);

       while (stats->status == 0x02) {
           // If status is 0x02, write to the register again
           stats->status = MAX22200_write_register(MAX22200_STATUS, data1);
       }

       // Step 3: Write to channel register
       stats->status = MAX22200_write_register(MAX22200_CFG_CH_1, data2);

       if (stats->status == 0x02) {
           // If status is 0x02, start over from writing to the status register
           continue;
       }
       stats->status = MAX22200_write_register(MAX22200_CFG_CH_2, data3);

       if (stats->status == 0x02) {
           // If status is 0x02, start over from writing to the status register
           continue;
       }
       stats->status = MAX22200_write_register(MAX22200_CFG_CH_3, data2);

       if (stats->status == 0x02) {
           // If status is 0x02, start over from writing to the status register
           continue;
       }
       stats->status = MAX22200_write_register(MAX22200_CFG_CH_4, data3);

       if (stats->status == 0x02) {
           // If status is 0x02, start over from writing to the status register
           continue;
       }
       stats->status = MAX22200_write_register(MAX22200_CFG_CH_5, data2);
       // If status is 0x02, start over from writing to the status register
       if (stats->status == 0x02) {
           continue;
       }
       stats->status = MAX22200_write_register(MAX22200_CFG_CH_6, data3);

       if (stats->status == 0x02) {
           // If status is 0x02, start over from writing to the status register
           continue;
       }
       // Step 4: Read status register
       stats->result = MAX22200_read_register(MAX22200_STATUS, &stats->status);
       if (stats->status != 0x01 && stats->status != 0x03) {
           // If status is not 0x01 or 0x03, start over from writing to the status register
           continue;
       }
       while (stats->status == 0x03) {
           // If status is 0x03, keep reading the status register until it becomes 0x01
           stats->result = MAX22200_read_register(MAX22200_STATUS, &stats->status);
       }

       if (stats->status == 0x01) {
           break;
       }
       stats->fault = MAX22200_read_register(MAX22200_FAULT, &stats->status);
    }
}

// Initialize the status register and status struct
void MAX22200_init_statreg(MAX22200_status* stats_, MAX22200_StatusReg* status_Reg)
{
    *status_Reg = (MAX22200_StatusReg) {
        .ONCH = {0, 0, 0, 0, 0, 0, 0, 0},
        .M_OVT = 0,
        .M_OCP = 0,
        .M_OLF = 0,
        .M_HHF = 0,
        .M_DPM = 0,
        .M_COMF = 1,
        .M_UVM = 0,
        .FREQM = 0,
        .CM76 = {1, 0},
        .CM54 = {1, 0},
        .CM32 = {1, 0},
        .CM10 = {1, 0},
        .OVT = 0,
        .OCP = 0,
        .OLF = 0,
        .HHF = 0,
        .DPM = 0,
        .COMER = 0,
        .UVM = 0,
        .ACTIVE = 1
    };
    *stats_ = (MAX22200_status) {
        .status = 0x02,
        .fault = 0x00,
        .result = 0x00,
        .status_reg = 0x01
    };
}

// Write to a register
uint8_t MAX22200_write_register(uint8_t reg_adr, uint32_t data)
{
    // First, write the Command byte to setup the SPI transfer
    MAX22200_tx[0] = ((reg_adr << 1) & 0x7e) | 0x80;                    // MSB = 1 -> write - LSB = 0 -> 32 bit register access

    MAX22200_CMD_HIGH;
    MAX22200_CS_LOW;

    HAL_SPI_TransmitReceive(&hspi3, MAX22200_tx, MAX22200_rx, 1, 2);  // SPI RW 1 byte

    MAX22200_CS_HIGH;
    MAX22200_CMD_LOW;

    // Now write the actual data
    MAX22200_tx[3] = ( data >> 24 ) & 0xff;   // MS Byte
    MAX22200_tx[2] = ( data >> 16 ) & 0xff;   //
    MAX22200_tx[1] = ( data >> 8  ) & 0xff;   //
    MAX22200_tx[0] = ( data       ) & 0xff;   // LS Byte

    MAX22200_CS_LOW;

    HAL_SPI_Transmit(&hspi3, MAX22200_tx, 4, 2);                      // SPI W 4 bytes

    MAX22200_CS_HIGH;

    return MAX22200_rx[0];                                            // Return the status byte
}

// Read from a register
uint32_t MAX22200_read_register(uint8_t reg_adr, uint8_t* status)//added status pointer for sniffing out the status of the transmitrecieve operation
{
    // First, write the Command byte to setup the SPI transfer
    MAX22200_tx[0] = ((reg_adr << 1) & 0x7e);                           // MSB = 0 -> read - LSB = 0 -> 32 bit register access

    MAX22200_CMD_HIGH;
    MAX22200_CS_LOW;

    HAL_SPI_TransmitReceive(&hspi3, MAX22200_tx, MAX22200_rx, 1, 2);  // SPI RW 1 byte
    *status = MAX22200_rx[0];//added myself

    MAX22200_CS_HIGH;
    MAX22200_CMD_LOW;

    // For read, the TX bytes aren't important, but we set them to 0
    MAX22200_tx[0] = 0;   // MS Byte
    MAX22200_tx[1] = 0;   //
    MAX22200_tx[2] = 0;   //
    MAX22200_tx[3] = 0;   // LS Byte

    MAX22200_CS_LOW;

    HAL_SPI_TransmitReceive(&hspi3, MAX22200_tx, MAX22200_rx, 4, 2);  // SPI Read 4 bytes

    MAX22200_CS_HIGH;

    uint32_t result = ((MAX22200_rx[0] << 24) | (MAX22200_rx[1] << 16) | (MAX22200_rx[2] << 8) | (MAX22200_rx[3]));

    return result;
}

//********************************************************************
//*
//* Function:    MAX22200_Set_CH
//* Description: Set up one Channel for driving
//*
//* Input:
//*     channel          1 - 8  -> Selected channel to set up (We're counting Board Channels !! NOT Chip Channels)
//*     HalfScale        0 = FullScale      1 = HalfScale
//*     HOLD_DutyCycle   0 = OFF state,     1-126 duty-cycle     127 ON state
//*     TRIG_pin         0 = SPI Control    1 = TrigPin Control
//*     HIT_DutyCycle    0 = OFF state,     1-126 duty-cycle     127 ON state
//*     HIT_Time         0 = no Hit         1-254 x40/F_Chop     255 continuous HIT
//*     V_Mode           0 = CurrentMode    1 = VoltageMode
//*     HighSideMode     0 = LowSide Mode   1 = HighSide Mode
//*     FREQ_CFG         0 = FreqMain/4     1 = FreqMain/3       2 = FreqMain/2  3 = FreqMain
//*     SRC              0 = No Slew Ctrl   1 = Slew rate controlled
//*     OL_EN            0 = No OpenLoadD   1 = Open Load detection enabled
//*     DPM_EN           0 = No PlungerD    1 = Plunger movement detect enabled
//*     HHF_EN           0 = No HitCurr Chk 1 = Hit Current Check enabled
//*
//* Output: None
//*
//********************************************************************/

// Print the contents of all registers (for debugging purposes)
void MAX22200_print_all_registers()
{
    uint32_t result = 0;

    for (int i = 0; i < 11; i++) {
        // result = MAX22200_read_register(i, &status);
        printf("MAX22000 reg 0x%02x - 0x%08lx\r\n", i, result);
    }
}

// Function to build STATUS register
uint32_t buildStatusRegister(const MAX22200_StatusReg* status)
{
    uint32_t result = 0;

    for (int i = 0; i < 8; i++) {
        result |= status->ONCH[7 - i] << (31 - i);  // Process ONCH array in reverse
    }

    result |= (status->M_OVT << 23);
    result |= (status->M_OCP << 22);
    result |= (status->M_OLF << 21);
    result |= (status->M_HHF << 20);
    result |= (status->M_DPM << 19);
    result |= (status->M_COMF << 18);
    result |= (status->M_UVM << 17);
    result |= (status->FREQM << 16);

    result |= (status->CM76[0] << 15);
    result |= (status->CM76[1] << 14);

    result |= (status->CM54[0] << 13);
    result |= (status->CM54[1] << 12);

    result |= (status->CM32[0] << 11);
    result |= (status->CM32[1] << 10);

    result |= (status->CM10[0] << 9);
    result |= (status->CM10[1] << 8);

    result |= (status->OVT << 7);
    result |= (status->OCP << 6);
    result |= (status->OLF << 5);
    result |= (status->HHF << 4);
    result |= (status->DPM << 3);
    result |= (status->COMER << 2);
    result |= (status->UVM << 1);
    result |= status->ACTIVE;

    return result;
}

// Update the status register and transmit it via SPI
void update_status_register(MAX22200_StatusReg* statusReg, uint8_t ONCH[], uint8_t M_OVT, uint8_t M_OCP, uint8_t M_OLF, uint8_t M_HHF, uint8_t M_DPM, uint8_t M_COMF, uint8_t M_UVM, uint8_t FREQM, uint8_t CM76[], uint8_t CM54[], uint8_t CM32[], uint8_t CM10[], uint8_t OVT, uint8_t OCP, uint8_t OLF, uint8_t HHF, uint8_t DPM, uint8_t COMER, uint8_t UVM, uint8_t ACTIVE)
{
    memcpy(statusReg->ONCH, ONCH, sizeof(statusReg->ONCH));
    statusReg->M_OVT = M_OVT;
    statusReg->M_OCP = M_OCP;
    statusReg->M_OLF = M_OLF;
    statusReg->M_HHF = M_HHF;
    statusReg->M_DPM = M_DPM;
    statusReg->M_COMF = M_COMF;
    statusReg->M_UVM = M_UVM;
    statusReg->FREQM = FREQM;
    memcpy(statusReg->CM76, CM76, sizeof(statusReg->CM76));
    memcpy(statusReg->CM54, CM54, sizeof(statusReg->CM54));
    memcpy(statusReg->CM32, CM32, sizeof(statusReg->CM32));
    memcpy(statusReg->CM10, CM10, sizeof(statusReg->CM10));
    statusReg->OVT = OVT;
    statusReg->OCP = OCP;
    statusReg->OLF = OLF;
    statusReg->HHF = HHF;
    statusReg->DPM = DPM;
    statusReg->COMER = COMER;
    statusReg->UVM = UVM;
    statusReg->ACTIVE = ACTIVE;

    uint32_t result = buildStatusRegister(statusReg);
    MAX22200_build_and_send_status_register(statusReg);
}

// Build and send the STATUS register via SPI
void MAX22200_build_and_send_status_register(const MAX22200_StatusReg* statusReg)
{
    // First, write the Command byte to setup the SPI transfer
    MAX22200_tx[0] = (MAX22200_STATUS << 1) & 0x7e;    // MSB = 0 -> read - LSB = 0 -> 32 bit register access

    MAX22200_CMD_HIGH;
    MAX22200_CS_LOW;

    HAL_SPI_TransmitReceive(&hspi3, MAX22200_tx, MAX22200_rx, 1, 2);  // SPI RW 1 byte

    MAX22200_CS_HIGH;
    MAX22200_CMD_LOW;

    // Build the data bytes
    MAX22200_tx[3] = (statusReg->ACTIVE << 7) | (statusReg->UVM << 6) | (statusReg->COMER << 5) | (statusReg->DPM << 4) | (statusReg->HHF << 3) | (statusReg->OLF << 2) | (statusReg->OCP << 1) | (statusReg->OVT);
    MAX22200_tx[2] = (statusReg->CM10[1] << 7) | (statusReg->CM10[0] << 6) | (statusReg->CM32[1] << 5) | (statusReg->CM32[0] << 4) | (statusReg->CM54[1] << 3) | (statusReg->CM54[0] << 2) | (statusReg->CM76[1] << 1) | (statusReg->CM76[0]);
    MAX22200_tx[1] = (statusReg->FREQM << 4) | (statusReg->M_UVM << 3) | (statusReg->M_COMF << 2) | (statusReg->M_DPM << 1) | (statusReg->M_HHF);
    MAX22200_tx[0] = (statusReg->M_OLF << 7) | (statusReg->M_OCP << 6) | (statusReg->M_OVT << 5) | (statusReg->ONCH[7] << 7) | (statusReg->ONCH[6] << 6) | (statusReg->ONCH[5] << 5) | (statusReg->ONCH[4] << 4) | (statusReg->ONCH[3] << 3) | (statusReg->ONCH[2] << 2) | (statusReg->ONCH[1] << 1) | (statusReg->ONCH[0]);

    MAX22200_CS_LOW;

    HAL_SPI_Transmit(&hspi3, MAX22200_tx, 4, 2);  // SPI W 4 bytes

    MAX22200_CS_HIGH;
}

