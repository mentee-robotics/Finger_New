/*
 * MAX22200_registers.h
 *
 *  Created on: Jun 11, 2023
 *      Author: nir
 */

#ifndef INC_MAX22200_REGISTERS_H_
#define INC_MAX22200_REGISTERS_H_

//
//Command read/write values
//

#define MAX22200_READ 0
#define MAX22200_WRITE 1

//
//Command register bits
//

#define MAX22200_N8BITS 0
#define MAX22200_A_BNK   1
#define MAX22200_RFU     5
#define MAX22200_RW      7

#define MAX22200_COMMAND(n8, addr, rw) (\
    (_BV(MAX22200_N8BITS)&n8) | (addr<<MAX22200_A_BNK) | (_BV(MAX22200_RW)&rw)\
)

//
//Register addresses
//

#define MAX22200_STATUS  0x00
#define MAX22200_CFG_CH_1 0x01
#define MAX22200_CFG_CH_2 0x02
#define MAX22200_CFG_CH_3 0x03
#define MAX22200_CFG_CH_4 0x04
#define MAX22200_CFG_CH_5 0x05
#define MAX22200_CFG_CH_6 0x06
#define MAX22200_CFG_CH_7 0x07
#define MAX22200_CFG_CH_8 0x08
#define MAX22200_FAULT   0x09
#define MAX22200_CFG_DPM 0x0A

//
//Channel config values
//

#define MAX22200_DEFAULT_CONFIG  0x00
#define MAX22200_PARALLEL_CONFIG 0x01
#define MAX22200_FULL_BRIDGE     0x10

//
//Status register bits
//

#define MAX22200_ACTIVE  0
#define MAX22200_UVM     1
#define MAX22200_COMER   2
#define MAX22200_DPM     3
#define MAX22200_HHF     4
#define MAX22200_OLF     5
#define MAX22200_OCP     6
#define MAX22200_OVT     7

#define MAX22200_CM10    8
#define MAX22200_CM32    10
#define MAX22200_CM54    12
#define MAX22200_CM76    14

#define MAX22200_FREQM   16
#define MAX22200_M_UVM   17
#define MAX22200_M_COMER 18
#define MAX22200_M_DPM   19
#define MAX22200_M_HHF   20
#define MAX22200_M_OLF   21
#define MAX22200_M_OCP   22
#define MAX22200_M_OVT   23

#define MAX22200_ONCH    24

//
//CFG_CH register bits
//

#define MAX22200_HHF_EN    0
#define MAX22200_DPM_EN    1
#define MAX22200_OL_EN     2
#define MAX22200_SRC       3
#define MAX22200_FREQ_CFG  4
#define MAX22200_HSnLS     6
#define MAX22200_VDRnCDR   7
#define MAX22200_HIT_T     8
#define MAX22200_HIT      16
#define MAX22200_TRIGnSPI 23
#define MAX22200_HOLD     24
#define MAX22200_HFS      31

//
//CFG_DPM register bits
//

#define MAX22200_DPM_IPTH    0
#define MAX22200_DPM_TDEB    4
#define MAX22200_DPM_ISTART  8

#endif /* INC_MAX22200_REGISTERS_H_ */
