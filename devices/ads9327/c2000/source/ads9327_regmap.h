/**
 *
 * \copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ADS9327_REGMAP_H_
#define ADS9327_REGMAP_H_

#ifdef __cplusplus

extern "C" {
#endif


#include <stdint.h>


//**********************************************************************************
//
// Register definitions - Register Bank 0
//
//**********************************************************************************

//

/* Register 0x01 (REGISTER_01H) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                               REG_READ_ADDR[7:0]                                                                                              |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       RESERVED[5:0]                                                                       |          RESET          |         DATA_SEL        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

/* REGISTER_01H register */
#define REGISTER_01H_ADDRESS                                ((uint8_t) 0x01)
#define REGISTER_01H_DEFAULT                                ((uint16_t) 0x0000)

/* REG_READ_ADDR field */
#define REGISTER_01H_REG_READ_ADDR_MASK                     ((uint16_t) 0xFF00)
#define REGISTER_01H_REG_READ_ADDR_BITOFFSET                (8)

/* RESET field */
#define REGISTER_01H_RESET_MASK                             ((uint16_t) 0x0002)
#define REGISTER_01H_RESET_BITOFFSET                        (1)
#define REGISTER_01H_RESET_NORMAL                           ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_01H_RESET_RESET                            ((uint16_t) 0x0002)

/* DATA_SEL field */
#define REGISTER_01H_DATA_SEL_MASK                          ((uint16_t) 0x0001)
#define REGISTER_01H_DATA_SEL_BITOFFSET                     (0)
#define REGISTER_01H_DATA_SEL_ADCOUTPUT                     ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_01H_DATA_SEL_REGDATA                       ((uint16_t) 0x0001)


/* Register 0x02 (REGISTER_02H) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                 RESERVED[11:4]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                                               RESERVED[3:0]                                             |                                           REG_BANK_SEL[3:0]                                           |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_02H register */
#define REGISTER_02H_ADDRESS                                ((uint8_t) 0x02)
#define REGISTER_02H_DEFAULT                                ((uint16_t) 0x0000)

/* REG_BANK_SEL field */
#define REGISTER_02H_REG_BANK_SEL_MASK                      ((uint16_t) 0x000F)
#define REGISTER_02H_REG_BANK_SEL_BITOFFSET                 (0)
#define REGISTER_02H_REG_BANK_SEL_REGBANK0                  ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_02H_REG_BANK_SEL_REGBANK1                  ((uint16_t) 0x0002)


/* Register 0xFE (REGISTER_FEH) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                 REG_LOCK[15:8]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                                                                                                   REG_LOCK[7:0]
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_FEH register */
#define REGISTER_FEH_ADDRESS                                ((uint8_t) 0xFE)
#define REGISTER_FEH_DEFAULT                                ((uint16_t) 0x0000)

/* REG_LOCK field */
#define REGISTER_FEH_REG_LOCK_MASK                          ((uint16_t) 0xFFFF)
#define REGISTER_FEH_REG_LOCK_BITOFFSET                     (0)
#define REGISTER_FEH_REG_LOCK_UNLOCKKEY0                    ((uint16_t) 0xB38F)
#define REGISTER_FEH_REG_LOCK_UNLOCKKEY1                    ((uint16_t) 0xABCD)
#define REGISTER_FEH_REG_LOCK_BANK1_SEL                     ((uint16_t) 0x0002)
#define REGISTER_FEH_REG_LOCK_LOCKKEY                       ((uint16_t) 0x1234)


//**********************************************************************************
//
// Register definitions - Register Bank 1
//
//**********************************************************************************


/* Register 0x08 (REGISTER_08H) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                 RESERVED[11:4]
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                               RESERVED[3:0]                                             |                    PDN_CH[1:0]                    |         RESERVED        |         PDN_CTL         |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

/* REGISTER_08H register */
#define REGISTER_08H_ADDRESS                                ((uint8_t) 0x08)
#define REGISTER_08H_DEFAULT                                ((uint16_t) 0x0000)

/* PDN_CH field */
#define REGISTER_08H_PDN_CH_MASK                            ((uint16_t) 0x000C)
#define REGISTER_08H_PDN_CH_BITOFFSET                       (2)
#define REGISTER_08H_PDN_CH_NORMAL                          ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_08H_PDN_CH_CHA_PDN                         ((uint16_t) 0x0004)
#define REGISTER_08H_PDN_CH_CHB_PDN                         ((uint16_t) 0x0008)
#define REGISTER_08H_PDN_CH_CHA_CHB_PDN                     ((uint16_t) 0x000C)

/* PDN_CTL field */
#define REGISTER_08H_PDN_CTL_MASK                           ((uint16_t) 0x0001)
#define REGISTER_08H_PDN_CTL_BITOFFSET                      (0)
#define REGISTER_08H_PDN_CTL_NORMAL                         ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_08H_PDN_CTL_DEVICE_PDN                     ((uint16_t) 0x0001)


/* Register 0x09 (REGISTER_09H) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                          RESERVED[4:0]                                                          |       LATENCY_MODE      |                   RESERVED[2:1]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*        RESERVED[0:0]      |                             NUM_DATA_LANES[2:0]                             |                                RESERVED[2:0]                                |        DAISY_CLK        |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_09H register */
#define REGISTER_09H_ADDRESS                                ((uint8_t) 0x09)
#define REGISTER_09H_DEFAULT                                ((uint16_t) 0x0000)

/* LATENCY_MODE field */
#define REGISTER_09H_LATENCY_MODE_MASK                      ((uint16_t) 0x0400)
#define REGISTER_09H_LATENCY_MODE_BITOFFSET                 (10)
#define REGISTER_09H_LATENCY_MODE_0_CYCLE                   ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_09H_LATENCY_MODE_1_CYCLE                   ((uint16_t) 0x0400)

/* NUM_DATA_LANES field */
#define REGISTER_09H_NUM_DATA_LANES_MASK                    ((uint16_t) 0x0070)
#define REGISTER_09H_NUM_DATA_LANES_BITOFFSET               (4)
#define REGISTER_09H_NUM_DATA_LANES_4_LANES                 ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_09H_NUM_DATA_LANES_2_LANES                 ((uint16_t) 0x0050)
#define REGISTER_09H_NUM_DATA_LANES_1_LANES                 ((uint16_t) 0x0060)

/* DAISY_CLK field */
#define REGISTER_09H_DAISY_CLK_MASK                         ((uint16_t) 0x0001)
#define REGISTER_09H_DAISY_CLK_BITOFFSET                    (0)
#define REGISTER_09H_DAISY_CLK_FALSE                        ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_09H_DAISY_CLK_TRUE                         ((uint16_t) 0x0001)


/* Register 0x0A (REGISTER_0AH) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                 RESERVED[11:4]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                                               RESERVED[3:0]                                             |       DIG_DELAY_EN      |                             DRIVE_STRENGTH[2:0]                             |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_0AH register */
#define REGISTER_0AH_ADDRESS                                ((uint8_t) 0x0A)
#define REGISTER_0AH_DEFAULT                                ((uint16_t) 0x0000)

/* DIG_DELAY_EN field */
#define REGISTER_0AH_DIG_DELAY_EN_MASK                      ((uint16_t) 0x0008)
#define REGISTER_0AH_DIG_DELAY_EN_BITOFFSET                 (3)
#define REGISTER_0AH_DIG_DELAY_EN_FALSE                     ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0AH_DIG_DELAY_EN_TRUE                      ((uint16_t) 0x0008)

/* DRIVE_STRENGTH field */
#define REGISTER_0AH_DRIVE_STRENGTH_MASK                    ((uint16_t) 0x0007)
#define REGISTER_0AH_DRIVE_STRENGTH_BITOFFSET               (0)
#define REGISTER_0AH_DRIVE_STRENGTH_NORMAL                  ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0AH_DRIVE_STRENGTH_0_5X                    ((uint16_t) 0x0005)
#define REGISTER_0AH_DRIVE_STRENGTH_2_0X                    ((uint16_t) 0x0006)
#define REGISTER_0AH_DRIVE_STRENGTH_1_5X                    ((uint16_t) 0x0007)


/* Register 0x0B (REGISTER_0BH) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                             RESERVED[3:0]                                             |                              DIG_DELAY_D3[2:0]                              |    DIG_DELAY_D2[2:2]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                   DIG_DELAY_D2[1:0]                 |                              DIG_DELAY_D1[2:0]                              |                              DIG_DELAY_D0[2:0]                              |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_0BH register */
#define REGISTER_0BH_ADDRESS                                ((uint8_t) 0x0B)
#define REGISTER_0BH_DEFAULT                                ((uint16_t) 0x0000)

/* DIG_DELAY_D3 field */
#define REGISTER_0BH_DIG_DELAY_D3_MASK                      ((uint16_t) 0x0E00)
#define REGISTER_0BH_DIG_DELAY_D3_BITOFFSET                 (9)
#define REGISTER_0BH_DIG_DELAY_D3_0_NS                      ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0BH_DIG_DELAY_D3_1_NS                      ((uint16_t) 0x0200)
#define REGISTER_0BH_DIG_DELAY_D3_2_NS                      ((uint16_t) 0x0400)
#define REGISTER_0BH_DIG_DELAY_D3_3_NS                      ((uint16_t) 0x0600)
#define REGISTER_0BH_DIG_DELAY_D3_4_NS                      ((uint16_t) 0x0800)
#define REGISTER_0BH_DIG_DELAY_D3_5_NS                      ((uint16_t) 0x0A00)

/* DIG_DELAY_D2 field */
#define REGISTER_0BH_DIG_DELAY_D2_MASK                      ((uint16_t) 0x01C0)
#define REGISTER_0BH_DIG_DELAY_D2_BITOFFSET                 (6)
#define REGISTER_0BH_DIG_DELAY_D2_0_NS                      ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0BH_DIG_DELAY_D2_1_NS                      ((uint16_t) 0x0040)
#define REGISTER_0BH_DIG_DELAY_D2_2_NS                      ((uint16_t) 0x0080)
#define REGISTER_0BH_DIG_DELAY_D2_3_NS                      ((uint16_t) 0x00C0)
#define REGISTER_0BH_DIG_DELAY_D2_4_NS                      ((uint16_t) 0x0100)
#define REGISTER_0BH_DIG_DELAY_D2_5_NS                      ((uint16_t) 0x0140)

/* DIG_DELAY_D1 field */
#define REGISTER_0BH_DIG_DELAY_D1_MASK                      ((uint16_t) 0x0038)
#define REGISTER_0BH_DIG_DELAY_D1_BITOFFSET                 (3)
#define REGISTER_0BH_DIG_DELAY_D1_0_NS                      ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0BH_DIG_DELAY_D1_1_NS                      ((uint16_t) 0x0008)
#define REGISTER_0BH_DIG_DELAY_D1_2_NS                      ((uint16_t) 0x0010)
#define REGISTER_0BH_DIG_DELAY_D1_3_NS                      ((uint16_t) 0x0018)
#define REGISTER_0BH_DIG_DELAY_D1_4_NS                      ((uint16_t) 0x0020)
#define REGISTER_0BH_DIG_DELAY_D1_5_NS                      ((uint16_t) 0x0028)

/* DIG_DELAY_D0 field */
#define REGISTER_0BH_DIG_DELAY_D0_MASK                      ((uint16_t) 0x0007)
#define REGISTER_0BH_DIG_DELAY_D0_BITOFFSET                 (0)
#define REGISTER_0BH_DIG_DELAY_D0_0_NS                      ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0BH_DIG_DELAY_D0_1_NS                      ((uint16_t) 0x0001)
#define REGISTER_0BH_DIG_DELAY_D0_2_NS                      ((uint16_t) 0x0002)
#define REGISTER_0BH_DIG_DELAY_D0_3_NS                      ((uint16_t) 0x0003)
#define REGISTER_0BH_DIG_DELAY_D0_4_NS                      ((uint16_t) 0x0004)
#define REGISTER_0BH_DIG_DELAY_D0_5_NS                      ((uint16_t) 0x0005)


/* Register 0x0C (REGISTER_0CH) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                       RESERVED[5:0]                                                                       |                    PD_REF[1:0]                    |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |         RESERVED        |                                 CLK_PWR[2:0]                                |                                             RESERVED[3:0]                                             |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_0CH register */
#define REGISTER_0CH_ADDRESS                                ((uint8_t) 0x0C)
#define REGISTER_0CH_DEFAULT                                ((uint16_t) 0x0000)

/* PD_REF field */
#define REGISTER_0CH_PD_REF_MASK                            ((uint16_t) 0x0300)
#define REGISTER_0CH_PD_REF_BITOFFSET                       (8)
#define REGISTER_0CH_PD_REF_REF_SEL_ACTIVE                  ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0CH_PD_REF_EXTERNAL_REF_ACTIVE             ((uint16_t) 0x0100)
#define REGISTER_0CH_PD_REF_INTERNAL_REF_ACTIVE             ((uint16_t) 0x0200)

/* CLK_PWR field */
#define REGISTER_0CH_CLK_PWR_MASK                           ((uint16_t) 0x0070)
#define REGISTER_0CH_CLK_PWR_BITOFFSET                      (4)
#define REGISTER_0CH_CLK_PWR_IOVDD_LOGIC                    ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0CH_CLK_PWR_VDD_1V8_LOGIC                  ((uint16_t) 0x0050)


/* Register 0x0D (REGISTER_0DH) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                    RESERVED[6:0]                                                                                    |       DATA_FORMAT       |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |                                             SAVG_MODE[3:0]                                            |                   RESERVED[1:0]                   |         AVG_SYNC        |         SAVG_EN         |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_0DH register */
#define REGISTER_0DH_ADDRESS                                ((uint8_t) 0x0D)
#define REGISTER_0DH_DEFAULT                                ((uint16_t) 0x0000)

/* DATA_FORMAT field */
#define REGISTER_0DH_DATA_FORMAT_MASK                       ((uint16_t) 0x0100)
#define REGISTER_0DH_DATA_FORMAT_BITOFFSET                  (8)
#define REGISTER_0DH_DATA_FORMAT_TWOS_COMPLEMENT            ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0DH_DATA_FORMAT_STRAIGHT_BINARY            ((uint16_t) 0x0100)

/* SAVG_MODE field */
#define REGISTER_0DH_SAVG_MODE_MASK                         ((uint16_t) 0x00F0)
#define REGISTER_0DH_SAVG_MODE_BITOFFSET                    (4)
#define REGISTER_0DH_SAVG_MODE_AVG_2                        ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0DH_SAVG_MODE_AVG_4                        ((uint16_t) 0x0010)
#define REGISTER_0DH_SAVG_MODE_AVG_8                        ((uint16_t) 0x0020)
#define REGISTER_0DH_SAVG_MODE_AVG_16                       ((uint16_t) 0x0030)
#define REGISTER_0DH_SAVG_MODE_AVG_32                       ((uint16_t) 0x0040)
#define REGISTER_0DH_SAVG_MODE_AVG_64                       ((uint16_t) 0x0050)
#define REGISTER_0DH_SAVG_MODE_AVG_128                      ((uint16_t) 0x0060)

/* AVG_SYNC field */
#define REGISTER_0DH_AVG_SYNC_MASK                          ((uint16_t) 0x0002)
#define REGISTER_0DH_AVG_SYNC_BITOFFSET                     (1)

/* SAVG_EN field */
#define REGISTER_0DH_SAVG_EN_MASK                           ((uint16_t) 0x0001)
#define REGISTER_0DH_SAVG_EN_BITOFFSET                      (0)
#define REGISTER_0DH_SAVG_EN_FALSE                          ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0DH_SAVG_EN_TRUE                           ((uint16_t) 0x0001)


/* Register 0x0F (REGISTER_0FH) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                                 RESERVED[9:2]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                     RESERVED[1:0]                   |                TEST_PATT_INCR[1:0]                |                TEST_PATT_MODE[1:0]                |         RESERVED        |       TEST_PATT_EN      |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_0FH register */
#define REGISTER_0FH_ADDRESS                                ((uint8_t) 0x0F)
#define REGISTER_0FH_DEFAULT                                ((uint16_t) 0x0000)

/* TEST_PATT_INCR field */
#define REGISTER_0FH_TEST_PATT_INCR_MASK                    ((uint16_t) 0x0030)
#define REGISTER_0FH_TEST_PATT_INCR_BITOFFSET               (4)
#define REGISTER_0FH_TEST_PATT_INCR_1024                    ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0FH_TEST_PATT_INCR_2048                    ((uint16_t) 0x0010)
#define REGISTER_0FH_TEST_PATT_INCR_3072                    ((uint16_t) 0x0020)
#define REGISTER_0FH_TEST_PATT_INCR_4096                    ((uint16_t) 0x0030)

/* TEST_PATT_MODE field */
#define REGISTER_0FH_TEST_PATT_MODE_MASK                    ((uint16_t) 0x000C)
#define REGISTER_0FH_TEST_PATT_MODE_BITOFFSET               (2)
#define REGISTER_0FH_TEST_PATT_MODE_TEST_PATT               ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0FH_TEST_PATT_MODE_RAMP_PATT               ((uint16_t) 0x0004)
#define REGISTER_0FH_TEST_PATT_MODE_TOGGLE                  ((uint16_t) 0x0008)

/* TEST_PATT_EN field */
#define REGISTER_0FH_TEST_PATT_EN_MASK                      ((uint16_t) 0x0001)
#define REGISTER_0FH_TEST_PATT_EN_BITOFFSET                 (0)
#define REGISTER_0FH_TEST_PATT_EN_FALSE                     ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_0FH_TEST_PATT_EN_TRUE                      ((uint16_t) 0x0001)


/* Register 0x10 (REGISTER_10H) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                               TEST_PATT_1[15:8]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                                                                                                  TEST_PATT_1[7:0]
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_10H register */
#define REGISTER_10H_ADDRESS                                ((uint8_t) 0x10)
#define REGISTER_10H_DEFAULT                                ((uint16_t) 0x0000)

/* TEST_PATT_1 field */
#define REGISTER_10H_TEST_PATT_1_MASK                       ((uint16_t) 0xFFFF)
#define REGISTER_10H_TEST_PATT_1_BITOFFSET                  (0)


/* Register 0x11 (REGISTER_11H) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                                               TEST_PATT_2[15:8]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*                                                                                                  TEST_PATT_2[7:0]
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_11H register */
#define REGISTER_11H_ADDRESS                                ((uint8_t) 0x11)
#define REGISTER_11H_DEFAULT                                ((uint16_t) 0x0000)

/* TEST_PATT_2 field */
#define REGISTER_11H_TEST_PATT_2_MASK                       ((uint16_t) 0xFFFF)
#define REGISTER_11H_TEST_PATT_2_BITOFFSET                  (0)


/* Register 0x13 (REGISTER_13H) definition
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |          Bit 15         |          Bit 14         |          Bit 13         |          Bit 12         |          Bit 11         |          Bit 10         |          Bit 9          |          Bit 8          |
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* |                                                                       RESERVED[5:0]                                                                       |              CSZ_CONVST_SHORT_EN[2:1]
* |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
* |          Bit 7          |          Bit 6          |          Bit 5          |          Bit 4          |          Bit 3          |          Bit 2          |          Bit 1          |          Bit 0          |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*   CSZ_CONVST_SHORT_EN[0:0]|                                                                                    RESERVED[6:0]                                                                                    |
* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
*/

/* REGISTER_13H register */
#define REGISTER_13H_ADDRESS                                ((uint8_t) 0x13)
#define REGISTER_13H_DEFAULT                                ((uint16_t) 0x0000)

/* CSZ_CONVST_SHORT_EN field */
#define REGISTER_13H_CSZ_CONVST_SHORT_EN_MASK               ((uint16_t) 0x0380)
#define REGISTER_13H_CSZ_CONVST_SHORT_EN_BITOFFSET          (7)
#define REGISTER_13H_CSZ_CONVST_SHORT_EN_NORMAL             ((uint16_t) 0x0000)    // DEFAULT
#define REGISTER_13H_CSZ_CONVST_SHORT_EN_CS_CONVST_SHORT    ((uint16_t) 0x0280)


#endif /* ADS9327_REGMAP_H_ */
