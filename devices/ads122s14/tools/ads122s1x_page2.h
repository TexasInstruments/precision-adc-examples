/*
 * @file ads122s1x_page2.h
 *
 * @brief ADS122S1x Descriptor
 *
 * @copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#ifndef ADS122S1X_PAGE2_H_
#define ADS122S1X_PAGE2_H_

#include <stdint.h>


//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************



//**********************************************************************************
//
// Device commands
//
//**********************************************************************************



//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (CTRL_OREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |       PROG      |       LOCK      |                                      RESERVED[4:0]                                      |       ACQ       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CTRL_OREG register */
    #define CTRL_OREG_ADDRESS                                   ((uint8_t) 0x00)
    #define CTRL_OREG_DEFAULT                                   ((uint8_t) 0x00)

    /* PROG field */
    #define CTRL_OREG_PROG_MASK                                 ((uint8_t) 0x80)
    #define CTRL_OREG_PROG_BITOFFSET                            (7)
    #define CTRL_OREG_PROG_NOACTION                             ((uint8_t) 0x00)    // DEFAULT
    #define CTRL_OREG_PROG_1PROGRAMTHESELECTEDWORD              ((uint8_t) 0x80)

    /* LOCK field */
    #define CTRL_OREG_LOCK_MASK                                 ((uint8_t) 0x40)
    #define CTRL_OREG_LOCK_BITOFFSET                            (6)
    #define CTRL_OREG_LOCK_DISABLED                             ((uint8_t) 0x00)    // DEFAULT
    #define CTRL_OREG_LOCK_PROGRAMLOCKWORDADDR70WHENPROGISHIGH  ((uint8_t) 0x40)

    /* ACQ field */
    #define CTRL_OREG_ACQ_MASK                                  ((uint8_t) 0x01)
    #define CTRL_OREG_ACQ_BITOFFSET                             (0)
    #define CTRL_OREG_ACQ_NOACTION                              ((uint8_t) 0x00)    // DEFAULT
    #define CTRL_OREG_ACQ_WRITE1OTPARRAYREADLOADEDINTOOTPVALUEREGS ((uint8_t) 0x01)


/* Register 0x01 (ADDR_OREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                   ADDR[7:0]                                                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ADDR_OREG register */
    #define ADDR_OREG_ADDRESS                                   ((uint8_t) 0x01)
    #define ADDR_OREG_DEFAULT                                   ((uint8_t) 0x00)

    /* ADDR field */
    #define ADDR_OREG_ADDR_MASK                                 ((uint8_t) 0xFF)
    #define ADDR_OREG_ADDR_BITOFFSET                            (0)


/* Register 0x02 (DATA_OREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                 PROG_DATA[7:0]                                                                |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA_OREG register */
    #define DATA_OREG_ADDRESS                                   ((uint8_t) 0x02)
    #define DATA_OREG_DEFAULT                                   ((uint8_t) 0x00)

    /* PROG_DATA field */
    #define DATA_OREG_PROG_DATA_MASK                            ((uint8_t) 0xFF)
    #define DATA_OREG_PROG_DATA_BITOFFSET                       (0)


/* Register 0x03 (DFT_OREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |                    RESERVED[2:0]                    |    IREF_TEST    |   IREF_EXT_EN   |     RESERVED    |            MARGIN[1:0]            |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DFT_OREG register */
    #define DFT_OREG_ADDRESS                                    ((uint8_t) 0x03)
    #define DFT_OREG_DEFAULT                                    ((uint8_t) 0x00)

    /* IREF_TEST field */
    #define DFT_OREG_IREF_TEST_MASK                             ((uint8_t) 0x10)
    #define DFT_OREG_IREF_TEST_BITOFFSET                        (4)
    #define DFT_OREG_IREF_TEST_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define DFT_OREG_IREF_TEST_INTERNALOTPREFERENCECURRENTOUTPUTTOPINSDODRDYN ((uint8_t) 0x10)

    /* IREF_EXT_EN field */
    #define DFT_OREG_IREF_EXT_EN_MASK                           ((uint8_t) 0x08)
    #define DFT_OREG_IREF_EXT_EN_BITOFFSET                      (3)
    #define DFT_OREG_IREF_EXT_EN_DISABLED                       ((uint8_t) 0x00)    // DEFAULT
    #define DFT_OREG_IREF_EXT_EN_EXTERNALOTPREFERENCECURRENTINPUTFROMPINDRDYN ((uint8_t) 0x08)

    /* MARGIN field */
    #define DFT_OREG_MARGIN_MASK                                ((uint8_t) 0x03)
    #define DFT_OREG_MARGIN_BITOFFSET                           (0)
    #define DFT_OREG_MARGIN_1                                   ((uint8_t) 0x00)    // DEFAULT
    #define DFT_OREG_MARGIN_0                                   ((uint8_t) 0x01)
    #define DFT_OREG_MARGIN_6                                   ((uint8_t) 0x02)
    #define DFT_OREG_MARGIN_8                                   ((uint8_t) 0x03)


/* Register 0x04 (LOCK_OREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                               RESERVED[5:0]                                               |   SOME_LOCKED   |    ALL_LOCKED   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOCK_OREG register */
    #define LOCK_OREG_ADDRESS                                   ((uint8_t) 0x04)
    #define LOCK_OREG_DEFAULT                                   ((uint8_t) 0x03)

    /* SOME_LOCKED field */
    #define LOCK_OREG_SOME_LOCKED_MASK                          ((uint8_t) 0x02)
    #define LOCK_OREG_SOME_LOCKED_BITOFFSET                     (1)
    #define LOCK_OREG_SOME_LOCKED_NOROWLOCKED                   ((uint8_t) 0x00)
    #define LOCK_OREG_SOME_LOCKED_ATLEASTONEROWLOCKED           ((uint8_t) 0x02)    // DEFAULT

    /* ALL_LOCKED field */
    #define LOCK_OREG_ALL_LOCKED_MASK                           ((uint8_t) 0x01)
    #define LOCK_OREG_ALL_LOCKED_BITOFFSET                      (0)
    #define LOCK_OREG_ALL_LOCKED_NOTALLROWSLOCKED               ((uint8_t) 0x00)
    #define LOCK_OREG_ALL_LOCKED_ALLROWSLOCKED                  ((uint8_t) 0x01)    // DEFAULT


/* Register 0x0B (OTP_ACQ_STATUS_OREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                      RESERVED[4:0]                                      | SOME_LOCKED_ACQ |  ALL_LOCKED_ACQ |   OTP_ACQ_DONE  |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OTP_ACQ_STATUS_OREG register */
    #define OTP_ACQ_STATUS_OREG_ADDRESS                         ((uint8_t) 0x0B)
    #define OTP_ACQ_STATUS_OREG_DEFAULT                         ((uint8_t) 0x00)

    /* SOME_LOCKED_ACQ field */
    #define OTP_ACQ_STATUS_OREG_SOME_LOCKED_ACQ_MASK            ((uint8_t) 0x04)
    #define OTP_ACQ_STATUS_OREG_SOME_LOCKED_ACQ_BITOFFSET       (2)
    #define OTP_ACQ_STATUS_OREG_SOME_LOCKED_ACQ_NOROWLOCKED     ((uint8_t) 0x00)    // DEFAULT
    #define OTP_ACQ_STATUS_OREG_SOME_LOCKED_ACQ_ATLEASTONEROWLOCKED ((uint8_t) 0x04)

    /* ALL_LOCKED_ACQ field */
    #define OTP_ACQ_STATUS_OREG_ALL_LOCKED_ACQ_MASK             ((uint8_t) 0x02)
    #define OTP_ACQ_STATUS_OREG_ALL_LOCKED_ACQ_BITOFFSET        (1)
    #define OTP_ACQ_STATUS_OREG_ALL_LOCKED_ACQ_NOTALLROWSLOCKED ((uint8_t) 0x00)    // DEFAULT
    #define OTP_ACQ_STATUS_OREG_ALL_LOCKED_ACQ_ATLEASTONEROWLOCKED ((uint8_t) 0x02)

    /* OTP_ACQ_DONE field */
    #define OTP_ACQ_STATUS_OREG_OTP_ACQ_DONE_MASK               ((uint8_t) 0x01)
    #define OTP_ACQ_STATUS_OREG_OTP_ACQ_DONE_BITOFFSET          (0)
    #define OTP_ACQ_STATUS_OREG_OTP_ACQ_DONE_OTPNOTAQUIRED      ((uint8_t) 0x00)    // DEFAULT
    #define OTP_ACQ_STATUS_OREG_OTP_ACQ_DONE_OTPAQUIRED         ((uint8_t) 0x01)


/* Register 0x3F (PAGE_POINTER) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |      Bit 7      |      Bit 6      |      Bit 5      |      Bit 4      |      Bit 3      |      Bit 2      |      Bit 1      |      Bit 0      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                               PAGE_POINTER[7:0]                                                               |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_POINTER register */
    #define PAGE_POINTER_ADDRESS                                ((uint8_t) 0x3F)
    #define PAGE_POINTER_DEFAULT                                ((uint8_t) 0x00)

    /* PAGE_POINTER field */
    #define PAGE_POINTER_MASK                                   ((uint8_t) 0xFF)
    #define PAGE_POINTER_BITOFFSET                              (0)


#endif /* ADS122S1X_PAGE2_H_ */
