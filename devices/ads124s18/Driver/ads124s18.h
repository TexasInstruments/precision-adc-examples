/**
 * @file ads124s18.h
 * @brief ADS124S18 precision ADC driver interface and register management.
 *
 * This header provides the main driver interface for the Texas Instruments
 * ADS124S18 precision analog-to-digital converter. The driver uses a shadow
 * register architecture to maintain a software copy of all ADC registers for
 * efficient multi-page access.
 *
 * Key features:
 * - Multi-page register map management (25 pages total)
 * - Single-channel and multi-channel (sequencer) conversion modes
 * - SPI communication with optional CRC validation
 * - Integrated status and error checking
 * - Offset and gain calibration support
 *
 * Register Map Organization:
 * - Page 0: General settings (device control, status, configuration)
 * - Pages 1-24: Individual sequencer steps (measurement configurations)
 *
 * Typical Usage:
 * 1. resetDevice() - Initialize ADC
 * 2. writeSingleRegister() / readSingleRegister() - Configure settings
 * 3. startAdcConversion() - Begin measurements
 * 4. readData() - Retrieve conversion results
 * 5. checkStatus() - Monitor for errors
 *
 * @copyright Copyright (C) 2024-2026 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS124S18_H_
#define ADS124S18_H_

#include <stdint.h>
#include <stdbool.h>

#include "Driver/ads124s18_pages.h"
#include "Driver/ads124s18_page0.h"
#include "Driver/ads124s18_page1.h"



#define NUM_GENERAL_REGISTERS                           ((uint8_t) 64)
/** Maximum register address or address of the last register in the regmap */
#define MAX_GENERAL_REGISTER_ADDRESS                    ((uint8_t) 0x3F)


#define NUM_STEP_REGISTERS                           ((uint8_t) 24)
/** Maximum register address or address of the last register in the regmap */
#define MAX_STEP_REGISTER_ADDRESS                    ((uint8_t) 0x10)


//**********************************************************************************
//
// Typedefs
//
//**********************************************************************************

/**
 * @typedef STATUS_IO
 * @brief ADC status and error flag container.
 *
 * Holds the complete status snapshot from the ADC including summary
 * error flags and detailed status registers for diagnostic purposes.
 */
typedef struct {
    uint8_t STATUS_MSB;        //!< Summary status register (MSB) - indicates presence of errors
    uint8_t STATUS_LSB;        //!< Summary status register (LSB) - indicates SPI/register errors
    uint8_t ADC_STATUS;        //!< ADC-specific status (supply, reference, conversions)
    uint8_t SUPPLY_STATUS;     //!< Supply voltage monitoring status
    uint8_t DIGITAL_STATUS;    //!< Digital interface status
    bool errorPresent;         //!< Flag indicating any error detected in status registers
} STATUS_IO;

/**
 * @typedef ADC_IO
 * @brief ADC conversion result container.
 *
 * Holds a single conversion result along with associated metadata
 * including status and CRC validation information.
 */
typedef struct {
    uint32_t    ADC_reading;   //!< 24-bit or 16-bit conversion result (format depends on resolution)
    uint8_t     Register_data; //!< Raw register value (for register reads)
    uint8_t    STATUS_MSB;     //!< STATUS_MSB from conversion (if STATUS_ENABLED)
    uint8_t    STATUS_LSB;     //!< STATUS_LSB from conversion (if STATUS_ENABLED)
    bool        CRC_Valid;     //!< CRC validation result (true if CRC check passed, if SPI_CRC_ENABLED)
} ADC_IO;

/**
 * @typedef REG_IO
 * @brief Register read/write result container.
 *
 * Holds the result of a register operation including the register value,
 * status flags, and CRC validation status.
 */
typedef struct {
    uint8_t STATUS_MSB;        //!< STATUS_MSB from transaction (if STATUS_ENABLED)
    uint8_t STATUS_LSB;        //!< STATUS_LSB from transaction (if STATUS_ENABLED)
    uint8_t Register_data;     //!< Register value read or write confirmation
    bool    CRC_Valid;         //!< CRC validation result (if SPI_CRC_ENABLED)
} REG_IO;

//**********************************************************************************
//
// Function Prototypes
//
//**********************************************************************************

/**
 * @brief Read shadow register value without SPI transaction.
 * @param page Register page (0=GENERAL_SETTINGS, 1-24=STEP_X)
 * @param address Register address within page
 * @return Shadow register value (0 if not yet written)
 *
 * Retrieves the software shadow copy of a register. Does not perform
 * actual SPI communication - use for quick access to known values.
 */
uint8_t getRegisterValue(uint8_t page, uint8_t address);

/**
 * @brief Initialize ADC with configuration from ADS124S18_RegisterDetails.h.
 *
 * Loads all sequencer step configurations and clears status flags.
 * Must be called after resetDevice(). Initializes shadow register map
 * from the centralized RegisterDetails file.
 */
void initADC(void);

/**
 * @brief Reset ADC to default state via SPI command.
 * @return Always returns 0
 *
 * Issues SPI reset command, waits for reset completion, and restores
 * all shadow registers to power-on defaults. Required before first use.
 * Takes ~500µs to complete.
 */
bool resetDevice(void);

/**
 * @brief Enter low-power standby mode.
 *
 * Sets the PWDN bit in ADC_CFG register to enable power-down mode.
 * ADC stops conversions and minimizes power consumption. 
 */
void powerDownMode(void);

/**
 * @brief Exit low-power mode and resume active operation.
 *
 * Clears the PWDN bit to return ADC to active mode. ADC is ready to
 * convert after this call, but startAdcConversion() must be called
 * to begin measurements.
 */
void activeMode(void);

/**
 * @brief Start ADC conversions (continuous or sequencer mode).
 *
 * Sets the START bit in CONVERSION_CTRL register. ADC begins converting
 * immediately in the configured mode (continuous or sequencer).
 * DRDY signal indicates when data is ready to read.
 */
void startAdcConversion(void);

/**
 * @brief Stop ADC conversions.
 *
 * Sets the STOP bit in CONVERSION_CTRL register. Halts ongoing conversions
 * and prepares ADC for standby. Current conversion (if in progress) completes.
 */
void stopAdcConversion(void);

/**
 * @brief Read a register from specified page via SPI.
 * @param page Register page (GENERAL_SETTINGS or STEP_0-23)
 * @param address Register address within page
 * @return REG_IO struct with register value, status, and CRC
 *
 * Performs 2-frame SPI operation to read register. Switches pages as needed.
 * Updates shadow register and returns result with optional STATUS and CRC info.
 */
REG_IO readSingleRegister(uint8_t page, uint8_t address);

/**
 * @brief Write a register to specified page via SPI.
 * @param page Register page (GENERAL_SETTINGS or STEP_0-23)
 * @param address Register address within page
 * @param data 8-bit value to write
 * @return REG_IO struct with status and CRC
 *
 * Performs SPI write operation. Automatically switches pages if needed.
 * Updates shadow register and returns result with optional STATUS and CRC info.
 */
REG_IO writeSingleRegister(uint8_t page, uint8_t address, uint8_t data);

/**
 * @brief Check ADC status registers for errors.
 * @return STATUS_IO struct with all status flags and error state
 *
 * Reads STATUS_MSB and STATUS_LSB summary registers. If errors detected,
 * also reads detailed status registers (SUPPLY, ADC, DIGITAL). Use this
 * before and after conversions to catch communication or hardware errors.
 */
STATUS_IO checkStatus(void);

/**
 * @brief Read latest ADC conversion result.
 * @return ADC_IO struct with 24/16-bit result, status, and CRC
 *
 * Waits for DRDY interrupt (1s timeout), reads 24 SPI bits from ADC,
 * and returns conversion result. Result format depends on RESOLUTION_IS_24_BIT
 * macro. Includes optional STATUS bytes and CRC if enabled.
 */
ADC_IO readData(void);

/**
 * @brief Sign-extend 24-bit ADC data to 32-bit signed integer.
 * @param dataBytes 3-byte array containing [MSB, MidSB, LSB]
 * @return 32-bit signed integer result
 *
 * Converts 24-bit two's complement to 32-bit signed integer.
 * Handles negative values correctly for differential measurements.
 */
int32_t signExtend(const uint8_t dataBytes[]);

/**
 * @brief Enable/disable register map CRC protection.
 * @param enable true=enable CRC checking, false=disable
 *
 * When enabled, computes CRC over all memory pages and validates
 * on each register access. Detects memory corruption.
 */
void enableRegisterMapCrc(bool enable);

/**
 * @brief Validate SPI CRC of last transaction output.
 * @return true if CRC check passed, false if mismatch detected
 *
 * Checks the CRC byte returned in the last SPI transaction.
 */
bool isValidCrcOut(void);
 
//**********************************************************************************
//
// Variables
//
//**********************************************************************************

 STATUS_IO status;
 ADC_IO regValue;
 extern bool g_Status;
 extern bool g_CRC;
 
//**********************************************************************************
//
// Register macros
//
//**********************************************************************************


#define MASKED_REG_DATA(page, addr, mask)     (getRegisterValue(page, addr) & (mask))

/** Returns true if STATUS bit is set in shadow register */
#define STATUS_ENABLED          ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DIGITAL_CFG_ADDRESS, STATUS_EN_MASK))

/** Returns true if SPI_CRC bit is set in shadow register */
#define SPI_CRC_ENABLED         ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, MONITOR_CFG_ADDRESS, SPI_CRC_EN_MASK))

#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DEVICE_ID_ADDRESS, DEVICE_IS_16_BIT_MASK))

#define RESOLUTION_IS_24_BIT   ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DEVICE_ID_ADDRESS, DEVICE_IS_24_BIT_MASK))

#define CHANNEL_COUNT_IS_18   ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DEVICE_ID_ADDRESS,DEVICE_ID_CH_CNT_18_MASK ))

#define CHANNEL_COUNT_IS_08   ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DEVICE_ID_ADDRESS,DEVICE_ID_CH_CNT_08_MASK ))

#define UNIPOLAR_DATA_ENABLED ((bool) MASKED_REG_DATA(registerMap[GENERAL_SETTINGS]0x3F), STEPX_DIGITAL_CFG_ADDRESS,STEPX_CODING_MASK ))

#endif /* ADS124S18_H_ */
