/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ads122s14_ptx_daisychain.h"
#include "system/flash.h"
#include "main.h"
#include "ti_msp_dl_config.h"
#include "system/uart.h"


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>


//****************************************************************************
//
// Internal variables and structures
//
//****************************************************************************

static enum status_enum help_cmd(char *cmd);
static enum status_enum get_cmd(char *cmd);
static enum status_enum stream_cmd(char *cmd);
static enum status_enum rreg_cmd(char *cmd);
static enum status_enum wreg_cmd(char *cmd);
static enum status_enum bridgetimer_cmd(char *cmd);
static enum status_enum bridgectrl_cmd(char *cmd);
static enum status_enum lowpwr_cmd(char *cmd);
static enum status_enum cm_test_cmd(char *cmd);


static void readRegistersDaisyChain(uint8_t address, uint8_t *data);
static void writeRegisterDaisyChain(uint8_t addressADC1, uint8_t dataADC1,
                                    uint8_t addressADC2, uint8_t dataADC2);
static void stopADCconversions(void);
static void startADCconversions(void);
static void init_ADCs(void);
static void readDataDaisyChain(int32_t *res, uint8_t *status);
bool time_calc = false;

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"get", "return one raw reading", get_cmd},
    {"stream", "starts/stops streaming raw readings", stream_cmd},
    {"rreg", "read register on both ADCs - rreg reg", rreg_cmd},
    {"wreg", "write register to ADC1 - wreg 1 reg value\r\n\t\t    - write register to ADC2 - wreg 2 reg value", wreg_cmd},
    {"start", "start conversion", (enum status_enum (*)(char *))startADCconversions},
    {"stop", "stop conversion", (enum status_enum (*)(char *)) stopADCconversions},
    {"reset", "rewrite default EVM adc config", (enum status_enum (*)(char *)) init_ADCs},
    {"bridgectrl", "Enable/disable bridge and bridge status readback - bridgectrl [on|off]", (enum status_enum (*)(char *)) bridgectrl_cmd},
    {"lowpwr", "operate bridge and ADC in duty cycle mode - enter time between power-ups in ms", (enum status_enum (*) (char *)) lowpwr_cmd},
    {"cm_test", "production test for manufacturer", cm_test_cmd},
};

struct ads122s14_config_struct {
    uint8_t reg;
    uint8_t value;
};

/*
 * default configuration ADC1 will be connected to bridge
 * ADC2 will be RTD input
 */
#ifndef ADC_REV_E1
volatile struct ads122s14_config_struct gADSDefault[2][9] = {
    { /* ADC 1 */
     {DEVICE_CFG_ADDRESS,       DEVICE_CFG_CONV_MODE_CONT},
     {DATA_RATE_CFG_ADDRESS,    DATA_RATE_CFG_FLTR_OSR_20_SPS},         // 20 sps
     {MUX_CFG_ADDRESS,          MUX_CFG_AINP_AIN2|MUX_CFG_AINN_AIN3},   // set mux to AIN2-AIN3
     {GAIN_CFG_ADDRESS,         GAIN_CFG_GAIN_1},                       // set gain to 1
     {REFERENCE_CFG_ADDRESS,    REFERENCE_CFG_REFP_BUF_EN_ENABLED |
                                    REFERENCE_CFG_REFN_BUF_EN_DISABLED|
                                    REFERENCE_CFG_REF_SEL_EXTERNAL},        // set external reference and enable positive reference buffer
     {DIGITAL_CFG_ADDRESS,      DIGITAL_CFG_STATUS_EN_ENABLED},  
     {IDAC_MUX_CFG_ADDRESS,     IDAC_MUX_CFG_IUNIT_10UA|
                                    IDAC_MUX_CFG_I1MUX_AIN0},               // set IDAC1 to AIN0 and set units to 10uA    
     {IGNORE_ENTRY,             IGNORE_ENTRY},                          // not used entry, just make sure both dimensions of entry have same length
    },
    { /* ADC 2*/
     {DEVICE_CFG_ADDRESS,       DEVICE_CFG_CONV_MODE_CONT},  
     {DATA_RATE_CFG_ADDRESS,    DATA_RATE_CFG_FLTR_OSR_20_SPS},         // 20 sps
     {MUX_CFG_ADDRESS,          MUX_CFG_AINP_AIN2|MUX_CFG_AINN_AIN3},   // set mux to AIN2-AIN3
     {GAIN_CFG_ADDRESS,         GAIN_CFG_GAIN_1},                       // set gain to 1
     {REFERENCE_CFG_ADDRESS,    REFERENCE_CFG_REFP_BUF_EN_ENABLED |
                                    REFERENCE_CFG_REFN_BUF_EN_DISABLED|
                                    REFERENCE_CFG_REF_SEL_EXTERNAL},        // set external reference and enable positive reference buffer
    {DIGITAL_CFG_ADDRESS,      DIGITAL_CFG_STATUS_EN_ENABLED},
     {IDAC_MUX_CFG_ADDRESS,     IDAC_MUX_CFG_IUNIT_10UA|
                                    IDAC_MUX_CFG_I2MUX_AIN7|
                                    IDAC_MUX_CFG_I1MUX_AIN6},               // set IDAC 2 to AIN7, IDAC1 to AIN6 and sets unit to 10uA
     {IDAC_MAG_CFG_ADDRESS,     IDAC_MAG_CFG_I1MAG_30X},                // enable IDAC1 and set magnitude to 20 x 10uA (0.2mA)
    },
};
#else
volatile struct ads122s14_config_struct gADSDefault[2][9] = {
    {
        /* ADC 1 */
        {MUX_CFG_ADDRESS,       MUX_CFG_AINP_AIN2 | MUX_CFG_AINN_AIN3},  // set mux to AIN2-AIN3
        {REFERENCE_CFG_ADDRESS, REFERENCE_CFG_REFP_BUF_EN_ENABLED |
                                REFERENCE_CFG_REFN_BUF_EN_ENABLED |
                                REFERENCE_CFG_REF_SEL_EXTERNAL},        // set external reference and enable reference buffers
        {GAIN_CFG_ADDRESS,      GAIN_CFG_GAIN_128},                     // set gain to 128
        {IDAC_MUX_CFG_ADDRESS,
                                IDAC_MUX_CFG_IUNIT_10UA |
                                IDAC_MUX_CFG_I1MUX_AIN2},  // set IDAC1 to AIN2 and set units to 10uA
        {DATA_RATE_CFG_ADDRESS, DATA_RATE_CFG_FLTR_OSR_20_SPS},  // 20 sps
        {DEVICE_CFG_ADDRESS,    DEVICE_CFG_CONV_MODE_SINGLE},       // single shot
        {DIGITAL_CFG_ADDRESS,   DIGITAL_CFG_STATUS_EN_ENABLED},
        {IGNORE_ENTRY,          IGNORE_ENTRY},  // not used entry, just make sure both dimensions of entry have same length
    },
    {
        /* ADC 2*/
        {MUX_CFG_ADDRESS,       MUX_CFG_AINP_AIN2 | MUX_CFG_AINN_AIN3},  // set mux to AIN2-AIN3
        {GAIN_CFG_ADDRESS,      GAIN_CFG_GAIN_1},         // set gain to 1
        {REFERENCE_CFG_ADDRESS, REFERENCE_CFG_REFP_BUF_EN_ENABLED |
                                REFERENCE_CFG_REFN_BUF_EN_ENABLED |
                                REFERENCE_CFG_REF_SEL_EXTERNAL},  // set external reference and enable reference buffers
        {IDAC_MUX_CFG_ADDRESS,  IDAC_MUX_CFG_IUNIT_10UA |
                                IDAC_MUX_CFG_I2MUX_AIN7 |
                                IDAC_MUX_CFG_I1MUX_AIN6},  // set IDAC1 to AIN6 and sets unit to 10uA
        {IDAC_MAG_CFG_ADDRESS,  IDAC_MAG_CFG_I1MAG_30X},  // enable IDAC1 and set magnitude to 10 x 10uA (0.1mA)
        {DATA_RATE_CFG_ADDRESS, DATA_RATE_CFG_FLTR_OSR_20_SPS},  // 20 sps
        {DEVICE_CFG_ADDRESS,    DEVICE_CFG_CONV_MODE_SINGLE},       // single shot
        {DIGITAL_CFG_ADDRESS,   DIGITAL_CFG_STATUS_EN_ENABLED},
    },
};
#endif

enum bridge_control_state_enum {on, off, toggle};

struct ads122s14_ptx_chain_config_struct {
  uint8_t config_length;
  uint8_t config_chksum;
  uint32_t bridge_time;
  uint32_t sample_time;
  enum bridge_control_state_enum bridge_state;

};
static struct ads122s14_ptx_chain_config_struct *gADSConfigMem;

enum spi_state_enum { idle, busy };
static volatile enum spi_state_enum gSPIState = idle;
static volatile uint8_t gSPIRxBuf[12];
static volatile uint8_t gSPITxBuf[12];

static int32_t gRawReading[2];
static bool gStreaming = false;
static uint32_t gADCTimeoutTimer=0;
static bool ADCRunning = false;


//****************************************************************************
//
// Function Definitions
//
//****************************************************************************


void SPI_0_INST_IRQHandler(void) { // this will be moved somewhere and get configurable
                                   // when multiple trx cards are available using spi irq
  static char dummy;
  switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
      case DL_SPI_IIDX_DMA_DONE_RX:
          gSPIState = idle;
          DL_GPIO_setPins(GPIO_ADC_PORT, GPIO_ADC_ADC_CS_PIN);
          break;

      default:
          break;
  }
}

/**
 * @brief Prints help information about available commands.
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 */
static enum status_enum help_cmd(char *cmd) {
    int i;
    uart_printf("All commands must start with adc\r\n");
    for (i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        if(commands[i].help[0] != '$'){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
        }
    }
    return STATUS_OK;
}

/**
 * @brief     Prints the current ADC raw reading value as a hexadecimal string.
 *
 * @details  This function simply prints the contents of `_raw_reading` to the UART console and returns immediately.
 *
 * @param[in] cmd        Pointer to an unused command string (this parameter is not used by this function)
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
static enum status_enum get_cmd(char *cmd) {
    uart_printf("0x%x 0x%x\r\n", gRawReading[0], gRawReading[1]);
    return STATUS_OK;
}

/**
 * @brief Toggles gStreaming state.
 *
 * This function flips the `gStreaming` flag when called, turning it on or off.
 * The current value of `gStreaming` is used to stream the ADC readings to UART as well or not
 *
 * @param[in] cmd Unused command parameter; kept for consistency with other functions.
 *
 * @return STATUS_OK regardless of the new gStreaming state.
 */
static enum status_enum stream_cmd(char *cmd) {
    if (gStreaming)
        gStreaming = false;
    else
        gStreaming = true;
    return STATUS_OK;
}

/**
 * @brief Reads and prints register values from a daisy-chained ADS122S14.
 *
 * This function parses the input command string, extracts the target register
 * address in hexadecimal format (0xXX), reads the corresponding value
 * from the daisy-chain of ADCs, and then prints out the register value to the UART.
 *
 * If any error occurs during parsing, an error status is returned.
 *
 * @param[in] cmd The command string containing the register address as "rreg 0xXX".
 *
 * @return status if command was parsed successfully
 *
 * @retval STATUS_OK on success
 * @retval STATUS_CMDERROR on syntax errors
 */
static enum status_enum rreg_cmd(char *cmd){
    uint8_t value[2];
    char *subcmd = strtok(cmd, " ");
    int reg;
    int ret;
    if(!subcmd)
        return STATUS_CMDERROR;
    else{
        ret = sscanf(cmd, "%x", &reg);
        if(ret != 1) return STATUS_CMDERROR;
        readRegistersDaisyChain(reg, value);
        uart_printf("Reg 0x%x, 0x%x 0x%x\r\n", reg, value[0], value[1]);
        return STATUS_OK;
    }
    return STATUS_CMDERROR; /* should never get here */
}

/**
 * @brief Writes values to specified registers in a daisy-chained ADS122S14 ADC.
 *
 * This function parses the input command string, extracts the target ADC
 * addresses and value in hexadecimal format, and then writes out
 * these values to the corresponding registers using a low-level write API.
 *
 * If any error occurs during parsing or execution, an error status is returned.
 *
 * @param[in] cmd The command string containing register addresses and value(s).
 *
 * @return Status indicating success or failure:
 *
 * @retval STATUS_OK on successful execution.
 * @retval STATUS_CMDERROR if there was a problem with parsering the input command.
 */
static enum status_enum wreg_cmd(char *cmd){
    int reg, val, adc;

    if(sscanf(cmd, "%d %x %x", &adc, &reg, &val) == 3){ /* need exactly 3 arguments */
        if(adc == 1){
            writeRegisterDaisyChain(reg, val, IGNORE_ADC, 0);
        }else if(adc == 2){
            writeRegisterDaisyChain(IGNORE_ADC, 0, reg, val);
        }else
            return STATUS_CMDERROR;
        time_calc = false;
        return STATUS_OK;
    }else if(sscanf(cmd, "%x %x", &reg, &val) == 2){
            writeRegisterDaisyChain(reg, val, reg, val);
        time_calc = false;
        return STATUS_OK;
    }
    else
        return STATUS_CMDERROR;
}

/**
 * @brief Handles command line input to read/update bridge timer
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum bridgetimer_cmd(char *cmd){
    enum status_enum ret = STATUS_CMDERROR;
    int cnt;
    uint32_t timer;;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){
        uart_printf("%d\r\n", gADSConfigMem->bridge_time);
        ret = STATUS_OK;
    }else{
        sscanf(subcmd, "%d", &timer);
        gADSConfigMem->bridge_time = timer;
        gADSConfigMem->config_chksum = crc8((void*)(gADSConfigMem)+2, gADSConfigMem->config_length);
        ret = STATUS_OK;
    }
    return ret;
}

/**
 * @brief Handles command line input to read/update bridge control
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum bridgectrl_cmd(char *cmd){
    enum status_enum ret = STATUS_CMDERROR;
    int cnt;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){
        uart_printf("%d\r\n", gADSConfigMem->bridge_state);
        ret = STATUS_OK;
    }else{
        if(!strcasecmp(subcmd, "on")){
            gADSConfigMem->bridge_state = on;
            DL_GPIO_setPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);
        } else if(!strcasecmp(subcmd, "off")){
            gADSConfigMem->bridge_state = off;
            DL_GPIO_clearPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);
           }

        gADSConfigMem->config_chksum = crc8((void*)(gADSConfigMem)+2, gADSConfigMem->config_length);
        ret = STATUS_OK;
    }
    return ret;
}

/**
 * @brief Start a non-blocking write-read SPI transaction.
 *
 * This function initiates an SPI transfer between the internal SPI peripheral
 * and the external chip through the DMA controller. The data to be written is
 * copied from `buf` into the SPI transmit buffer, while the receive data goes
 * directly into the global `gSPIRxBuf`.
 *
 * @param[in] buf The data buffer containing data to be transmitted over SPI.
 * @param[in] len The length of the data in bytes.
 */
static void rw_spi_non_blocking(const char *buf, int len) {
    gSPIState = busy;
    memcpy((void*)gSPITxBuf, buf, len);

    DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t) gSPITxBuf);
    DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID, (uint32_t)(&SPI_0_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, len);

    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&SPI_0_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) gSPIRxBuf);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, len);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

    DL_GPIO_clearPins(GPIO_ADC_PORT, GPIO_ADC_ADC_CS_PIN);
    DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);


}

/** 
 * @brief Timer 
 * Timer used for converting time in us to a delay outside of delay_cycles
 *  
 */
void timer_us(uint32_t time_in_us) {
    // 32 MHz clock = 32 cycles per microsecond
    uint32_t cycles = (time_in_us * 53) / 20;  // can be adjust based on timing needs
    
    while (cycles--) {
        __asm("NOP");  // Prevent compiler optimization
    }
}


/** 
 * @brief Low-power ADC bridge measurement with power cycling
 * 
 * Performs periodic bridge sensor measurements while minimizing power consumption
 * by cycling both ADCs and bridge power between conversions.
 * 
 * @param cmd Command string containing sample interval in ms, or "stop" to halt
 * @return STATUS_OK on success, STATUS_ERR on invalid parameters
 * 
 * Operation:
 * - User specifies sample interval via command argument (e.g., "100" for 100ms)
 * - Uses existing ADC configuration: SPEED_MODE, OSR, DELAY, GLOBAL_CHOP
 * - Validates timing: ensures sample interval > startup + conversion + 10% margin
 * - Streams ADC1/ADC2 readings at specified interval until "stop" command
 * 
 * Power Sequence per Sample interval:
 * 1. Wake ADCs from power-down (~10ms startup)
 * 2. Enable bridge excitation via low-side switch
 * 3. Wait for bridge settling (user-configured DELAY)
 * 4. Perform single-shot conversion
 * 5. Disable bridge and return ADCs to power-down
 * 6. Sleep for remainder of sample interval
 * 
 * Timing Diagram:
 * 
 *     |<-------------------- Sensor Cycle Period (interval_ms) ------------------->|
 *     |                                                                            |
 *     | Startup  | User Delay | Conversion |         Power-Down Sleep              |
 *     |<-------->|<---------->|<---------->|<------------------------------------->|
 *     |          |                         |
 *     |          |<----- Bridge Active --->|
 *     |                                    
 *     ADC Wake                              ADC Sleep
 * 
 * Example Usage:
 *   adc lowpwr           - Show current interval
 *   adc lowpwr 250       - Sample every 250ms
 *   adc lowpwr stop      - Stop streaming
 * 
 * Notes:
 * - Duty cycle displayed shows active time vs total sample period
 * - Minimum interval enforced to prevent >90% duty cycle
 * - 2% timing margin added to account for MCU/ADC clock variations
 * - maximum timer time between power-ups = 16,000ms limited by MCU timer used
 */

static enum status_enum lowpwr_cmd(char *cmd){
    static int32_t interval_ms = 100; // Default time between samples (ms)
    uint8_t value[2];
    int32_t adcResult[2];
    uint8_t adcStatus[2];
    uint8_t speed_mode, osr_reg , delay_reg, status, user_delay_bits;
    uint32_t f_mod_hz, osr, active_time_modclks, user_delay_time_modclks, latency_in_modclks;
    uint32_t start_up_time_cycles, active_time_cycles;
    float start_up_time_ms, active_time_ms, user_delay_time_ms, duty_cycle;
    float start_up_time_us, active_time_us, user_delay_time_us, effective_time_us, min_time_us, interval_us, t_mod_us;
    volatile bool adc_lowpwr_stop = false;

    // Speed mode and latency tables 
    static const struct {
        uint8_t reg_value;
        uint32_t f_mod_hz;
    } speed_modes[] = {
        {0x00, 32000},   // Mode 0: 32 kHz
        {0x01, 256000},  // Mode 1: 256 kHz
        {0x02, 512000},  // Mode 2: 512 kHz
        {0x03, 1024000}  // Mode 3: 1.024 MHz
    };
   
static const struct {
        uint8_t reg_value;
        uint32_t latency[4];
	// latancy table from datasheet 
    } latency_table[] = {
        {0x00, 80, 88, 88, 104},                 // OSR16
        {0x01, 144, 152, 152, 168},              // OSR32
        {0x02, 240, 248, 248, 264},              // OSR128
        {0x03, 368, 376, 376, 392},              // OSR256
        {0x04, 624, 632, 632, 648},              // OSR512
        {0x05, 1136, 1144, 1144, 1160},          // OSR1024
        {0x06, 1416, 10384, 20624, 41120},       // SPS25
        {0x07, 1736, 12944, 25744, 51360}        // SPS20

    };

    // Parse subcommand (interval_ms) confirm valid time entered by user 
    char *subcmd = strtok(cmd, " ");
    if (!subcmd){
         uart_printf("Current sensor cycle period: %u ms\r\n", interval_ms);
        return STATUS_OK;
    }

    if (strcmp(subcmd, "stop") == 0){
        adc_lowpwr_stop = true;
        uart_printf("Stopping ADC Low power bridge cycling stream...\r\n");
        return STATUS_OK;
    }

    uint32_t new_interval_ms;
    if(sscanf(subcmd, "%u", &new_interval_ms) != 1 || new_interval_ms == 0){
            uart_printf("Invalid sensor cycle period: %s\r\n", subcmd);
            return STATUS_ERR;
    }
    interval_ms = new_interval_ms;
    interval_us = interval_ms*1000; // convert interval from ms to us

    /*ADC configuration*/
    // Read Speed mode from DEVICE_CFG_ADDRESS[1:0]
    readRegistersDaisyChain(DEVICE_CFG_ADDRESS, value);
    speed_mode = value[0] & 0x03; // Bits [1:0]
    f_mod_hz = speed_modes[speed_mode].f_mod_hz;

    // Set Single-Shot mode (preserve existing bits)
    writeRegisterDaisyChain(DEVICE_CFG_ADDRESS,
                            value[0] | DEVICE_CFG_CONV_MODE_SINGLE,
                            DEVICE_CFG_ADDRESS, 
                            value[0] | DEVICE_CFG_CONV_MODE_SINGLE);
    
    // Read DATA_RATE register for OSR, Delay bits, and Global Chop 
    readRegistersDaisyChain(DATA_RATE_CFG_ADDRESS, value);

    // Read user ADC delay register bits  
    delay_reg = value[0] & 0xF0; // Bits [7:4]

    // Global Chop
    bool global_chop = value[0] & 0x08; // Bit [3]

    // Isolating OSR value bits
    osr_reg = value[0] & 0x07; // Bits [2:0]

    // tMOD Calculation based on fMOD
    t_mod_us = 1000000.0f / f_mod_hz; // tMOD time in us

    // First Conversion Latency table from datasheet 
    latency_in_modclks = latency_table[osr_reg].latency[speed_mode];

    // ADC delay bit isolations and calculations 
    user_delay_bits = (delay_reg) >> 4; 
    user_delay_time_modclks = (1 << (user_delay_bits - 1));
    user_delay_time_us = t_mod_us * (float)(user_delay_time_modclks);

    // Total time the ADC is active 
    active_time_modclks = latency_in_modclks + user_delay_time_modclks;
    active_time_us = active_time_modclks * t_mod_us * 1.03; // addtional 3% timing error buffer added for internal oscillator tolerance of ADC and MSPMO

    if (global_chop){                       // Global Chop bit check 
        active_time_us *= 2;
    }
    
    // PDWN start up time (5-10ms)
    start_up_time_us = 10000;

    // duty cycle as a percentage of the sample time 
    duty_cycle = ((active_time_us + start_up_time_us) / interval_us) *100.0f;

    min_time_us = 1.111f * (active_time_us + start_up_time_us); // 90% duty cycle check to confirm sample period 
    effective_time_us = interval_us;
    if ((float)interval_us < min_time_us){
        uart_printf("Sensor cycle period too short for settings(min time: %.2fms). Please adjust inputs.\r\n",
        min_time_us /1000.0f);
        return STATUS_ERR;
    }

    // Converison from us to ms for terminal output 
    start_up_time_ms = start_up_time_us / 1000.0f;
    user_delay_time_ms = user_delay_time_us / 1000.0f;
    active_time_ms = active_time_us / 1000.0f; 

    // conversion from time to MCU cycles 
    start_up_time_cycles = start_up_time_us * g_system_clk_frequency_mhz;
    active_time_cycles = active_time_us * g_system_clk_frequency_mhz;

    // Output: Speed Mode | fMOD frequency | OSR setting | Start up delay time | user set ADC delay | ADC Active time | Duty cycle of ADC active vs interval_ms
    uart_printf("Mode: %u | f_MOD: %u Hz | OSR: 0x0%uh | Start up: %.1f ms | Delay: %.4f ms | Conv: %.2f ms | Duty Cycle: %.2f %%\r\n",
                    speed_mode, f_mod_hz, osr_reg, start_up_time_ms, user_delay_time_ms, active_time_ms, duty_cycle);       

    // Sampling loop start terminal output
    uart_printf("ADC Low power bridge cycling streaming every %d ms. Type 'adc lowpwr stop' to halt.\r\n\n", interval_ms);
    float down_time_us = (effective_time_us > active_time_us) ? (effective_time_us - active_time_us - start_up_time_us) : 0.0f;    

    // Loop to stream data every interval_ms while utilizing the powerdown function of the ADC
    while (!adc_lowpwr_stop) {
        if (strcmp(subcmd, "stop") == 0) // break out from loop to stop the stream 
        {      
            adc_lowpwr_stop = true;
            uart_printf("Stopping ADC Low power bridge cycling stream...\r\n");

            // Turn off bridge
            DL_GPIO_clearPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);
            writeRegisterDaisyChain(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_STOP_STOP, 
                                CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_STOP_STOP);
            //init_ADCs(); // reset adc to default settings 
        break;
        }
        // optional to start in PWDN
   
        // Setting the ADCs to idle mode
        writeRegisterDaisyChain(DEVICE_CFG_ADDRESS, DEVICE_CFG_PWDN_NO_PWDN | speed_mode | DEVICE_CFG_CONV_MODE_SINGLE, 
                                DEVICE_CFG_ADDRESS, DEVICE_CFG_PWDN_NO_PWDN | speed_mode | DEVICE_CFG_CONV_MODE_SINGLE);
       
        // delay for adc startup from powerdown 
        delay_high_res_us(start_up_time_us);

        // Turn bridge on after ADC startup delay
        DL_GPIO_setPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);

        // Start conversions writing to the CONVERSION CONTROL register 
        writeRegisterDaisyChain(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START, 
                                CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);

        // Delay for the active time of the ADC 
        delay_high_res_us(active_time_us);
       
        // Turn off bridge after converion 
        DL_GPIO_clearPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);

        // ADC data out to MCU memory 
        readDataDaisyChain(adcResult, adcStatus);

        // error handling could be done here to check the status for DRDY status bit 
        
        //Setting the ADCs into power down mode from idle mode
        writeRegisterDaisyChain(DEVICE_CFG_ADDRESS, DEVICE_CFG_PWDN_PWDN | speed_mode | DEVICE_CFG_CONV_MODE_SINGLE, 
                                DEVICE_CFG_ADDRESS, DEVICE_CFG_PWDN_PWDN | speed_mode | DEVICE_CFG_CONV_MODE_SINGLE);
        
        //Print result ADC1: 0xresult ADC2: 0xresult                        
        uart_printf("ADC1: 0x%x ADC2: 0x%x\r\n", adcResult[0], adcResult[1]);

        // Delay remainder of time of interval_ms
        if (down_time_us > 0){    
            delay_us_standby(down_time_us);
        }
    }
    uart_printf("ADC Low power bridge cycling stream stopped.\r\n");
    return STATUS_OK;
}

/**
 * @brief Performs a simple test to verify proper manufacturing.
 *
 * This function sets predefined register configuration for both ADCs and reads expected values.  
 * if the ADC reading is outside the expected limits, then a manufacturing error is possible. 
 * 
 */
static enum status_enum cm_test_cmd(char *cmd){
    int32_t adcResult[2];
    uint8_t adcStatus[2];
    uint8_t value[2];
    bool test_pass_M0 = false;
    bool test_pass_adc1 = false; 
    bool test_pass_adc2 = false;
    bool test_pass_bridge = false;
    bool test_pass_rtd = false; 
    delay_cycles(100000);
    writeRegisterDaisyChain(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START,
                          CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);
    DL_GPIO_setPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);
    writeRegisterDaisyChain(MUX_CFG_ADDRESS, 0x23, MUX_CFG_ADDRESS, 0x23);
    writeRegisterDaisyChain(GAIN_CFG_ADDRESS, 0x03,GAIN_CFG_ADDRESS, 0x01);
    writeRegisterDaisyChain(REFERENCE_CFG_ADDRESS, 0x00, REFERENCE_CFG_ADDRESS, 0x31);
    writeRegisterDaisyChain(IGNORE_ADC, IGNORE_ENTRY, IDAC_MUX_CFG_ADDRESS, 0xF6);
    writeRegisterDaisyChain(IGNORE_ADC, IGNORE_ENTRY, IDAC_MAG_CFG_ADDRESS, 0x0A);

    writeRegisterDaisyChain(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START,
                          CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);
    delay_cycles(10000000);
    readRegistersDaisyChain(MUX_CFG_ADDRESS, value);

    readDataDaisyChain(adcResult, adcStatus);
    delay_cycles(100000);

uart_printf("Starting Manufacturing Test: \r\n Result:\r\n");

    if(                                 // M0 to ADC1 connection test

        value[0] == 0x23)
            {test_pass_adc1 = true;}

    test_pass_adc1? uart_printf("\tADC1 communication: Passed\r\n") : uart_printf("\tADC1 communication: FAILED\r\n");

    if(                                 // M0 to ADC2 connection test

        value[1] == 0x23)
            {test_pass_adc2 = true;}

    test_pass_adc2? uart_printf("\tADC2 communication: Passed\r\n") : uart_printf("\tADC2 communication: FAILED\r\n");


    if (                               // ADC1 signal chain test
        (adcResult[0] >= 0x172977) &&  // ADC1 min
        (adcResult[0] <= 0x19999E))    // ADC1 max
         
         
            {test_pass_bridge = true;}

    test_pass_bridge? uart_printf("\tBridge: Passed\r\n") : uart_printf("\tBridge: FAILED\r\n");

    if (                                // ADC2 signal chain test
        (adcResult[1] >= 0x1E3FAE) &&   // ADC2 min
        (adcResult[1] <= 0x216ECE))     // ADC2 max
            {test_pass_rtd = true;}

    test_pass_rtd? uart_printf("\tRTD: Passed\r\n") : uart_printf("\tRTD: FAILED\r\n");
    
    bool good_comms = false;
    good_comms = test_pass_adc1 && test_pass_adc2;
    bool good_sensors = false;
    good_sensors = test_pass_bridge && test_pass_rtd;

    if  (good_comms && good_sensors)
    {
            uart_printf("\nTest complete\r\n"); 
    }  
            else 
            {
                  if (good_comms)
                    {   uart_printf("\nACTION: Check connections and rerun test\r\n\tIf test fails again discard unit\r\n"); }
                           
                    else { 
                        uart_printf("\nACTION: Communication test failed, rerun test\r\n\tIf test fails again discard unit\r\n");
                        }}



        return STATUS_OK;
    }
    

/**
 * @brief Write to registers on two daisy-chained ADS122S14 ADCs via SPI
 *
 * Writes register values to two ADS122S14 connected in a daisy-chain
 * configuration. Data is sent in a single 10-byte SPI transaction, with the
 * first write command routed to ADC2 (first in chain) and the second to ADC1.
 *
 * @param addressADC1 Register address for ADC1 (last device in chain)
 * @param dataADC1    Data byte to write to ADC1 register
 * @param addressADC2 Register address for ADC2 (first device in chain)
 * @param dataADC2    Data byte to write to ADC2 register
 *
 * Protocol:
 * - Each write command is 2 bytes: [0x80 | address, data]
 * - Total frame: 10 bytes 
 * - ADC2 command sent first (bytes 3-4), ADC1 second (bytes 8-9)
 *
 * Timing Diagram (Single Frame):
 *
 *                                                                                Frame 1        
 *            ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
 *                                              ADC2                                                                 ADC1       
 *            ┌───────────────────────────────────────────────────────────────────────┐┌───────────────────────────────────────────────────────────────────────┐
 *
 *
 * CSn  ‾‾‾‾‾‾|________________________________________________________________________________________________________________________________________________|‾‾‾‾
 *
 * SDI:           | don't care | don't care | don't care | 80h+Address | Register Data | don't care | don't care | don't care | 80h+Address | Register Data | 
 *
 * SDO:           | Status MSB | Status LSB | Data MSB   | Data MID    | Data LSB      | Status MSB | Status LSB | Data MSB   | Data MID    | Data LSB      |
 *
 *
 */
static void writeRegisterDaisyChain(uint8_t addressADC1, uint8_t dataADC1,
                                    uint8_t addressADC2, uint8_t dataADC2) {
    static uint8_t spibuf[10];

    /* wait until previous send is done */
    while (gSPIState != idle)
    asm("wfi");

    memset(spibuf, 0, sizeof(spibuf));
    /* ADS122s1x write format is [0x80 | address (byte 1), Data (byte 2)] */
    /* first bytes go to ADC2 */
    spibuf[3] = 0x80 + addressADC2;
    spibuf[4] = dataADC2;
    /* last to ADC1 */
    spibuf[8] = 0x80 + addressADC1;
    spibuf[9] = dataADC1;

    rw_spi_non_blocking((const char *)spibuf, sizeof(spibuf));
}

/**
 * @brief Initialize ADCs with preconfigured values.
 *
 * This function iterates over a list of default register settings for each
 * ADS122S14 chip (loaded from `gADSDefault` array), and writes these to the
 * corresponding registers using writeRegisterDaisyChain()..
 *
 * @note Entries with IGNORE_ENTRY as register address in either list will be
 *       skipped during initialization.
 */
static void init_ADCs(void){
    int i;

    for(i=0;i<sizeof(gADSDefault)/2/sizeof(gADSDefault[0][0]);i++){
#ifdef DEBUG
        uart_printf("ADC1 0x%x -> 0x%x\r\n", gADSDefault[0][i].reg, gADSDefault[0][i].value);
        uart_printf("ADC2 0x%x -> 0x%x\r\n", gADSDefault[1][i].reg, gADSDefault[1][i].value);
#endif
        if(gADSDefault[0][i].reg != IGNORE_ENTRY)
            writeRegisterDaisyChain(gADSDefault[0][i].reg, gADSDefault[0][i].value, IGNORE_ADC, 0x00);
        while(gSPIState != idle)
            asm("wfi");
        if(gADSDefault[1][i].reg != IGNORE_ENTRY)
            writeRegisterDaisyChain(IGNORE_ADC, 0x00, gADSDefault[1][i].reg, gADSDefault[1][i].value);
        while(gSPIState != idle)
            asm("wfi");
    }
//startADCconversions()
}

/**
 * @brief Start ADC conversions.
 *
 * This function writes a start signal to the conversion control register of both
 * ADS122S14 chips connected in daisy-chain configuration, triggering them to begin sampling.
 */
static void startADCconversions(void) {
    writeRegisterDaisyChain(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START,
                          CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);
    ADCRunning = true;
}

/**
 * @brief Stop ADC conversions.
 *
 * This function writes a stop signal to the conversion control register of both
 * ADS122S14 chips connected in daisy-chain configuration, halting any ongoing sampling.
 */
static void stopADCconversions(void) {
    writeRegisterDaisyChain(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_STOP_STOP,
                          CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_STOP_STOP);
    ADCRunning = false;
}


/**
 * @brief Reads ADC status and conversion results from both ADCs.
 *
 * @param[out] res pointer to an array of 32-bit integers storing the conversion results
 * @param[out] status pointer to an array of 8-bit unsigned integers storing the ADC status
 */
static void readDataDaisyChain(int32_t *res, uint8_t *status){
    static uint8_t spibuf[10];
    /* wait until previous send is done */
    while (gSPIState != idle)
      asm("wfi");
    memset(spibuf, 0, sizeof(spibuf));

    /* Read data command is a 1 frame operation
    * SDI should be set to 0x000000 while clocking 24 SCLK bits
    */
    rw_spi_non_blocking((const char *)spibuf, sizeof(spibuf));

    /* wait until data trx is done */
    while (gSPIState != idle)
      asm("wfi");

    res[1] = (gSPIRxBuf[2] << 24 | gSPIRxBuf[3] << 16 | gSPIRxBuf[4] << 8) >> 8;
    res[0] = (gSPIRxBuf[7] << 24 | gSPIRxBuf[8] << 16 | gSPIRxBuf[9] << 8) >> 8;

    status[1] = gSPIRxBuf[0];
    status[0] = gSPIRxBuf[5];
}

/** @brief Read a register from two daisy-chained ADS122S14 ADCs
 *
 * Reads the same register address from both ADCs using a two-frame SPI sequence.
 * Frame 1 sends the read commands (0x40 | address), Frame 2 retrieves the data.
 *
 * @param[in]  address Register address to read (0x00-0x0F)
 * @param[out] data    Array to store register values: [0]=ADC1, [1]=ADC2
 *
 * Protocol:
 * - Frame 1: Send read command (0x40 | address) to both ADCs
 * - Frame 2: Send dummy bytes (0x00) and capture register data from previous command
 * - Each frame is 10 bytes to accommodate daisy-chain routing
 * - Register data appears in Frame 2 at byte positions 2 (ADC2) and 7 (ADC1)
 *
 * Frame 1 - Issue Read Commands:
 *
 *
 *                                                                             Frame 1        
 *            ┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
 *                                              ADC2                                                                 ADC1       
 *            ┌───────────────────────────────────────────────────────────────────┐┌──────────────────────────────────────────────────────────────────┐
 *
 *
 * CSn  ‾‾‾‾‾‾|________________________________________________________________________________________________________________________________________|‾‾‾‾
 *
 * SDI:           | don't care | don't care | don't care | 40h+Address | dont'care | don't care | don't care | don't care | 40h+Address | dont'care | 
 *
 * SDO:           | Status MSB | Status LSB | Data MSB   | Data MID    | Data LSB  | Status MSB | Status LSB | Data MSB   | Data MID    | Data LSB  |
 *
 * Frame 2 - Retrieve Register Data:    
 *                                                                        Frame 2        
 *            ┌───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
 *                                              ADC2                                                                 ADC1       
 *            ┌──────────────────────────────────────────────────────────────────┐┌───────────────────────────────────────────────────────────────────┐
 *
 *
 * CSn  ‾‾‾‾‾‾|________________________________________________________________________________________________________________________________________|‾‾‾‾
 *
 * SDI:           | don't care | don't care | don't care    | 0x00h   | 0x00h     | don't care | don't care | don't care    | 0x00h     | 0x00h     | 
 *
 * SDO:           | Status MSB | Status LSB | Register Data | Address | 0x00h     | Status MSB | Status LSB | Register Data | Address   | 0x00h     |
 *
 *
 *
 */
static void readRegistersDaisyChain(uint8_t address, uint8_t *data)
{
    static uint8_t spibuf[10];
    /* wait until previous send is done */
    while(gSPIState != idle)
        asm("wfi");
    memset(spibuf, 0, sizeof(spibuf));
    /* first bytes go to ADC2 */
    spibuf[3] = (0x40 + address);
    /* last to ADC1 */
    spibuf[8] = (0x40 + address);

    /* wait until data trx is done */
    rw_spi_non_blocking((const char*)spibuf, sizeof(spibuf));
    while(gSPIState != idle)
        asm("wfi");

    /* transfer new frame and wait until completed */
    memset(spibuf, 0, sizeof(spibuf));
    rw_spi_non_blocking((const char*)spibuf, sizeof(spibuf));
    while(gSPIState != idle)
        asm("wfi");

    /* copy register values */
    data[1] = gSPIRxBuf[2];
    data[0] = gSPIRxBuf[7];
    return;
}

//****************************************************************************
//
// Helper functions
//
//****************************************************************************

/**
 * @brief Manages adaptive timing delays for ADC sampling in the main loop
 * 
 * This cyclic function is called repeatedly from main() and implements an adaptive
 * delay mechanism based on the ADC's conversion characteristics. The function serves
 * two purposes:
 * 1. Calculates appropriate timing delays based on ADC configuration (first call or 
 *    after register changes)
 * 2. Implements the calculated delay on each call to pace ADC read operations
 * 
 * Timing Calculation (performed when time_calc flag is false):
 * - Reads SPEED_MODE from DEVICE_CFG register [1:0] to determine f_mod frequency
 * - Reads OSR (oversampling ratio) from DATA_RATE_CFG register [2:0]
 * - Handles special cases: fixed 25 SPS (OSR=0x06) and 20 SPS (OSR=0x07) modes
 * - Calculates conversion time: (OSR / f_mod) * 1000000 µs
 * - Doubles conversion time if Global Chop mode is enabled (bit [3])
 * - Derives delay period as 1/4 of conversion time for 4x oversampling in main loop
 * - Caches the calculated delay for subsequent calls
 * 
 * Subsequent Calls:
 * - Uses the cached delay value via timer_us() without recalculation
 * - Recalculation is triggered when time_calc flag is reset (e.g., via WREG command)
 * 
 * @note Depends on global flag 'time_calc' to determine calculation vs. cached mode
 * @note The 1/4 conversion time allows main loop to poll ADC 4 times per conversion
 * @note Contains commented bridge power control and ADC timeout recovery features
 * 
 * @return STATUS_OK 
 */
 
enum status_enum ads122s14_ptx_daisychain_cyclic(void){
    uint8_t value[2];
    uint8_t speed_mode, osr_reg , status;
    uint32_t f_mod_hz, osr, time_us, latency_in_modclks, conversion_time_modclks, conversion_pull_modclks;
    uint32_t conversion_no_calc_cycles, conversion_pull_cycles;
    float  t_mod_us, conversion_pull_us;
    static uint32_t time_us_no_calc = 0;

    // speed mode table 
    static const struct {
        uint8_t reg_value;
        uint32_t f_mod_hz;
    } speed_modes[] = {
        {0x00, 32000},   // Mode 0: 32 kHz
        {0x01, 256000},  // Mode 1: 256 kHz
        {0x02, 512000},  // Mode 2: 512 kHz
        {0x03, 1024000},  // Mode 3: 1.024 MHz
    };

    // Second conversion latency table 
static const struct {
        uint8_t reg_value;
        uint32_t latency[4];
	
    } latency_table[] = {
        {0x00, 16, 16, 16, 16},                  // OSR16
        {0x01, 32, 32, 32, 32},                  // OSR32
        {0x02, 128, 128, 128, 128},              // OSR128
        {0x03, 256, 256, 256, 256},              // OSR256
        {0x04, 512, 512, 512, 512},              // OSR512
        {0x05, 1024, 1024, 1024, 1024},          // OSR1024
        {0x06, 1280, 10240, 20480, 40960},       // SPS25
        {0x07, 1600, 12800, 25600, 51200}        // SPS20

    };

    if (!time_calc){
        // Read Speed mode from DEVICE_CFG_ADDRESS[1:0]
        readRegistersDaisyChain(DEVICE_CFG_ADDRESS, value);
        speed_mode = value[0] & 0x03; // Bits [1:0]
        f_mod_hz = speed_modes[speed_mode].f_mod_hz;

        // Read DATA_RATE register for OSR, Delay bits, and Global Chop 
        readRegistersDaisyChain(DATA_RATE_CFG_ADDRESS, value);

        // Global Chop
        bool global_chop = value[0] & 0x08; // Bit [3]

        // Isolating OSR value bits
        osr_reg = value[0] & 0x07; // Bits [2:0]
    
        // tMOD Calculation based on fMOD
        t_mod_us = 1000000.0f / f_mod_hz; // tMOD time in us

        // Second Conversion Latency table 
        latency_in_modclks = latency_table[osr_reg].latency[speed_mode];

        // Converting conversion time to 1/4 conversion time for DRDY staus pulling 
        conversion_time_modclks = latency_in_modclks;
        conversion_pull_modclks = conversion_time_modclks / 4; 
        conversion_pull_us = conversion_pull_modclks * t_mod_us + 5; // buffer added for syncronization 

        if (global_chop){                       // Global Chop bit check 
            conversion_pull_us *= 2;
        }   
        // setting time_us_no_calc for efficient runs after calculations
        // After any register write these will be recalculated 
        time_us = conversion_pull_us; 
        time_us_no_calc = time_us;
        conversion_pull_cycles = (uint32_t)(conversion_pull_us * g_system_clk_frequency_mhz);
        conversion_no_calc_cycles = conversion_pull_cycles;

        //timer_us(time_us); // timer fuction for delay
        delay_high_res_us(time_us);
        
        time_calc = true; // time has beeen calc'ed
    }
    else {
        //timer_us(time_us_no_calc); // timer function for delay
        delay_high_res_us(time_us_no_calc);
    }

    /* this is a work around, as it can happen when starting up from the loop power, 
    that the ADC get initialized to early and does not start sampling */
    /*if(gADCTimeoutTimer++ > 500){  if not getting one sample in 0.5s something is odd, restart ADC. 
        if(ADCRunning){
            //init_ADCs();
            startADCconversions();
            gADSConfigMem->bridge_time = 200; // 5 samples per second 
            gADSConfigMem->config_chksum = crc8((void*)(gADSConfigMem)+2, gADSConfigMem->config_length);
            gADCTimeoutTimer = 0;
        }
    }*/

    return STATUS_OK;
}

/**
 * @brief init function of ADS122S14 driver. Inits SPI, DMA and the ADC
 *
 * @param[in] config pointer where config can be stored
 * @return STATUS_OK if everything went well
 */
enum status_enum ads122s14_ptx_daisychain_init(void *config){
    enum status_enum ret = STATUS_OK;

    static const DL_SPI_Config SPI_0_config = {
        .mode        = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA1,
        .parity      = DL_SPI_PARITY_NONE,
        .dataSize    = DL_SPI_DATA_SIZE_8,
        .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_0,
    };

    static const DL_SPI_ClockConfig SPI_0_clockConfig = {
        .clockSel    = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
    };

    static const DL_DMA_Config DMA_CH0Config = {
        .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode   = DL_DMA_NORMAL_MODE,
        .destIncrement  = DL_DMA_ADDR_UNCHANGED,
        .srcIncrement   = DL_DMA_ADDR_INCREMENT,
        .destWidth      = DL_DMA_WIDTH_BYTE,
        .srcWidth       = DL_DMA_WIDTH_BYTE,
        .trigger        = DMA_SPI0_TX_TRIG,
        .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    };

    static const DL_DMA_Config DMA_CH1Config = {
        .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode   = DL_DMA_NORMAL_MODE,
        .destIncrement  = DL_DMA_ADDR_INCREMENT,
        .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
        .destWidth      = DL_DMA_WIDTH_BYTE,
        .srcWidth       = DL_DMA_WIDTH_BYTE,
        .trigger        = DMA_SPI0_RX_TRIG,
        .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    };


    /* turn off bridge power */
    DL_GPIO_clearPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);

    uart_printf("Using ADS122S14 in daisy chain for pressure sensing \r\n");

    /*Keep CS high*/
    DL_GPIO_setPins(GPIO_ADC_PORT, GPIO_ADC_ADC_CS_PIN);

    __disable_irq();
    DL_SPI_disable(SPI_0_INST);
    DL_SPI_setClockConfig(SPI_0_INST, (DL_SPI_ClockConfig *) &SPI_0_clockConfig);
    DL_SPI_init(SPI_0_INST, (DL_SPI_Config *) &SPI_0_config);
    DL_SPI_setBitRateSerialClockDivider(SPI_0_INST, 3);
    DL_SPI_setFIFOThreshold(SPI_0_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
    NVIC_ClearPendingIRQ(SPI_0_INST_INT_IRQN);
    DL_SPI_enableDMATransmitEvent(SPI_0_INST);
    DL_SPI_enableDMAReceiveEvent(SPI_0_INST, DL_SPI_DMA_INTERRUPT_RX);
    DL_SPI_enableInterrupt(SPI_0_INST, DL_SPI_INTERRUPT_DMA_DONE_RX);
    DL_SPI_disableInterrupt(SPI_0_INST, DL_SPI_INTERRUPT_RX);

    DL_SPI_enable(SPI_0_INST);
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);
    __enable_irq();

    DL_DMA_initChannel(DMA, DMA_CH2_CHAN_ID , (DL_DMA_Config *) &DMA_CH0Config);
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &DMA_CH1Config);
    gADSConfigMem = config;
    gADSConfigMem->config_length = sizeof(struct ads122s14_ptx_chain_config_struct)-2;
    if(crc8(config+2, gADSConfigMem->config_length) != gADSConfigMem->config_chksum){
        ERR_LOG("ADS122S14 config reset");
        gADSConfigMem->bridge_time = 500; /* 2 samples per second */
        gADSConfigMem->bridge_state = on;
        gADSConfigMem->config_chksum = crc8(config+2, gADSConfigMem->config_length);
        ret = STATUS_CFG_CHANGED;
    }

    if(gADSConfigMem->bridge_state == off){
        DL_GPIO_clearPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);
    }else if(gADSConfigMem->bridge_state == on){
        DL_GPIO_setPins(GPIO_ADC_PORT, GPIO_ADC_ADC_BRIDGE_SWITCH_CONTROL_PIN);
    }

    init_ADCs();
    
    startADCconversions();

    gADSConfigMem->config_chksum = crc8(config+2, gADSConfigMem->config_length);
    return ret;
}

/**
 * @brief called by main loop to poll for new ADC reading. Needs to provide new reading if available
 *
 * @param[out] adc_raw pointer to store readings to
 * @return STATUS_OK if new readings where available; STATUS_NOCHANGE if no new readings are available
 */
enum status_enum ads122s14_ptx_daisychain_get_reading(int32_t *adc_raw){
    uint8_t status[2]; 
    static bool ADC1_DRDY;
    static bool ADC2_DRDY;
        
    readDataDaisyChain(adc_raw, status);
    if(status[0] & 0x01){
        ADC1_DRDY = 1;
    }
     if(status[1] & 0x01){
        ADC2_DRDY = 1;
    } 
    
    if(ADC1_DRDY && ADC2_DRDY){ /* check status bits, if ADCs have finished conversion */
        gADCTimeoutTimer = 0;
       
        memcpy(gRawReading, adc_raw, sizeof(gRawReading));
        if(gStreaming)
              uart_printf("0x%x 0x%x\r\n", gRawReading[0], gRawReading[1]);
        ADC1_DRDY = 0;
        ADC2_DRDY = 0;
        return STATUS_OK;
    }
    return STATUS_NOCHANGE;
}

/**
 * @brief handles UART commands
 *
 * @param[in] cmd pointer of UART input
 * @return return value of command function, STATUS_CMDNOTFOUND if command is not available, STATUS_ERR in case of generic error
 */
enum status_enum ads122s14_ptx_daisychain_cmd(char *cmd){
    int i;
    char *subcmd = strtok(cmd, " ");
    bool found=false;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
       if(!strcmp(commands[i].name, subcmd)){
           /* command found, call it */
           return(commands[i].function(cmd+strlen(commands[i].name)+1));
           found = true;
           break;
       }
    }
    if(found == false) { /* unknown command */
       uart_printf("command not found - try out help\r\n");
       return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; /* should never get here */
}