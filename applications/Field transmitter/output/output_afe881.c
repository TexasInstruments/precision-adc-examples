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

#include "output_afe881.h"
#include "ti_msp_dl_config.h"

#include "main.h"
#include "system/uart.h"
#include "system/flash.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef HART
#include "soft_timer.h"
#include "hart_appli.h"
#include "hart_frame.h"
#endif

static enum status_enum help_cmd(char *cmd);
static enum status_enum get_cmd(char *cmd);
static enum status_enum rreg_cmd(char *cmd);
static enum status_enum wreg_cmd(char *cmd);
static enum status_enum hart_test_cmd(char *cmd);


#define READ_REG        0x80

#define NOP_CMD         0x00

#define DAC_DATA_REG    0x01

#define GEN_STATUS_REG  0x21
#define PVDD_HI_BIT     (1<<4)

#define MODEM_CFG_REG   0x0E


static int32_t gOutRawReading;
enum spi_state_enum{idle, tx};
static volatile enum spi_state_enum gSPIState = idle;
static volatile char *gSPITxBuf;
static volatile int gSPITxLen;
static volatile int gSPIRxBuf;
static uint8_t gSPIBuf[4];

static void rw_register_non_blocking(uint8_t reg, uint16_t data);

struct afe881_config_struct{
    uint8_t config_length;
    uint8_t config_chksum;
    bool pvdd_hi;
};
static struct afe881_config_struct *afe_config;

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"get", "return raw reading", get_cmd},
    {"rreg", "read register - rreg 0xreg", rreg_cmd},
    {"wreg", "write register - wreg 0xreg 0xvalue", wreg_cmd},
    {"hart_test", "different hart experiments", hart_test_cmd},
};

/**
 * @brief Prints help information about available commands.
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 */
static enum status_enum help_cmd(char *cmd){
    int i;
    uart_printf("All commands must start with out \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief Reads and prints register values from AFE881/882.
 *
 * This function parses the input command string, extracts the target register
 * address in hexadecimal format (0xXX), reads the corresponding value
 * from the AFE, and then prints out the register value to the UART.
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
    char *subcmd = strtok(cmd, " ");
    int reg;
    int ret;
    if(!subcmd)
        return STATUS_CMDERROR;
    else{
        ret = sscanf(cmd, "0x%x", &reg);
        if(ret != 1) return STATUS_CMDERROR;
        rw_register_non_blocking(reg|READ_REG, NOP_CMD);
        rw_register_non_blocking(NOP_CMD, NOP_CMD);
        while(gSPIState != idle) asm("wfi");
        uart_printf("Reg 0x%x, 0x%x\r\n", reg, gSPIRxBuf);
        return STATUS_OK;
    }
    return STATUS_CMDERROR; //should never get here
}

/**
 * @brief Writes values to specified registers
 *
 * This function parses the input command string, extracts the target
 * address and value in hexadecimal format, and then writes out
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
    int reg, val;
    if(sscanf(cmd, "0x%x 0x%x", &reg, &val) == 2){ // need exactly 2 arguments
        rw_register_non_blocking(reg, val);
        return STATUS_OK;
    }else
        return STATUS_CMDERROR;
}

/**
 * @brief Initializes the HART
 *
 * This function sets up the UART interface for serial communication, configures GPIO pins as needed,
 * and enables HART modem in register settings.
 *
 * @return STATUS_OK always
 */
static enum status_enum hart_init(void){
    //init uart

    static const DL_UART_Main_ClockConfig gUART_IOClockConfig = {
        .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };

    static const DL_UART_Main_Config gUART_IOConfig = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_EVEN,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
    };

    DL_UART_Main_setClockConfig(UART_IO_INST, (DL_UART_Main_ClockConfig *) &gUART_IOClockConfig);
    DL_UART_Main_init(UART_IO_INST, (DL_UART_Main_Config *) &gUART_IOConfig);
    DL_UART_Main_setOversampling(UART_IO_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_IO_INST, 1666, 43); //1200 baud @ 32 MHz
    DL_UART_Main_enable(UART_IO_INST);

    //set gpios
    DL_GPIO_setPins(GPIO_DAC_PORT, GPIO_DAC_RTS_PIN); // enable demodulator

    //enable modem
    rw_register_non_blocking(MODEM_CFG_REG, 0x40|1<<3);
    return STATUS_OK;
}

/**
 * @brief Sends a single byte over the UART using the HART peripherals.
 *
 * This function transmits the given byte using the DL_UART_Main_transmitData API call,
 * and waits until the transmission is complete to return.
 *
 * @param[in]  byte The byte to be sent.
 * @return STATUS_OK always
 */
static enum status_enum hart_send_byte(char byte){
    DL_UART_Main_transmitData(UART_IO_INST, byte);
    while (DL_UART_Main_isBusy(UART_IO_INST)); // wait till send is complete

    return STATUS_OK;
}

/**
 * @brief Sends a null-terminated string over the UART using the HART peripherals.
 *
 * This function enables the modulator, waits until modem is ready
 * sends each character of the string individually via hart_send_byte(), and finally enables the demodulator.
 *
 * Note: The caller is responsible for ensuring that the supplied string is properly null-terminated.
 *
 * @param[in]  s The string to be sent.
 * @return STATUS_OK always
 */
static enum status_enum hart_send_str(char *s){
    DL_GPIO_clearPins(GPIO_DAC_PORT, GPIO_DAC_RTS_PIN); // enable modulator
    delay_cycles(32000);         // wait for modulator to be active, CD pin should go to low

    while(*s != 0)
        hart_send_byte(*s++);

    DL_GPIO_setPins(GPIO_DAC_PORT, GPIO_DAC_RTS_PIN); // enable demodulator
    return STATUS_OK;
}

/**
 * @brief Parses the hart_test command to init and test if HART modulator is doing something
 *
 * This function parses a string command, splitting it into subcommands using spaces as delimiters,
 * and executes actions based on the first word of the command. Supported subcommands are "init"
 * and "send".
 *
 * @param[in]  cmd The string command to be executed.
 * @return STATUS_OK if execution was successful, an error code otherwise (e.g., STATUS_CMDERROR).
 */
static enum status_enum hart_test_cmd(char *cmd){
    char *subcmd = strtok(cmd, " ");
    if(!subcmd) return STATUS_CMDERROR;
    if(!strcmp("init", subcmd)){
        return hart_init();
    } else if(!strcmp("send", subcmd)){
        return hart_send_str("test");
    } else
        return STATUS_CMDERROR;
}

/**
 * @brief Interrupt Service Routine (ISR) for SPI transaction on instance SPI_1_INST.
 *
 * This function handles incoming interrupts from the SPI peripheral,
 * and executes actions based on the type of interrupt received. It supports transmission
 * and reception of data, as well as state machine control for multi-byte transactions.
 *
 */
void SPI_1_INST_IRQHandler(void){ // todo: this will need to be moved somewhere and get configurable when multiple trx cards are available using spi irq
    static char dummy;
    switch (DL_SPI_getPendingInterrupt(SPI_1_INST)) {
        case DL_SPI_IIDX_TX:
            break;

        case DL_SPI_IIDX_RX:
            if(gSPIState == tx){
                switch(gSPITxLen){
                    case 1:
                        DL_SPI_transmitData8(SPI_1_INST, *(gSPITxBuf++));
                        gSPITxLen--;
                        gSPIRxBuf = DL_SPI_receiveData8(SPI_1_INST)<<8;
                        break;

                    case 0:
                        gSPIRxBuf |= DL_SPI_receiveData8(SPI_1_INST);
                        DL_GPIO_setPins(GPIO_DAC_PORT, GPIO_DAC_CS_PIN);
                        gSPIState = idle;
                        gSPITxLen--;
                        break;

                    default:
                        DL_SPI_transmitData8(SPI_1_INST, *(gSPITxBuf++));
                        gSPITxLen--;
                        dummy = DL_SPI_receiveData8(SPI_1_INST);
                        break;
                }
            }
            break;
        default:

            gSPIRxBuf = DL_SPI_receiveData8(SPI_1_INST);
            break;
    }
}

/**
 * @brief Initiates a non-blocking SPI write operation.
 *
 * This function starts transmitting the given buffer over the configured SPI interface,
 * and transitions the state machine to idle as soon as all data has been sent. It waits until all prev SPI transactions are completed
 *
 * @param[in]  buf The buffer containing the data to be transmitted.
 * @param[in]  len The number of data bytes to be sent
 */
static void rw_spi_non_blocking(const char *buf, int len){
    int i;
    while(gSPIState != idle) asm("wfi"); //wait till idle

    gSPITxBuf = (char *) buf;
    gSPITxLen = len;
    DL_GPIO_clearPins(GPIO_DAC_PORT, GPIO_DAC_CS_PIN);
    gSPIState = tx;
    gSPITxLen--;
    DL_SPI_transmitData8(SPI_1_INST, *(gSPITxBuf++));
}

/**
 * @brief Writes a register of AFE881/882 to an 16 bit value
 *
 * This function constructs a 24-bit data packet, and then calls rw_spi_non_blocking() to transmit it over the configured SPI interface.
 *
 * @param[in]  reg The 8-bit register address, which is prepended to the 16-bit value as most-significant bits in a 24-bit packet.
 * @param[in]  data The 16-bit register value to be written.
 */
static void rw_register_non_blocking(uint8_t reg, uint16_t data){
    gSPIBuf[0] = reg;
    gSPIBuf[1] = (data&0xff00)>>8;
    gSPIBuf[2] = data&0xff;
    rw_spi_non_blocking((const char*)gSPIBuf, 3);
}

/**
 * @brief Handles the "get" command from command line
 *
 * This function prints the current raw input reading value to the UART
 *
 * @param[in]  cmd The incoming command string (only examined for "get" prefix in this implementation).
 *
 * @return STATUS_OK always
 */
static enum status_enum get_cmd(char *cmd){
    uart_printf("0x%x\r\n", gOutRawReading);
    return STATUS_OK;
}

/**
 * @brief Cyclic function not used.
 *
 * @return STATUS_OK
 */
enum status_enum output_afe881h1_cyclic(void){
#ifdef HART
    hart_poll();
    //HART_SoftTimer_Handler();
#endif
    return STATUS_OK;
}

/**
 * @brief init function of AFE881H1 driver. Inits SPI, and the DAC
 *
 * @param[in] config pointer where config can be stored
 * @return STATUS_OK if everything went well
 */
enum status_enum output_afe881h1_init(void *config){
    enum status_enum ret = STATUS_OK;

    static const DL_SPI_Config gSPI_1_config = {
        .mode        = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA0,
        .parity      = DL_SPI_PARITY_NONE,
        .dataSize    = DL_SPI_DATA_SIZE_8,
        .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_0,
    };

    static const DL_SPI_ClockConfig gSPI_1_clockConfig = {
        .clockSel    = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
    };

    uart_printf("Using AFE88x\r\n");

    DL_SPI_disable(SPI_1_INST);
    DL_SPI_setClockConfig(SPI_1_INST, (DL_SPI_ClockConfig *) &gSPI_1_clockConfig);
    DL_SPI_init(SPI_1_INST, (DL_SPI_Config *) &gSPI_1_config);
    DL_SPI_setBitRateSerialClockDivider(SPI_1_INST, 3);
    //DL_SPI_setFIFOThreshold(SPI_1_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_EMPTY);
    DL_SPI_setFIFOThreshold(SPI_1_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);

    NVIC_ClearPendingIRQ(SPI_1_INST_INT_IRQN);
    //DL_SPI_enableInterrupt(SPI_1_INST, (DL_SPI_INTERRUPT_TX_EMPTY));
    DL_SPI_enable(SPI_1_INST);
    NVIC_EnableIRQ(SPI_1_INST_INT_IRQN);
    DL_SPI_enableInterrupt(SPI_1_INST, DL_SPI_INTERRUPT_RX);

    const char disable_crc[4] = {0x02, 0x00, 0x26, 0x24};
    const char enable_sdo[3] = {0x02, 0x00, 0x00};
    rw_spi_non_blocking(disable_crc, sizeof(disable_crc));
    rw_spi_non_blocking(enable_sdo, sizeof(enable_sdo));

    afe_config = config;
    afe_config->config_length = sizeof(struct afe881_config_struct)-2;
    if(crc8(config+2, afe_config->config_length) != afe_config->config_chksum){
        ERR_LOG("AFE88x config reset");
        afe_config->config_chksum = crc8(config+2, afe_config->config_length);
        ret = STATUS_CFG_CHANGED;
    }


    //read GEN_STATUS 0x21, PVDD_HI bit and set offset accordingly
    rw_register_non_blocking(GEN_STATUS_REG|READ_REG, NOP_CMD);
    rw_register_non_blocking(NOP_CMD, NOP_CMD);
    while(gSPIState != idle) asm("wfi");
    afe_config->pvdd_hi =  (gSPIRxBuf & PVDD_HI_BIT)?true:false;
    uart_printf("AFE88x PVDD: %s - Make sure jumpers are correct!\r\n", afe_config->pvdd_hi? "3.3 V": "1.8 V");

    output_afe881h1_set_reading(0);

#ifdef HART
    //hart stack init
    soft_timer_init();
    hart_appli_init();

    //set gpios
   DL_GPIO_setPins(GPIO_DAC_PORT, GPIO_DAC_RTS_PIN); // enable demodulator

   //enable modem
   rw_register_non_blocking(MODEM_CFG_REG, 0x40|1<<3);
#endif


    afe_config->config_chksum = crc8(config+2, afe_config->config_length);
    return ret;
}

/**
 * @brief called by main loop to set new DAC value
 *7
 * @param[in] raw value to set the DAC to
 * @return STATUS_OK always
 */
enum status_enum output_afe881h1_set_reading(int32_t raw){
    gOutRawReading = raw;
    rw_register_non_blocking(DAC_DATA_REG, raw);
    return STATUS_OK;
}

/**
 * @brief handles UART commands
 *
 * @param[in] cmd pointer of UART input
 * @return return value of command function, STATUS_CMDNOTFOUND if command is not available, STATUS_ERR in case of generic error
 */
enum status_enum output_afe881h1_cmd(char *cmd){
    int i;
    char *subcmd = strtok(cmd, " ");
    bool found=false;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        if(!strcmp(commands[i].name, subcmd)){
            // command found, call it
            return(commands[i].function(cmd+strlen(commands[i].name)+1));
            found = true;
            break;
        }
    }
    if(found == false) { //unknown command
        uart_printf("command not found - try out help\r\n");
        return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; //should never get here
}



