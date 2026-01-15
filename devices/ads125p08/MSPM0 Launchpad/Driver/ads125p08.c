/**
 *
 *
 * @brief This file contains all basic communication and device setup.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2025-2026 Texas Instruments Incorporated - http://www.ti.com/
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

#include "Driver/ads125p08.h"         
#include "Driver/ads125p08_pages.h"
#include "Driver/ads125p08_page0.h"
#include "Driver/ads125p08_page1.h"
#include "Driver/hal.h"
#include "Driver/crc.h"
#include "ti/driverlib/m0p/dl_core.h"
#include <stdint.h>
#include <stdbool.h>

// Register configuration tool available here:
// https://dev.ti.com/gallery/view/PADC/PADC_Design_Calculator_Tool/?device=ADS125P08
#include "Driver/ads125p08_RegisterDetails.h"   



static uint8_t buildSPIarray(uint8_t byte1, uint8_t byte2);

const char *generalRegisterNames[NUM_GENERAL_REGISTERS] = {"DEVICE_ID", \
"REVISION_ID", "STATUS_MSB", "STATUS_LSB", "ADC_REF_STATUS", \
"DIGITAL_STATUS", "AGPIO_DATA_INPUT", "GPIO_DATA_INPUT", "FIFO_SEQ_STATUS", \
"FIFO_DEPTH_MSB", "FIFO_DEPTH_LSB", "CONVERSION_CTRL", "RESET", "ADC_CFG", \
"REFERENCE_CFG", "CLK_DIGITAL_CFG", "AGPIO_CFG0", "AGPIO_CFG1", "GPIO_CFG", \
"SPARE_CFG", "SEQUENCER_CFG", "SEQUENCE_STEP_EN_0", "SEQUENCE_STEP_EN_1", \
"SEQUENCE_STEP_EN_2", "SEQUENCE_STEP_EN_3", "FIFO_CFG", "FIFO_THRES_A_MSB", \
"FIFO_THRES_A_LSB", "FIFO_THRES_B_MSB", "FIFO_THRES_B_LSB", "DIAG_MONITOR_CFG", \
"POSTFILTER_CFG0", "POSTFILTER_CFG1", "POSTFILTER_CFG2", "CS_FWD_CFG", \
"AGPIO_FWD_CFG", "GPIO_FWD_CFG", "REG_MAP_CRC", "PAGE_INDICATOR", "PAGE_POINTER"
};

const char *stepxRegisterNames[NUM_STEP_REGISTERS] = {"STEPX_AINP_CFG", \
"STEPX_AINN_CFG", "STEPX_ADC_REF_CFG", "STEPX_FLTR1_CFG", "STEPX_DELAY_MSB_CFG", \
"STEPX_DELAY_LSB_CFG", "STEPX_OFFSET_CAL_MSB", "STEPX_OFFSET_CAL_ISB", \
"STEPX_OFFSET_CAL_LSB", "STEPX_GAIN_CAL_MSB", "STEPX_GAIN_CAL_LSB", \
"STEPX_OW_SYSMON_CFG", "STEPX_TDAC_CFG0", "STEPX_TDAC_CFG1", "STEPX_SPARE_CFG", \
"STEPX_AGPIO_DATA_OUT", "STEPX_GPIO_DATA_OUT", "STEPX_REG_MAP_CRC", \
"STEPX_PAGE_INDICATOR", "STEPX_PAGE_POINTER"
};

const char *pageNames[NUM_PAGES] = {"GENERAL_SETTINGS", \
"STEP_1", "STEP_2", "STEP_3", "STEP_4", "STEP_5", "STEP_6", "STEP_7", \
"STEP_8", "STEP_9", "STEP_10", "STEP_11", "STEP_12", "STEP_13", "STEP_14", \
"STEP_15", "STEP_16", "STEP_17", "STEP_18", "STEP_19", "STEP_20", "STEP_21", \
"STEP_22", "STEP_23", "STEP_24", "STEP_25", "STEP_26", "STEP_27", "STEP_28", \
"STEP_29", "STEP_30", "STEP_31", "STEP_32"
};

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations
static uint8_t registerMap[NUM_PAGES][NUM_GENERAL_REGISTERS];
static uint8_t dataTx[6] = {0};
static uint8_t dataRx[6] = {0};
static uint8_t data00[6] = {0};
uint8_t adcDataArray[3];
static uint8_t dataFIFO[3] = {0x00, 0x0F, 0x00};
static uint8_t numWords = 0;
uint32_t numSamples = 0;
uint8_t streamRx[6];
REG_IO REG_DATA;
ADC_IO Conversions;
ADC_IO FIFO;

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

static void restoreRegisterDefaults(void)
{
    // use these registerMap definitions to change your configuration as needed.
    registerMap[GENERAL_SETTINGS][DEVICE_ID_ADDRESS]              = DEVICE_ID_DEFAULT;
    registerMap[GENERAL_SETTINGS][REVISION_ID_ADDRESS]            = REVISION_ID_DEFAULT;
    registerMap[GENERAL_SETTINGS][STATUS_MSB_ADDRESS]             = STATUS_MSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][STATUS_LSB_ADDRESS]             = STATUS_LSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][ADC_REF_STATUS_ADDRESS]         = ADC_REF_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][DIGITAL_STATUS_ADDRESS]         = DIGITAL_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][AGPIO_DATA_INPUT_ADDRESS]       = AGPIO_DATA_INPUT_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_DATA_INPUT_ADDRESS]        = GPIO_DATA_INPUT_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_SEQ_STATUS_ADDRESS]        = FIFO_SEQ_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_DEPTH_MSB_ADDRESS]         = FIFO_DEPTH_MSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_DEPTH_LSB_ADDRESS]         = FIFO_DEPTH_LSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][CONVERSION_CTRL_ADDRESS]        = CONVERSION_CTRL_DEFAULT;
    registerMap[GENERAL_SETTINGS][RESET_ADDRESS]                  = RESET_DEFAULT;
    registerMap[GENERAL_SETTINGS][ADC_CFG_ADDRESS]                = ADC_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][REFERENCE_CFG_ADDRESS]          = REFERENCE_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][CLK_DIGITAL_CFG_ADDRESS]        = CLK_DIGITAL_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][AGPIO_CFG0_ADDRESS]             = AGPIO_CFG0_DEFAULT;
    registerMap[GENERAL_SETTINGS][AGPIO_CFG1_ADDRESS]             = AGPIO_CFG1_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_CFG_ADDRESS]               = GPIO_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][SPARE_CFG_ADDRESS]              = SPARE_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCER_CFG_ADDRESS]          = SEQUENCER_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_0_ADDRESS]     = SEQUENCE_STEP_EN_0_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_1_ADDRESS]     = SEQUENCE_STEP_EN_1_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_2_ADDRESS]     = SEQUENCE_STEP_EN_2_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_3_ADDRESS]     = SEQUENCE_STEP_EN_3_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_CFG_ADDRESS]               = FIFO_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_THRES_A_MSB_ADDRESS]       = FIFO_THRES_A_MSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_THRES_A_LSB_ADDRESS]       = FIFO_THRES_A_LSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_THRES_B_MSB_ADDRESS]       = FIFO_THRES_B_MSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][FIFO_THRES_B_LSB_ADDRESS]       = FIFO_THRES_B_LSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][DIAG_MONITOR_CFG_ADDRESS]       = DIAG_MONITOR_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][POSTFILTER_CFG0_ADDRESS]        = POSTFILTER_CFG0_DEFAULT;
    registerMap[GENERAL_SETTINGS][POSTFILTER_CFG1_ADDRESS]        = POSTFILTER_CFG1_DEFAULT;
    registerMap[GENERAL_SETTINGS][POSTFILTER_CFG2_ADDRESS]        = POSTFILTER_CFG2_DEFAULT;
    registerMap[GENERAL_SETTINGS][CS_FWD_CFG_ADDRESS]             = CS_FWD_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][AGPIO_FWD_CFG_ADDRESS]          = AGPIO_FWD_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_FWD_CFG_ADDRESS]           = GPIO_FWD_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][REG_MAP_CRC_ADDRESS]            = REG_MAP_CRC_DEFAULT;
    registerMap[GENERAL_SETTINGS][PAGE_INDICATOR_ADDRESS]         = PAGE_INDICATOR_DEFAULT;
    registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS]           = PAGE_POINTER_DEFAULT;

    for ( uint8_t STEPX = 1; STEPX < 32; STEPX++ )  {
        registerMap[STEPX][STEPX_AINP_CFG_ADDRESS]                = STEPX_AINP_CFG_DEFAULT;
        registerMap[STEPX][STEPX_AINN_CFG_ADDRESS]                = STEPX_AINN_CFG_DEFAULT;
        registerMap[STEPX][STEPX_ADC_REF_CFG_ADDRESS]             = STEPX_ADC_REF_CFG_DEFAULT;
        registerMap[STEPX][STEPX_FLTR1_CFG_ADDRESS]               = STEPX_FLTR1_CFG_DEFAULT;
        registerMap[STEPX][STEPX_DELAY_MSB_CFG_ADDRESS]           = STEPX_DELAY_MSB_CFG_DEFAULT;
        registerMap[STEPX][STEPX_DELAY_LSB_CFG_ADDRESS]           = STEPX_DELAY_LSB_CFG_DEFAULT;
        registerMap[STEPX][STEPX_OFFSET_CAL_MSB_ADDRESS]          = STEPX_OFFSET_CAL_MSB_DEFAULT;
        registerMap[STEPX][STEPX_OFFSET_CAL_ISB_ADDRESS]          = STEPX_OFFSET_CAL_ISB_DEFAULT;
        registerMap[STEPX][STEPX_OFFSET_CAL_LSB_ADDRESS]          = STEPX_OFFSET_CAL_LSB_DEFAULT;
        registerMap[STEPX][STEPX_GAIN_CAL_MSB_ADDRESS]            = STEPX_GAIN_CAL_MSB_DEFAULT;
        registerMap[STEPX][STEPX_GAIN_CAL_LSB_ADDRESS]            = STEPX_GAIN_CAL_LSB_DEFAULT;
        registerMap[STEPX][STEPX_OW_SYSMON_CFG_ADDRESS]           = STEPX_OW_SYSMON_CFG_DEFAULT;
        registerMap[STEPX][STEPX_TDAC_CFG0_ADDRESS]               = STEPX_TDAC_CFG0_DEFAULT;
        registerMap[STEPX][STEPX_TDAC_CFG1_ADDRESS]               = STEPX_TDAC_CFG1_DEFAULT;
        registerMap[STEPX][STEPX_SPARE_CFG_ADDRESS]               = STEPX_SPARE_CFG_DEFAULT;
        registerMap[STEPX][STEPX_AGPIO_DATA_OUT_ADDRESS]          = STEPX_AGPIO_DATA_OUT_DEFAULT;
        registerMap[STEPX][STEPX_GPIO_DATA_OUT_ADDRESS]           = STEPX_GPIO_DATA_OUT_DEFAULT;
        registerMap[STEPX][STEPX_REG_MAP_CRC_ADDRESS]             = STEPX_REG_MAP_CRC_DEFAULT;
    }  
}

uint8_t getRegisterValue(uint8_t page, uint8_t address)
{
    return registerMap[page][address];
}

void initADC()
{
    // Initalize registers as defined from the _RegisterDetails.h file
    restoreRegisterDefaults();
    for (int i = 0; i < SEQUENCE_SIZE; i++ )
        {
            writeSingleRegister(0, PAGE_POINTER_ADDRESS,ADS125P08_sequencer_config[i].pageNumber);
       
        for (int j = 0; j < ADS125P08_sequencer_config[i].registerCount; j++ )
            {
                writeSingleRegister(i, ADS125P08_sequencer_config[i].configPointer[j].addr,ADS125P08_sequencer_config[i].configPointer[j].value);
            }
    }

    // clear status flags
    writeSingleRegister(GENERAL_SETTINGS, ADC_REF_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, DIGITAL_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, FIFO_SEQ_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, STATUS_MSB_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, STATUS_LSB_ADDRESS, 0xFF);
}

bool resetDevice(void)
{
    restoreRegisterDefaults();
    writeSingleRegister(GENERAL_SETTINGS, RESET_ADDRESS, 0x5A);   // write reset code to register
    delay_us(500);                                                // wait for reset

    return 0;
}

REG_IO writeSingleRegister(uint8_t page, uint8_t address, uint8_t data)

{
//The write format is [0x80 | address (byte 1), Data (byte 2)]
//----------------------------------------WREG----------------------------------
//      __                                                                     _
//[CSn]   |___________________________________________________________________|
//          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
//[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |______
//          ___
//[DIN] ___|   |___<---ADDRESS[5:0]-------><-----------DATA-------------->______
//------------------------------------------------------------------------------
               
    //change page if needed
    if (page != registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS]) {  
        numWords = buildSPIarray(0x80 + PAGE_POINTER_ADDRESS, page);
        spiSendReceiveArrays(dataTx, dataRx, numWords);   
        registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS] = page;     
    }

    //build and send frame
    numWords = buildSPIarray(0x80 + address, data);
    spiSendReceiveArrays(dataTx, dataRx, numWords);
    
    if (STATUS_ENABLED) {
        REG_DATA.STATUS_MSB = dataRx[0];
        REG_DATA.STATUS_LSB = dataRx[1];
    }         

    // special case: update internal page pointer
    if (address == 0x3F) {                                   
        registerMap[GENERAL_SETTINGS][address] = data;
    }

    registerMap[page][address] = data;
    return(REG_DATA);
}

REG_IO readSingleRegister(uint8_t page, uint8_t address)
{
// Command is sent as 2, 16-bit frames, register data is returned in the first byte of frame #2.
// RREG command format is [0x40 + address, 0x00]  [0x00, 0x00]
//------------------------------------------------------------------------RREG----------------------------------------------------------------
//      __                                                                  _                                                                _
//[CSn]   |________________________________________________________________| |______________________________________________________________|
//           _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _     _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
//[SCLK]____| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_
//              ___
//[SDI] _______|   |_<---ADDRESS[5:0]--->_________________0x00______________________________0x00____________________________0x00______________
//
//[SDO]________________________________________________________________________<----------regValue----------->_<---------regAddress---------->
//--------------------------------------------------------------------------------------------------------------------------------------------
    
    //change page if needed
    if (page != registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS]) {  
        numWords = buildSPIarray(0x80 + PAGE_POINTER_ADDRESS, page);
        spiSendReceiveArrays(dataTx, dataRx, numWords); 
        registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS] = page;       
    }

    //build and send frame
    numWords = buildSPIarray(0x40 + address, 0x00);
    spiSendReceiveArrays(dataTx, dataRx, numWords);
    numWords = buildSPIarray(0x00, 0x00);
    spiSendReceiveArrays(dataTx, dataRx, numWords);      
    
    uint8_t i = 0;
    if (STATUS_ENABLED) {
        REG_DATA.STATUS_MSB = dataRx[0];
        REG_DATA.STATUS_LSB = dataRx[1];    
        i = 2;
    }  

    REG_DATA.Register_data =  dataRx[i];
    
    if (SPI_CRC_ENABLED)
    {
        REG_DATA.CRC_Valid = (dataRx[i+3] == getCRC(dataRx, i+3, CRC_INITIAL_SEED));
    }
    
    registerMap[page][address] = REG_DATA.Register_data;
    return (REG_DATA);
}

void startAdcConversion(void)
{
    writeSingleRegister(GENERAL_SETTINGS, CONVERSION_CTRL_ADDRESS, START_STARTORRESTARTCONVERSIONS);    // write to start bit
   
    return;
}

ADC_IO readData(void)
{
//-------------------------------------------------READ DATA-------------------------------------------------
//      __                                                                                                 __
//[CSn]   |_______________________________________________________________________________________________|
//          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
//[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |___
//
//[DIN] _____________________________________________________________________________________________________
//
//[DOUT]_____<---------------------------------------DATA------------------------------------------------>___
//      _                                                                                                   _
//[DRDY] |_________________________________________________________________________________________________|
//-----------------------------------------------------------------------------------------------------------

// Read data command is a 1 frame operation
// DIN should be set to 0x000000 while clocking 24 SCLK bits

        waitForDRDYinterrupt(1000);     // wait for falling DRDY. 1000tick timeout
        numWords = buildSPIarray(0x00, 0x00);
        spiSendReceiveArrays(dataTx, dataRx, numWords);
        
        uint8_t i = 0;
    if (STATUS_ENABLED) {
        Conversions.STATUS_MSB = dataRx[0];
        Conversions.STATUS_LSB = dataRx[1];    
        i = 2;
    }  

    if (RESOLUTION_IS_24_BIT) {
        int32_t upperByte   = ((uint32_t) dataRx[i+0] << 24);
        int32_t middleByte  = ((uint32_t) dataRx[i+1] << 16);
        int32_t lowerByte   = ((uint32_t) dataRx[i+2] << 8);
        Conversions.ADC_reading= (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);
    } else {
        int32_t upperByte   = ((uint32_t) dataRx[i+0] << 24);
        int32_t middleByte  = ((uint32_t) dataRx[i+1] << 16);
        int32_t lowerByte   = 0x00000000;
        Conversions.ADC_reading= (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);
    }

    if (SPI_CRC_ENABLED)
    {
        Conversions.CRC_Valid = (dataRx[i+3] == getCRC(dataRx, i+3, CRC_INITIAL_SEED));
    }

    return(Conversions);
}

 ADC_IO readFIFO(void)
// Command is sent as 2, 24-bit frames, fifo data is returned in the second frame.
// READ FIFO command format is [0x00, 0x0F, 0x00]  [0x00, 0x00, 0x00]
//------------------------------------------------------------------------READ FIFO----------------------------------------------------------------------------------------------------------------------------
//      __                                                                                                 _                                                                                                 _
//[CSn]   |___________________________________________<24 clks>___________________________________________| |___________________________________________<24 clks>___________________________________________| 
//          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _     _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _ 
//[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_
//                   
//[DIN] ________________0x00____________________________0x0F_(CMD WORD)_______________0x00__________________________________0x00____________________________0x0F_____________________________0x00____________
//
//[DOUT]_______________________________________________________________________________________________________<----------FIFO MSB-----------><----------FIFO MID-----------><----------FIFO LSB----------->____
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

{
        numWords = buildSPIarray(0x0F, 0x00);
        spiSendReceiveArrays(dataTx, dataRx, numWords);
        uint8_t i = 0;
    
        if (STATUS_ENABLED) {
            FIFO.STATUS_MSB = dataRx[0];
            FIFO.STATUS_LSB = dataRx[1];    
            i = 2;
        }  

        if (RESOLUTION_IS_24_BIT) {
            int32_t upperByte   = ((uint32_t) dataRx[i+0] << 24);
            int32_t middleByte  = ((uint32_t) dataRx[i+1] << 16);
            int32_t lowerByte   = ((uint32_t) dataRx[i+2] << 8);
            FIFO.ADC_reading= (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);
        } else {
            int32_t upperByte   = ((uint32_t) dataRx[i+0] << 24);
            int32_t middleByte  = ((uint32_t) dataRx[i+1] << 16);
            int32_t lowerByte   = 0x00000000;
            FIFO.ADC_reading= (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);
        }

        if (SPI_CRC_ENABLED)
        {
            FIFO.CRC_Valid = (dataRx[i+3] == getCRC(dataRx, i+3, CRC_INITIAL_SEED));
        }

    return(FIFO);
}

void stopAdcConversion(void)
{
    writeSingleRegister(GENERAL_SETTINGS, CONVERSION_CTRL_ADDRESS,STOP_STOPCONVERSIONS);   // write to stop bit
    return;
}

STATUS_IO checkStatus ()
// This routine checks the ADC Status words for errors and returns the error typedef ADC_STATUS. 
// The first 2 Status registers contain a summary of error flags currently present in the converter.
// If summary status registers show errors, additional status registers will be read.  

{
#define STATUS_MSB_ERROR_FLAGS ((uint8_t) 0x06)
#define STATUS_LSB_ERROR_FLAGS ((uint8_t) 0x0F)

STATUS_IO status;

// read status registers
    status.STATUS_MSB = readSingleRegister(GENERAL_SETTINGS, STATUS_MSB_ADDRESS).Register_data;
    status.STATUS_LSB = readSingleRegister(GENERAL_SETTINGS, STATUS_LSB_ADDRESS).Register_data;

// check these 2 status registers to see if an error is present.  If so, parse the specific error
status.errorPresent = status.STATUS_MSB + status.STATUS_LSB != STATUS_MSB_ERROR_FLAGS + STATUS_LSB_ERROR_FLAGS? true: false;

// read errors in the STATUS_MSB byte
switch (status.STATUS_MSB & STATUS_MSB_ERROR_FLAGS ^ STATUS_MSB_ERROR_FLAGS)
{
    case RESETN_NORESETOCCURRED:
        // a Reset has occured. 
    break;
    
    case ADC_REF_FAULTN_NOOUTOFRANGEFAULTOCCURRED:  // Read the ADC_REF Status register for more details. 
        status.ADC_REF = readSingleRegister(GENERAL_SETTINGS, ADC_REF_STATUS_ADDRESS).Register_data;
    break;
}

switch (status.STATUS_LSB & STATUS_LSB_ERROR_FLAGS ^ STATUS_LSB_ERROR_FLAGS)
{
    case SPI_CRC_FAULTN_NOSPICRCFAULTOCCURRED:
    // previous spi frame CRC was incorrect
    break;

    case REG_WRITE_FAULTN_NOPAGEORREGISTERACCESSFAULTOCCURRED:
    // out of range register was accesses
    break;

    case INTERNAL_FAULTN_NOINTERNALFAULTOCCURRED:
    status.DIGITAL_STATUS = readSingleRegister(GENERAL_SETTINGS, DIGITAL_STATUS_ADDRESS).Register_data;
    break;

    case FIFO_FAULTN_NOFIFOFAULTOCCURRED:
    status.FIFO = readSingleRegister(GENERAL_SETTINGS, FIFO_SEQ_STATUS_ADDRESS).Register_data;
    break;
}
    return(status);
}

void powerDownMode()
{
    writeSingleRegister(GENERAL_SETTINGS, ADC_CFG_ADDRESS, registerMap[GENERAL_SETTINGS][ADC_CFG_ADDRESS] | PWDN_POWERDOWNMODE);  
}

void activeMode()
{
    writeSingleRegister(GENERAL_SETTINGS, ADC_CFG_ADDRESS, registerMap[GENERAL_SETTINGS][ADC_CFG_ADDRESS] | PWDN_ACTIVE);  
}

int32_t signExtend(const uint8_t dataBytes[])
{
    int32_t upperByte   = ((uint32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((uint32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((uint32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit
}

void enableRegisterMapCrc(bool enable)
{
    // Disable the REG_MAP_CRC_EN bit
    writeSingleRegister(GENERAL_SETTINGS, DIAG_MONITOR_CFG_ADDRESS, (registerMap[GENERAL_SETTINGS][DIAG_MONITOR_CFG_ADDRESS]) & ~REG_MAP_CRC_EN_ENABLED);

    if (!enable) { return; }
     
    // pre-enable REG_MAP_CRC_MASK, this reg 08h is used in CRC calculation. 
    registerMap[GENERAL_SETTINGS][DIAG_MONITOR_CFG_ADDRESS] |= REG_MAP_CRC_EN_ENABLED;

    // Compute and update MAIN_CRC
    uint8_t crc = CRC_INITIAL_SEED; 
    crc = getCRC(&registerMap[GENERAL_SETTINGS][0x12], 0x07, crc);      // registers 12h to 18h 
    crc = getCRC(&registerMap[GENERAL_SETTINGS][0x20], 0x0E, crc);      // registers 20h to 2Dh 
    crc = getCRC(&registerMap[GENERAL_SETTINGS][0x30], 0x03, crc);      // registers 30h to 32h 

    writeSingleRegister(GENERAL_SETTINGS, REG_MAP_CRC_ADDRESS, crc);

    //compute regmap crc values for all step pages
    for ( uint8_t i = 1; i < 31; i++) {
        crc = CRC_INITIAL_SEED;
        crc = getCRC(&registerMap[i][0x00], 0x11, crc);
        writeSingleRegister(i, STEPX_REG_MAP_CRC_ADDRESS, crc);
    }

    // Enable the REG_MAP_CRC_EN bit
    writeSingleRegister(GENERAL_SETTINGS, DIAG_MONITOR_CFG_ADDRESS, registerMap[GENERAL_SETTINGS][DIAG_MONITOR_CFG_ADDRESS] | REG_MAP_CRC_EN_ENABLED);
}

bool isValidCrcOut(void)
{
    bool crc_valid = true;
    if (SPI_CRC_ENABLED)
    {
        uint8_t crcIndex = (STATUS_ENABLED ? 2 : 0) + (RESOLUTION_IS_16_BIT ? 2 : 3);

        // (OPTION 1) Comparing bytes
        //crc_valid = (dataTx[crcIndex] == getCRC8(dataTx, crcIndex, CRC8_INITIAL_SEED));

        // (OPTION 2) Including CRC byte
        crc_valid = !(bool) getCRC(dataTx, crcIndex + 1, CRC_INITIAL_SEED);
    }
    return crc_valid;
}

static uint8_t buildSPIarray(uint8_t byte1, uint8_t byte2)
{
   uint8_t i = 0;  // byte index

    if (STATUS_ENABLED) { 
        dataTx[i++] = 0x00; 
        dataTx[i++] = 0x00;
    }         

    if (RESOLUTION_IS_24_BIT) { 
        dataTx[i++] = 0x00; 
    }
    
    dataTx[i++] = byte1;    // SPI byte 1
    dataTx[i++] = byte2;    // SPI byte 2
    
    if (SPI_CRC_ENABLED) {
        // Compute CRC-IN, ignoring 'don't care' bytes
        dataTx[i++] = getCRC(&dataTx[i - 2], 2, CRC_INITIAL_SEED);
    }
    return i;
}