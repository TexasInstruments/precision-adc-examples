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

#include "Driver/ads124s18.h"         
#include "Driver/ads124s18_pages.h"
#include "Driver/ads124s18_page0.h"
#include "Driver/ads124s18_page1.h"
#include "Driver/hal.h"
#include "Driver/crc.h"
#include "ti/driverlib/m0p/dl_core.h"
#include <stdint.h>
#include <stdbool.h>

// Register configuration tool available here:
// https://dev.ti.com/gallery/view/PADC/PADC_Design_Calculator_Tool/?device=ADS124S18
#include "Driver/ADS124S18_RegisterDetails.h"  



static uint8_t buildSPIarrayforREG(uint8_t byte1, uint8_t byte2);
static uint8_t buildSPIarrayforDATA(uint8_t byte1, uint8_t byte2);


const char *generalRegisterNames[NUM_GENERAL_REGISTERS] = {"DEVICE_ID", \
"REVISION_ID", "STATUS_MSB", "STATUS_LSB", "SUPPLY_STATUS", "ADC_STATUS" \
"DIGITAL_STATUS", "COMP_ALERT_STATUS", "GPIO_DATA_INPUT", "FIFO_SEQ_STATUS", \
"CONVERSION_CTRL", "RESET", "ADC_CFG", "DIGITAL_CFG", "VBIAS",  \
"GPIO_CFG0", "GPIO_CFG1", "GPIO_CFG2", "MONITOR_CFG", "SPARE_CFG", \
"SEQUENCER_CFG", "SEQUENCE_STEP_EN_0", "SEQUENCE_STEP_EN_1", \
"SEQUENCE_STEP_EN_2", "REG_MAP_CRC", "PAGE_INDICATOR", "PAGE_POINTER" 
};

const char *stepxRegisterNames[NUM_STEP_REGISTERS] = {"STEPX_AINP_CFG", \
"STEPX_AINN_CFG", "STEPX_GAIN_REF_CFG", "STEPX_DIGITAL_CFG", "STEPX_FLTR_CFG", \
"STEPX_DELAY_CFG", "STEPX_OFFSET_CAL_MSB", "STEPX_OFFSET_CAL_ISB", \
"STEPX_OFFSET_CAL_LSB", "STEPX_GAIN_CAL_MSB", "STEPX_GAIN_CAL_LSB", \
"STEPX_OW_SYSMON_CFG", "STEPX_IDAC_MAG_CFG", "STEPX_IDAC1_MUX_CFG", "STEPX_IDAC2_MUX_CFG", \
"STEPX_GPIO_DATA_OUT", "STEPX_COMPARATOR_HIGH_THRESHOLD_MSB", "STEPX_COMPARATOR_HIGH_THRESHOLD_LSB", \
"STEPX_COMPARATOR_LOW_THRESHOLD_MSB", "STEPX_COMPARATOR_LOW_THRESHOLD_LSB", "STEPX_COMPARATOR_CFG", \
"STEPX_REG_MAP_CRC", "STEPX_PAGE_INDICATOR", "STEPX_PAGE_POINTER"
};

const char *pageNames[NUM_PAGES] = {"GENERAL_SETTINGS", \
"STEP_0", "STEP_1", "STEP_2", "STEP_3", "STEP_4", "STEP_5", "STEP_6", "STEP_7", \
"STEP_8", "STEP_9", "STEP_10", "STEP_11", "STEP_12", "STEP_13", "STEP_14", \
"STEP_15", "STEP_16", "STEP_17", "STEP_18", "STEP_19", "STEP_20", "STEP_21", \
"STEP_22", "STEP_23"
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
uint32_t numSamples = 20;
uint8_t streamRx[6];
REG_IO REG_DATA;
ADC_IO Conversions;


//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

static void restoreRegisterDefaults(void)
{
    // use these registerMap definitions to change your configuration as needed.
    registerMap[GENERAL_SETTINGS][DEVICE_ID_ADDRESS]              = DEVICE_ID_DEFAULT;
    registerMap[GENERAL_SETTINGS][REVISION_ID_ADDRESS]            = 0x00;
    registerMap[GENERAL_SETTINGS][STATUS_MSB_ADDRESS]             = STATUS_MSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][STATUS_LSB_ADDRESS]             = STATUS_LSB_DEFAULT;
    registerMap[GENERAL_SETTINGS][SUPPLY_STATUS_ADDRESS]          = SUPPLY_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][ADC_STATUS_ADDRESS]             = ADC_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][DIGITAL_STATUS_ADDRESS]         = DIGITAL_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][COMP_ALERT_STATUS_ADDRESS]      = COMP_ALERT_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_DATA_INPUT_ADDRESS]        = GPIO_DATA_INPUT_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQ_STATUS_ADDRESS]             = SEQ_STATUS_DEFAULT;
    registerMap[GENERAL_SETTINGS][CONVERSION_CTRL_ADDRESS]        = CONVERSION_CTRL_DEFAULT;
    registerMap[GENERAL_SETTINGS][RESET_ADDRESS]                  = RESET_DEFAULT;
    registerMap[GENERAL_SETTINGS][ADC_CFG_ADDRESS]                = ADC_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][DIGITAL_CFG_ADDRESS]            = DIGITAL_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][VBIAS_ADDRESS]                  = VBIAS_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_CFG0_ADDRESS]              = GPIO_CFG0_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_CFG1_ADDRESS]              = GPIO_CFG1_DEFAULT;
    registerMap[GENERAL_SETTINGS][GPIO_CFG2_ADDRESS]              = GPIO_CFG2_DEFAULT;
    registerMap[GENERAL_SETTINGS][MONITOR_CFG_ADDRESS]            = MONITOR_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][0x17]                           = 0x00;
    registerMap[GENERAL_SETTINGS][SPARE_ADDRESS]                  = SPARE_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCER_CFG_ADDRESS]          = SEQUENCER_CFG_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_0_ADDRESS]     = SEQUENCE_STEP_EN_0_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_1_ADDRESS]     = SEQUENCE_STEP_EN_1_DEFAULT;
    registerMap[GENERAL_SETTINGS][SEQUENCE_STEP_EN_2_ADDRESS]     = SEQUENCE_STEP_EN_2_DEFAULT;
    registerMap[GENERAL_SETTINGS][0x1D]                           = 0x00;
    registerMap[GENERAL_SETTINGS][0x1E]                           = 0x00;
    registerMap[GENERAL_SETTINGS][0x1F]                           = 0x00;
    registerMap[GENERAL_SETTINGS][REG_MAP_CRC_ADDRESS]            = REG_MAP_CRC_DEFAULT;
    registerMap[GENERAL_SETTINGS][PAGE_INDICATOR_ADDRESS]         = PAGE_INDICATOR_DEFAULT;
    registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS]           = PAGE_POINTER_DEFAULT;

    for ( uint8_t STEPX = 1; STEPX < 24; STEPX++ )  {
        registerMap[STEPX][STEPX_AINP_CFG_ADDRESS]                = STEPX_AINP_CFG_DEFAULT;
        registerMap[STEPX][STEPX_AINN_CFG_ADDRESS]                = STEPX_AINN_CFG_DEFAULT;
        registerMap[STEPX][STEPX_GAIN_REF_CFG_ADDRESS]            = STEPX_GAIN_REF_CFG_DEFAULT;
        registerMap[STEPX][STEPX_DIGITAL_CFG_ADDRESS]             = STEPX_DIGITAL_CFG_DEFAULT;
        registerMap[STEPX][STEPX_FLTR_CFG_ADDRESS]                = STEPX_FLTR_CFG_DEFAULT;
        registerMap[STEPX][STEPX_DELAY_CFG_ADDRESS]               = STEPX_DELAY_CFG_DEFAULT;
        registerMap[STEPX][STEPX_OFFSET_CAL_MSB_ADDRESS]          = STEPX_OFFSET_CAL_MSB_DEFAULT;
        registerMap[STEPX][STEPX_OFFSET_CAL_ISB_ADDRESS]          = STEPX_OFFSET_CAL_ISB_DEFAULT;
        registerMap[STEPX][STEPX_OFFSET_CAL_LSB_ADDRESS]          = STEPX_OFFSET_CAL_LSB_DEFAULT;
        registerMap[STEPX][STEPX_GAIN_CAL_MSB_ADDRESS]            = STEPX_GAIN_CAL_MSB_DEFAULT;
        registerMap[STEPX][STEPX_GAIN_CAL_LSB_ADDRESS]            = STEPX_GAIN_CAL_LSB_DEFAULT;
        registerMap[STEPX][STEPX_OW_SYSMON_CFG_ADDRESS]           = STEPX_OW_SYSMON_CFG_DEFAULT;
        registerMap[STEPX][STEPX_IDAC_MAG_CFG_ADDRESS]            = STEPX_IDAC_MAG_CFG_DEFAULT;
        registerMap[STEPX][STEPX_IDAC1_MUX_CFG_ADDRESS]           = STEPX_IDAC1_MUX_CFG_DEFAULT;
        registerMap[STEPX][STEPX_IDAC2_MUX_CFG_ADDRESS]           = STEPX_IDAC2_MUX_CFG_DEFAULT;
        registerMap[STEPX][STEPX_GPIO_DATA_OUT_ADDRESS]           = STEPX_GPIO_DATA_OUT_DEFAULT;
        registerMap[STEPX][STEPX_COMPARATOR_HIGH_THRESHOLD_MSB_ADDRESS]           = STEPX_COMPARATOR_HIGH_THRESHOLD_MSB_DEFAULT;
        registerMap[STEPX][STEPX_COMPARATOR_HIGH_THRESHOLD_LSB_ADDRESS]           = STEPX_COMPARATOR_HIGH_THRESHOLD_LSB_DEFAULT;
        registerMap[STEPX][STEPX_COMPARATOR_LOW_THRESHOLD_MSB_ADDRESS]            = STEPX_COMPARATOR_LOW_THRESHOLD_MSB_DEFAULT;
        registerMap[STEPX][STEPX_COMPARATOR_LOW_THRESHOLD_LSB_ADDRESS]            = STEPX_COMPARATOR_LOW_THRESHOLD_LSB_DEFAULT;
        registerMap[STEPX][STEPX_COMPARATOR_CFG_ADDRESS]          = STEPX_COMPARATOR_CFG_DEFAULT;
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
            writeSingleRegister(0, PAGE_POINTER_ADDRESS,ADS124S18_sequencer_config[i].pageNumber);
       
        for (int j = 0; j < ADS124S18_sequencer_config[i].registerCount; j++ )
            {
                writeSingleRegister(i, ADS124S18_sequencer_config[i].configPointer[j].addr,ADS124S18_sequencer_config[i].configPointer[j].value);
            }
    }

    // clear status flags
    writeSingleRegister(GENERAL_SETTINGS, SUPPLY_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, ADC_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, DIGITAL_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, COMP_ALERT_STATUS_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, STATUS_MSB_ADDRESS, 0xFF);
    writeSingleRegister(GENERAL_SETTINGS, STATUS_LSB_ADDRESS, 0xFF);
}

bool resetDevice(void)
{
    writeSingleRegister(GENERAL_SETTINGS, RESET_ADDRESS, RESET_CODE);  
    delay_us(500);     // wait for reset
    restoreRegisterDefaults();
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
        numWords = buildSPIarrayforREG(0x80 + PAGE_POINTER_ADDRESS, page);
        spiSendReceiveArrays(dataTx, dataRx, numWords);   
        registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS] = page;     
    }

    //build and send frame
    numWords = buildSPIarrayforREG(0x80 + address, data);
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
        numWords = buildSPIarrayforREG(0x80 + PAGE_POINTER_ADDRESS, page);
        spiSendReceiveArrays(dataTx, dataRx, numWords); 
        registerMap[GENERAL_SETTINGS][PAGE_POINTER_ADDRESS] = page;       
    }

    //build and send frame
    numWords = buildSPIarrayforREG(0x40 + address, 0x00);
    spiSendReceiveArrays(dataTx, dataRx, numWords);
    numWords = buildSPIarrayforREG(0x00, 0x00);
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
        numWords = buildSPIarrayforDATA(0x00, 0x00);
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

void powerDownMode(void)
{
    // lookup current register value and update register 'Powerdown' bit set high
    writeSingleRegister(GENERAL_SETTINGS, ADC_CFG_ADDRESS, getRegisterValue(GENERAL_SETTINGS, ADC_CFG_ADDRESS) | PWDN_POWERDOWNMODE);           
}

void activeMode(void)
{
    // lookup current register value and update register 'Powerdown' bit set low
    writeSingleRegister(GENERAL_SETTINGS, ADC_CFG_ADDRESS, getRegisterValue(GENERAL_SETTINGS, ADC_CFG_ADDRESS) & ~PWDN_MASK);
}

void startAdcConversion(void)
{
    // lookup current register value and update register value with 'START' bit set high. Negate the STOP bit, as this bit always reads 0
    writeSingleRegister(GENERAL_SETTINGS, CONVERSION_CTRL_ADDRESS, (getRegisterValue(GENERAL_SETTINGS,CONVERSION_CTRL_ADDRESS) & STEP_INIT_MASK) | START_STARTORRESTARTCONVERSIONS);
}

void stopAdcConversion(void)
{   
    // lookup current register value and update register value with 'STOP' bit set high. Negate START bit, as this bit always reads 0
    writeSingleRegister(GENERAL_SETTINGS, CONVERSION_CTRL_ADDRESS, (getRegisterValue(GENERAL_SETTINGS,CONVERSION_CTRL_ADDRESS) & STEP_INIT_MASK)  | STOP_STOPCONVERSIONS);  // start conversion, 
}

STATUS_IO checkStatus (void)
// This routine checks the ADC Status words for errors and returns the error typedef ADC_STATUS. 
// The first 2 Status registers contain a summary of error flags currently present in the converter.
// If summary status registers show errors, additional status registers will be read.  

{
#define STATUS_MSB_ERROR_MASK           ((uint8_t) 0x06)
#define STATUS_MSB_ERROR_MASK_RESET    ((uint8_t) 0x02)
#define STATUS_MSB_ERROR_MASK_SAD      ((uint8_t) 0x04)
#define STATUS_LSB_ERROR_MASK           ((uint8_t) 0x03)
#define STATUS_LSB_ERROR_MASK_REG       ((uint8_t) 0x02)
#define STATUS_LSB_ERROR_MASK_SPI_CRC   ((uint8_t) 0x01)

STATUS_IO status;

// read status registers
    status.STATUS_MSB = readSingleRegister(GENERAL_SETTINGS, STATUS_MSB_ADDRESS).Register_data;
    status.STATUS_LSB = readSingleRegister(GENERAL_SETTINGS, STATUS_LSB_ADDRESS).Register_data;

// check these 2 status registers to see if an error is present.  If so, parse the specific error

    if ( (status.STATUS_MSB & STATUS_MSB_ERROR_MASK) != STATUS_MSB_ERROR_MASK ) 
    {
        if ( (status.STATUS_MSB & STATUS_MSB_ERROR_MASK_RESET) != STATUS_MSB_ERROR_MASK_RESET )
        {
            // A reset has occurred.  
        }

        if ( (status.STATUS_MSB & STATUS_MSB_ERROR_MASK_SAD) != STATUS_MSB_ERROR_MASK_SAD )
        {
            status.SUPPLY_STATUS  = readSingleRegister(GENERAL_SETTINGS, SUPPLY_STATUS_ADDRESS).Register_data;
            status.ADC_STATUS     = readSingleRegister(GENERAL_SETTINGS, ADC_STATUS_ADDRESS).Register_data;
            status.DIGITAL_STATUS = readSingleRegister(GENERAL_SETTINGS, DIGITAL_STATUS_ADDRESS).Register_data;
        }    
    
    }   

    if ( (status.STATUS_LSB & STATUS_LSB_ERROR_MASK) != STATUS_LSB_ERROR_MASK )    
    {
        if ((status.STATUS_LSB & STATUS_LSB_ERROR_MASK_REG) != STATUS_LSB_ERROR_MASK_REG )
        {
            //REG access error. 
        }

        if ((status.STATUS_LSB & STATUS_LSB_ERROR_MASK_SPI_CRC) != STATUS_LSB_ERROR_MASK_SPI_CRC )
        {
            //SPI CRC error occurred. 
        }
    }

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
    writeSingleRegister(GENERAL_SETTINGS, MONITOR_CFG_ADDRESS, (registerMap[GENERAL_SETTINGS][MONITOR_CFG_ADDRESS]) & ~REG_MAP_CRC_EN_ENABLED);

    if (!enable) { return; }
     
    // pre-enable REG_MAP_CRC_MASK, this reg is used in CRC calculation. 
    registerMap[GENERAL_SETTINGS][MONITOR_CFG_ADDRESS] |= REG_MAP_CRC_EN_ENABLED;

    // Compute and update MAIN_CRC
    uint8_t crc = CRC_INITIAL_SEED; 
    crc = getCRC(&registerMap[GENERAL_SETTINGS][0x10], 16, crc);      // registers 10h to 1Fh 
    writeSingleRegister(GENERAL_SETTINGS, REG_MAP_CRC_ADDRESS, crc);

    //compute regmap crc values for all step pages
    for ( uint8_t i = 1; i < 24; i++) {
        crc = CRC_INITIAL_SEED;
        crc = getCRC(&registerMap[i][0x00], 21, crc);
        writeSingleRegister(i, STEPX_REG_MAP_CRC_ADDRESS, crc);
    }

    // Enable the REG_MAP_CRC_EN bit
    writeSingleRegister(GENERAL_SETTINGS, MONITOR_CFG_ADDRESS, registerMap[GENERAL_SETTINGS][MONITOR_CFG_ADDRESS] | REG_MAP_CRC_EN_ENABLED);
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

static uint8_t buildSPIarrayforDATA(uint8_t byte1, uint8_t byte2)
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

static uint8_t buildSPIarrayforREG(uint8_t byte1, uint8_t byte2)
{
   uint8_t i = 0;  // byte index

    if (STATUS_ENABLED) { 
        dataTx[i++] = 0x00; 
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