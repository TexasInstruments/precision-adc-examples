/**
 * @file ads122s1x.h
 *
 * @brief This file contains all basic communication and device setup for the ADS122S14.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include "evm/ads122s1x.h"
#include "evm/hal.h"

const char *adcRegisterNames[NUM_REGISTERS] = {"DEVICE_ID", "REVISION_ID",
     "STATUS_MSB", "STATUS_LSB", "CONVERSION_CTRL", "DEVICE_CFG",
     "DATA_RATE_CFG", "MUX_CFG", "GAIN_CFG", "REFERENCE_CFG","DIGITAL_CFG",
     "GPIO_CFG", "GPIO_DATA_OUTPUT", "IDAC_MAG_CFG", "IDAC_MUX_CFG",
     "REGISTER_MAP_CRC"};


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations */
static uint16_t registerMap[NUM_REGISTERS];
static uint8_t dataTx[6] = {0};
static uint8_t dataRx[6] = {0};
static uint8_t numWords = 0;

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

static void restoreRegisterDefaults(void)
{
// use these registerMap definitions to change your configuration as needed.
   registerMap[DEVICE_ID_ADDRESS]              = DEVICE_ID_DEFAULT;
   registerMap[REVISION_ID_ADDRESS]            = REVISION_ID_DEFAULT;
   registerMap[STATUS_MSB_ADDRESS]             = STATUS_MSB_DEFAULT;
   registerMap[STATUS_LSB_ADDRESS]             = STATUS_LSB_DEFAULT;
   registerMap[CONVERSION_CTRL_ADDRESS]        = CONVERSION_CTRL_DEFAULT;
   registerMap[DEVICE_CFG_ADDRESS]             = DEVICE_CFG_DEFAULT;
   registerMap[DATA_RATE_CFG_ADDRESS]          = DATA_RATE_CFG_DEFAULT;
   registerMap[MUX_CFG_ADDRESS]                = MUX_CFG_DEFAULT;
   registerMap[GAIN_CFG_ADDRESS]               = GAIN_CFG_DEFAULT;
   registerMap[REFERENCE_CFG_ADDRESS]          = REFERENCE_CFG_DEFAULT;
   registerMap[DIGITAL_CFG_ADDRESS]            = DIGITAL_CFG_DEFAULT;
   registerMap[GPIO_CFG_ADDRESS]               = GPIO_CFG_DEFAULT;
   registerMap[GPIO_DATA_OUTPUT_ADDRESS]       = GPIO_DATA_OUTPUT_DEFAULT;
   registerMap[IDAC_MAG_CFG_ADDRESS]           = IDAC_MAG_CFG_DEFAULT;
   registerMap[IDAC_MUX_CFG_ADDRESS]           = IDAC_MUX_CFG_DEFAULT;
   registerMap[REGISTER_MAP_CRC_ADDRESS]       = REGISTER_MAP_CRC_DEFAULT;
}



void initADC()
{
    writeSingleRegister(MUX_CFG_ADDRESS, MUX_CFG_AINN_GND);                 // set inputs to AIN0-GND // will do this in configuration functions
    writeSingleRegister(REFERENCE_CFG_ADDRESS, REFERENCE_CFG_REF_VAL_2);     // set internal VREF value to 2.5V
    //writeSingleRegister(DEVICE_CFG_ADDRESS, DEVICE_CFG_CONV_MODE_SINGLE);    //set to single-shot mode
    writeSingleRegister(DATA_RATE_CFG_ADDRESS, DATA_RATE_CFG_FLTR_OSR_128); //set to 250SPS (default = 2000), fmod=32kHz, OSR=128 
}

//Angel additions 
void powerDownMode(){
    writeSingleRegister(DEVICE_CFG_ADDRESS, DEVICE_CFG_PWDN_PWDN);     // power down ADC
}

void activeMode(){
    writeSingleRegister(DEVICE_CFG_ADDRESS, DEVICE_CFG_PWDN_NO_PWDN);     // active mode ADC
}

void configurationA(){
    writeSingleRegister(MUX_CFG_ADDRESS, MUX_CFG_AINN_GND);                 // set inputs to AIN0-GND (single-ended)
    //writeSingleRegister(GAIN_CFG_ADDRESS, GAIN_CFG_GAIN_1);               //set PGA gain to 1 (default)
}

void configurationB()
{
    writeSingleRegister(MUX_CFG_ADDRESS, MUX_CFG_AINP_AIN1 | MUX_CFG_AINN_AIN2);                 // set inputs to AIN1-AIN2 (differential)
    //writeSingleRegister(GAIN_CFG_ADDRESS, GAIN_CFG_GAIN_10);     // set PGA gain to 10
}



uint8_t readSingleRegister(uint8_t address)
{
    // Command is sent as 2, 16-bit frames, register data is returned in the first byte of frame #2.
    // ADS122s1x RREG command format is [0x40 + address, 0x00]  [0x00, 0x00]
    //------------------------------------------------------------------------RREG----------------------------------------------------------------
    //      __                                                                  _                                                                _
    //[CSn]   |________________________________________________________________| |______________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |__| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |___
    //              ___
    //[DIN] _______|   |_<---ADDRESS[5:0]--->_________________0x00______________________________0x00____________________________0x00______________
    //
    //[DOUT]_______________________________________________________________________<----------regValue----------->________________________________
    //--------------------------------------------------------------------------------------------------------------------------------------------

    uint16_t regValue = 0;

    dataTx[0] = (0x40 + address);
    dataTx[1] = 0;
    numWords = 2;

    spiSendReceiveArrays(dataTx, dataRx, numWords);
    delay_cycles(SPI_DELAY);

    //new frame
    dataTx[0] = 0;
    dataTx[1] = 0;
    numWords = 2;

    spiSendReceiveArrays(dataTx, dataRx, numWords);
    delay_cycles(SPI_DELAY);

    regValue =  dataRx[0];
    registerMap[address] = regValue;
    return regValue;
}



bool writeSingleRegister(uint8_t address, uint8_t data)
{
    //ADS122s1x write format is [0x80 | address (byte 1), Data (byte 2)]
    //----------------------------------------WREG----------------------------------
    //      __                                                                     _
    //[CSn]   |___________________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |______
    //          ___
    //[DIN] ___|   |___<---ADDRESS[5:0]--->____<-----------DATA-------------->______
    //------------------------------------------------------------------------------

    dataTx[0] = 0x80 + address;
    dataTx[1] = data;
    numWords = 2;

      spiSendReceiveArrays(dataTx, dataRx, numWords);
      delay_cycles(SPI_DELAY);

    return 0;
}


bool resetDevice(void)
{
    // write value 0x5C to Control register 0x04
    // this frame then becomes 0x845C
    //--------------------------------------RESET-----------------------------------
    //      __                                                                     _
    //[CSn]   |___________________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |______
    //          ___                 ___             ___     ___________
    //[DIN] ___|   |_______________|   |___________|   |___|           |____________
    //------------------------------------------------------------------------------

    dataTx[0] = 0x84;
    dataTx[1] = 0x5C;
    numWords = 2;

    spiSendReceiveArrays(dataTx, dataRx, numWords);
    delay_cycles(SPI_DELAY);

    // wait for reset
   
    return 0;
}

void startAdcConversion(void)
{
    // write Start bit to Conversion Control register
    //---------------------------------------START----------------------------------
    //      __                                                                     _
    //[CSn]   |___________________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |______
    //          ___                 ___                                 ___
    //[DIN] ___|   |_______________|   |_______________________________|   |________
    //------------------------------------------------------------------------------

    dataTx[0] = 0x80 | CONVERSION_CTRL_ADDRESS;
    dataTx[1] = CONVERSION_CTRL_START_START;
    numWords = 2;

    spiSendReceiveArrays(dataTx, dataRx, numWords);
    delay_cycles(SPI_DELAY);

    return;
}

int32_t readData(void)
{
    //---------------------------------------READ DATA-----------------------------------------------------------
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


    dataTx[0] = 0x00;
    dataTx[1] = 0x00;
    dataTx[2] = 0x00;
    numWords = 3;

    

    // Read data command is a 1 frame operation
    // DIN should be set to 0x000000 while clocking 24 SCLK bits
    // FYI, max data rate is 128KSPS, which converts to a 7.8us delay between samples in continuous mode.

  //  waitForDRDYinterrupt(1000);
    // Read conversion results

   
    spiSendReceiveArrays(dataTx, dataRx, numWords);
    delay_cycles(SPI_DELAY);


    return (signExtend(dataRx));
}

void stopAdcConversion(void)
{
    // write Stop bit to Conversion Control register.
    //---------------------------------------STOP-----------------------------------
    //      __                                                                     _
    //[CSn]   |___________________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |______
    //          ___                 ___                                     ___
    //[DIN] ___|   |_______________|   |___________________________________|   |____
    //------------------------------------------------------------------------------

    dataTx[0] = 0x80 | CONVERSION_CTRL_ADDRESS;
    dataTx[1] = CONVERSION_CTRL_STOP_STOP;
    numWords = 2;

    spiSendReceiveArrays(dataTx, dataRx, numWords);
    delay_cycles(SPI_DELAY);
    return;
}

int32_t signExtend(const uint8_t dataBytes[])
{
    int32_t upperByte   = ((uint32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((uint32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((uint32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit
}






