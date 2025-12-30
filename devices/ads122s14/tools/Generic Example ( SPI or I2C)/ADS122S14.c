/**
 * @file ads122s1x.h
 *
 * @brief This file contains all basic communication and device setup for the ADS122y14 device family.
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
static bool DRDY = false;
static bool timeout = false;

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

static void restoreRegisterDefaults(void)
{
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




uint8_t readSingleRegister(uint8_t address)
{
    //ADS122s1x read format is [0x40 | address]   NEXT FRAME  [Data byte]

    uint16_t regValue = 0;
    dataTx[0] = (0x40 + address);
    dataTx[1] = 0;
    numWords = 2;

#ifdef SPI_COMMS
// Command is sent as 2, 16-bit frames, register data is returned in the first byte of frame #2.
// ADS122s1x RREG command format is [0x40 + address, 0x00]  [0x00, 0x00]

//------------------------------------------------------------------------RREG----------------------------------------------------------------
//      __                                                                  _                                                                _
//[CSn]   |________________________________________________________________| |______________________________________________________________|
//          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
//[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |__| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |___
//              ___
//[DIN] _______|   |_<---ADDRESS[5:0]--->_________________0x00______________________________0x00____________________________0x00_______________
//
//[DOUT]_______________________________________________________________________<----------regValue----------->_________________________________
//----------------------------------------------------------------------------------------------------------------------------------------------



    spiSendReceiveArrays(dataTx, dataRx, numWords);

    //new frame
    dataTx[0] = 0;
    dataTx[1] = 0;
    numWords = 2;

    spiSendReceiveArrays(dataTx, dataRx, numWords);


#endif

#ifdef I2C_COMMS

//------------------------------------------------------------------------RREG------------------------------------------------------------------------
//  [SDA]  |Start|   Address[6:0]   |Write|ACK|   Read Register CMD[7:0]   |ACK|Repeat|   Address[6:0]   |Read|ACK|   Register Data[7:0]   |NACK|Stop|
//----------------------------------------------------------------------------------------------------------------------------------------------------
//  [SCL]  |-----[     8-clocks     ]---------[          8-clocks          ]----------[      8-clocks    ]--------[         8-clocks       ]---------|
//----------------------------------------------------------------------------------------------------------------------------------------------------

    transferI2CData(dataTx, 1, dataRx, 1);
#endif

    regValue =  dataRx[0];
    registerMap[address] = regValue;

    return regValue;
}

bool writeSingleRegister(uint8_t address, uint8_t data)
{
    //ADS122s1x write format is [0x80 | address (byte 1), Data (byte 2)]

    dataTx[0] = 0x80 + address;
    dataTx[1] = data;
    numWords = 2;

#ifdef SPI_COMMS
//----------------------------------------WREG----------------------------------
//      __                                                                     _
//[CSn]   |___________________________________________________________________|
//          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
//[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |__
//          ___
//[DIN] ___|   |___<---ADDRESS[5:0]--->____<-----------DATA-------------->__
//------------------------------------------------------------------------------

  //  setCS(0);
    spiSendReceiveArrays(dataTx, dataRx, numWords);
  //  setCS(1);
#endif

#ifdef I2C_COMMS

//------------------------------------------------------WREG-------------------------------------------------------
//  [SDA]  |Start|   Address[6:0]   |Write|ACK|   Write Register CMD[7:0]   |ACK|   Register Data[7:0]   |ACK|Stop|
//-----------------------------------------------------------------------------------------------------------------
//  [SCL]  |-----[     8-clocks     ]----------[          8-clocks          ]----[         8-clocks      ]--------|
//-----------------------------------------------------------------------------------------------------------------

    transferI2CData(dataTx, 2, dataRx, 0);
#endif


    registerMap[address] = data;        // update internal register map variables.
    return 0;
}

void initAdcConfig(void)
{
    writeSingleRegister(DEVICE_CFG_ADDRESS, DEVICE_CFG_SPEED_MODE_HIGH);    // High Speed mode
    writeSingleRegister(DATA_RATE_CFG_ADDRESS, DATA_RATE_CFG_FLTR_OSR_1024);  // set to OSR = 1024;
    writeSingleRegister(GAIN_CFG_ADDRESS, GAIN_CFG_SYS_MON_ALDO_DIV4);      // SYS_MON set to ALDO/4

}

bool resetDevice(void)
{
    // write value 0x58 to Control register 0x04

    writeSingleRegister(CONVERSION_CTRL_ADDRESS, 0x58);

    // wait for reset
    delay_us(300);
    return 0;
}

void startAdcConversion(void)
{
    // write Start bit to Conversion Control register
    writeSingleRegister(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);
}


int32_t readDataDirect(void)
{
    // reads 24-bit ADC conversion immediately.  does not wait for any DRDY
    dataTx[0] = 0x00;
    dataTx[1] = 0x00;
    dataTx[2] = 0x00;
    numWords = 3;

#ifdef SPI_COMMS

    //------------------------------------------------Read ADC--------------------------------------------------
    //      __                                                                                                 _
    //[CSn]   |_______________________________________________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_
    //
    //[DOUT] ___<------------------------------------DATA[23:0]------------------------------------------------>
    //----------------------------------------------------------------------------------------------------------




    // Read data command is a 1 frame operation
    // DIN should be set to 0x000000 while clocking 24 SCLK bits
    // FYI, max data rate is 128KSPS, which converts to a 7.8us delay between samples in continuous mode.

    spiSendReceiveArrays(dataTx, dataRx, numWords);
#endif

#ifdef I2C_COMMS

//-----------------------------------------------------------Read ADC-------------------------------------------------------------
//  [SDA]  |Start|   Address[6:0]   |Read|ACK|   Conv Data[23:16]   |ACK|   Conv Data[15:8]   |ACK|   Conv Data[7:0]   |NACK|Stop|
//--------------------------------------------------------------------------------------------------------------------------------
//  [SCL]  |-----[     8-clocks     ]---------[       8-clocks      ]----[       8-clocks     ]---[       8-clocks     ]---------|
//--------------------------------------------------------------------------------------------------------------------------------

    transferI2CData(dataTx, 0, dataRx, 3);
#endif

    return (signExtend(dataRx));
}


int32_t readData(void)
{
    dataTx[0] = 0x00;
    dataTx[1] = 0x00;
    dataTx[2] = 0x00;
    numWords = 3;

#ifdef SPI_COMMS

    //------------------------------------------------Read ADC--------------------------------------------------
    //      __                                                                                                 _
    //[CSn]   |_______________________________________________________________________________________________|
    //          _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
    //[SCLK]___| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_
    //
    //[DOUT] ___<------------------------------------DATA[23:0]------------------------------------------------>
    //----------------------------------------------------------------------------------------------------------


    // adjust CSn pin pad type to pull down and observe DRDY signal.
    GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_CFG_OUT_STR_MED, GPIO_PIN_TYPE_STD_WPD);

    // Read data command is a 1 frame operation
    // DIN should be set to 0x000000 while clocking 24 SCLK bits
    // FYI, max data rate is 128KSPS, which converts to a 7.8us delay between samples in continuous mode.

    waitForDRDYinterrupt(1000);
    // Read conversion results

    // return bus configuration to allow CS to be pulled up high
    GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_CFG_OUT_STR_MED, GPIO_PIN_TYPE_STD_WPU);
    spiSendReceiveArrays(dataTx, dataRx, numWords);

return (signExtend(dataRx));

#endif

#ifdef I2C_COMMS

//-----------------------------------------------------------Read ADC-------------------------------------------------------------
//  [SDA]  |Start|   Address[6:0]   |Read|ACK|   Conv Data[23:16]   |ACK|   Conv Data[15:8]   |ACK|   Conv Data[7:0]   |NACK|Stop|
//--------------------------------------------------------------------------------------------------------------------------------
//  [SCL]  |-----[     8-clocks     ]---------[       8-clocks      ]----[       8-clocks     ]---[       8-clocks     ]---------|
//--------------------------------------------------------------------------------------------------------------------------------

   // the DRDYn signal is not available on the I2C variant of the EVM, therefore we will use the STATUS_MSB flag <DRDY> to
   // gait new data collection.

    uint8_t ADCData[3];
    writeSingleRegister(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_STATUS_EN_MASK | registerMap[DIGITAL_CFG_ADDRESS]); // Enable Status words
    transferI2CData(dataTx, 0, dataRx, 5);
    DRDY = dataRx[0] & STATUS_MSB_DRDY_DATA_NEW;
    if(!DRDY)
    {
        timeout = 0;
        do{
            delay_us(1);
            transferI2CData(dataTx, 0, dataRx, 5);          // get new i2c data
            DRDY = dataRx[0] & STATUS_MSB_DRDY_DATA_NEW;    // check if data is new
            timeout++;
        }
        while (!DRDY | timeout < 1000 );                        // exit loop if drdy is new, or loop exceeds ~1000ms.
    }
            ADCData[0] = dataRx[2];
            ADCData[1] = dataRx[3];
            ADCData[2] = dataRx[4];
            writeSingleRegister(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_STATUS_EN_MASK ^ registerMap[DIGITAL_CFG_ADDRESS]); // Disable Status words

            return(signExtend(ADCData));
#endif


}


void stopAdcConversion(void)
{
    // write Stop bit to Conversion Control register.
    writeSingleRegister(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_STOP_STOP);
}

int32_t signExtend(const uint8_t dataBytes[])
{
#ifdef EXAMPLE_CODE
    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);

    return (((int32_t) (upperByte | lowerByte)) >> 16);                 // Right-shift of signed data maintains signed bit

#else

#ifdef WORD_LENGTH_24BIT

    int32_t upperByte   = ((uint32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((uint32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((uint32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit


#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND

    int32_t signByte    = ((int32_t) dataBytes[0] << 24);
    int32_t upperByte   = ((int32_t) dataBytes[1] << 16);
    int32_t middleByte  = ((int32_t) dataBytes[2] << 8);
    int32_t lowerByte   = ((int32_t) dataBytes[3] << 0);

    return (signByte | upperByte | middleByte | lowerByte);

#elif defined WORD_LENGTH_32BIT_ZERO_PADDED

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit

#elif defined WORD_LENGTH_16BIT_TRUNCATED

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);

    return (((int32_t) (upperByte | lowerByte)) >> 16);                 // Right-shift of signed data maintains signed bit

#endif


#endif
}
