/**
 * @brief This file contains all basic communication and device setup.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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

#include "Driver/ads1285.h" 
#include "Driver/hal.h"
#include "ads1285.h"
#include "ti/driverlib/m0p/dl_core.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

const char *generalRegisterNames[NUM_REGISTERS] = {"ID_SYNC", \
"CONFIG0", "CONFIG1", "HPF0", "HPF1", "OFFSET0", "OFFSET1", "OFFSET2", \
"GAIN0", "GAIN1", "GAIN2", "GPIO", "SRC0", "SRC1", };

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations
static uint8_t registerMap[NUM_REGISTERS];
static uint8_t dataTx[16] = {0};
static uint8_t dataRx[16] = {0};    
static uint8_t numWords = 0;
uint32_t numSamples = 0;

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

//*****************************************************************************
//! Example initialization up sequence.
//!
//! \fn void adcStartupRoutine(void)
//!
//! In this function, set the ADCs internal registers to the 
//! desired values.  re-call this function anytime the ADC is
//! powered on or reset by a pin or SPI command. 
//!
//! \return None.
//*****************************************************************************
void initADC()
{
    resetDevice();
       
    if (MCLK_SPEED < ((double_t) 8192000) )
    {   
        //set to fastest Data Rate and low power mode
        writeSingleRegister(CONFIG0_ADDRESS, CONFIG0_DEFAULT & ~CONFIG0_DR_MASK | CONFIG0_DR_2000SPS_LOW|CONFIG0_MODE_LOW); 
    } else 
    {
        //set to fastest Data Rate and High power mode
        writeSingleRegister(CONFIG0_ADDRESS, CONFIG0_DEFAULT & ~CONFIG0_DR_MASK | CONFIG0_DR_4000SPS_HIGH_MID);
    }
    
    //set Reference to 4.096V mode
    writeSingleRegister(CONFIG1_ADDRESS, CONFIG1_REF_4 | CONFIG1_DEFAULT); 
}

//*****************************************************************************
//! Function for sending single byte SPI commands to the ADC
//!
//! \fn uint8_t sendCommand( uint8_t CMD)
//!
//! \param CMD the command byte/opcode.
//!
//! NOTE: Multi-byte SPI commands have their own dedicated functions.
//!
//! \return None.
//*****************************************************************************
void sendCommand(uint8_t CMD)
{
    dataTx[0] = CMD;
    spiSendReceiveArrays(dataTx, dataRx, 1);

    //******Supported Commands******//
    // WAKEUP_CMD        0x01
    // STANDBY_CMD       0x03
    // SYNC_CMD          0x05
    // RESET_CMD         0x07
    // RDATA_CMD         0x12
    // RREG_CMD          0x20
    // WREG_CMD          0x40
    // OFSCAL_CMD        0x60
    // GANCAL_CMD        0x61
}

//*****************************************************************************
//! Function for resetting the ADC via SPI command.  
//!
//! \fn void resetDevice(void)
//!
//!
//! NOTE: after reset, this command will 'delay();' for a time to allow for  
//! proper reset.  This delay time is determined by the MCLK_SPEED 
//!
//! \return None.
//*****************************************************************************
void resetDevice(void)
{
    sendCommand(RESET_CMD);     // send SPI reset command
    restoreRegisterDefaults();  // reset shadow memory
    
    double_t delay_time =  516874 / (MCLK_SPEED); // compute reset time
    delay_time = round(delay_time * 1000); // convert to ms
    delay_ms( (uint32_t)delay_time );        // wait for reset
}

//*****************************************************************************
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(uint8_t address, const uint8_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! \return None.
//*****************************************************************************
void writeSingleRegister( uint8_t address, uint8_t data)
{   //The write register CMD format is [0x40 | address, 0x00, value]
    //build and send frame
    dataTx[0] = WREG_CMD + address;
    dataTx[1] = 0x00;
    dataTx[2] = data;
    spiSendReceiveArrays(dataTx, dataRx, 3);
    registerMap[address] = data;
}

//*****************************************************************************
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint8_t readSingleRegister(uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//*****************************************************************************
uint8_t readSingleRegister(uint8_t address)
{   // RREG command format is [0x20 + address, 0x00, 0x00]
    //build and send frame
    dataTx[0] = RREG_CMD + address;
    dataTx[1] = 0x00;
    dataTx[2] = 0x00;
    spiSendReceiveArrays(dataTx, dataRx, 3);
    
    registerMap[address] = dataRx[2];
    return (dataRx[2]);
}

//*****************************************************************************
//! Reads multiple sequential registers starting at the specified address.
//!
//! \fn uint8_t readMultipleRegisters(uint8_t address, uint8_t numRegs)
//!
//! \param address is the 8-bit address of the register to read.
//! \param numRegs is teh number of registers to be read
//!
//! \return no return
//*****************************************************************************
void readMultipleRegisters(uint8_t address, uint8_t numRegs)
{   // RREG command format is [0x20 + address, numRegs-1, 0x00...]
    //build and send frame
    dataTx[0] = RREG_CMD + address;
    dataTx[1] = numRegs - 1;
    dataTx[2] = 0x00;
    dataTx[3] = 0x00;
    spiSendReceiveArrays(dataTx, dataRx, numRegs + 2);
    
    for (uint8_t i = 0; i < numRegs; i++)
    {
        registerMap[address + i] = dataRx[i+2];
    }
}

//*****************************************************************************
//! Function for retrieving ADC conversion results without a command
//!
//! \fn uint32_t readDataDirect(void)
//!
//! \return 32-bit ADC data.
//!
//*****************************************************************************
uint32_t readDataDirect(void)
{
// Read data command is a 1 frame operation
// DIN should be set to 0x00000000 while clocking 32 SCLK bits

        dataTx[0] = 0x00;
        dataTx[1] = 0x00;
        dataTx[2] = 0x00;
        dataTx[3] = 0x00;

        waitForDRDYinterrupt(1000);     // wait for falling DRDY. 1000tick timeout
        
        spiSendReceiveArrays(dataTx, dataRx, 4);
        
        int32_t highByte    = ((uint32_t) dataRx[0] << 24);
        int32_t upperByte   = ((uint32_t) dataRx[1] << 16);
        int32_t middleByte  = ((uint32_t) dataRx[2] << 8);
        int32_t lowerByte   = ((uint32_t) dataRx[3] << 0);

        int32_t ADCreading =  (highByte | upperByte | middleByte | lowerByte);
   
    return(ADCreading);
}

//*****************************************************************************
//! Function for retrieving ADC conversion results with Command
//!
//! \fn uint32_t readDataCMD(void)
//!
//! \return 32-bit ADC data.
//!
//*****************************************************************************
uint32_t readDataCommand(void)
{
// Read data command is a 1 frame operation
// DIN should be set to 0x12000000 while clocking 32 SCLK bits

        waitForDRDYinterrupt(1000);     // wait for falling DRDY. 1000tick timeout
        
        dataTx[0] = RDATA_CMD;
        dataTx[1] = 0x00;
        dataTx[2] = 0x00;
        dataTx[3] = 0x00;
        dataTx[4] = 0x00;

        spiSendReceiveArrays(dataTx, dataRx, 4);
        
        int32_t highByte    = ((uint32_t) dataRx[1] << 24);
        int32_t upperByte   = ((uint32_t) dataRx[2] << 16);
        int32_t middleByte  = ((uint32_t) dataRx[3] << 8);
        int32_t lowerByte   = ((uint32_t) dataRx[4] << 0);

        int32_t ADCreading =  (highByte | upperByte | middleByte | lowerByte);
   
    return(ADCreading);
}

//*****************************************************************************
//
//! Reverts internal variables to default state after a reset.
//!
//! \warning This function should only be called if the ADS1285 is reset by
//! some external method 
//! \fn void _restoreRegisterDefaults(void)
//!NOTES:
//! This functionshould be called after powering up or resetting the device, 
//! toggling the nRESET pin or sending the SPI "RESET" command).
//! \return None.
//
//*****************************************************************************
void restoreRegisterDefaults(void)
{
    // use these registerMap definitions to change your configuration as needed.
    registerMap[ID_SYNC_ADDRESS]    = ID_SYNC_DEFAULT;
    registerMap[CONFIG0_ADDRESS]    = CONFIG0_DEFAULT;
    registerMap[CONFIG1_ADDRESS]    = CONFIG1_DEFAULT;
    registerMap[HPF0_ADDRESS]       = HPF0_DEFAULT;
    registerMap[HPF1_ADDRESS]       = HPF1_DEFAULT;
    registerMap[OFFSET0_ADDRESS]    = OFFSET0_DEFAULT; 
    registerMap[OFFSET1_ADDRESS]    = OFFSET1_DEFAULT; 
    registerMap[OFFSET2_ADDRESS]    = OFFSET2_DEFAULT; 
    registerMap[GAIN0_ADDRESS]      = GAIN0_DEFAULT;
    registerMap[GAIN1_ADDRESS]      = GAIN1_DEFAULT;
    registerMap[GAIN2_ADDRESS]      = GAIN2_DEFAULT;
    registerMap[GPIO_ADDRESS]       = GPIO_DEFAULT;
    registerMap[SRC0_ADDRESS]       = SRC0_DEFAULT;
    registerMap[SRC1_ADDRESS]       = SRC1_DEFAULT;
}

//*****************************************************************************
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint8_t getRegisterValue(const uint8_t address)
//!
//! NOTE: The internal registerMap arrays stores the last know register value.
//!
//! \return unsigned 8-bit register value.
//*****************************************************************************
uint8_t getRegisterValue(uint8_t address)
{
    return registerMap[address];
}

//*****************************************************************************
//! Function to return the Data rate period (s)
//!
//! \fn double_t calcFdataPeriod(int CONFIG0_DR)
//!
//!
//! \return double Fdata period.
//*****************************************************************************
double_t calcFdataPeriod(int CONFIG0_DR)
{   
    double_t FdataPeriod = 0;
    switch (CONFIG0_DR) {
        case 0x00:
            FdataPeriod =  32768 / MCLK_SPEED;
            return(FdataPeriod);
            break;
        
        case 0x08:
            FdataPeriod =  16384 / MCLK_SPEED;
            return(FdataPeriod);
            break;  
            
        case 0x10:
            FdataPeriod = 8192 / MCLK_SPEED;
            return(FdataPeriod);
            break;
            
        case 0x18:
            FdataPeriod =  4096 / MCLK_SPEED;
            return(FdataPeriod);
            break;
            
        case 0x20:
            FdataPeriod = 2048 / MCLK_SPEED;
            return(FdataPeriod);
            break;
        default:
            return(0);
    }

}