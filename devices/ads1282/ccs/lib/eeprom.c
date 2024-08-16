/*
 *  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

// Import I2C Driver definitions
#include "eeprom.h"
#include "..\settings.h"
#include "..\evm\hal.h"     // TODO: Remove this dependence, by using TI driver delay functions
extern volatile bool g_EEPROM;
//*****************************************************************************
//
//! \brief Reads the contents from the I2C EEPROM.
//!
//! \fn void  readEEPROM(void)
//!
//! \return int8_t Operation status 0 = success; -1 = NACK; -2 = Empty or invalid EEPROM data
//! Reads the EEPROM data and transmits the data to GUI.If the received data
//! doesn't has the SOE or EOE, then EEPROM not programmed error is returned.
//!
//*****************************************************************************
int8_t readEEPROM(void)
{
    //
    // Initialize the optional I2C bus parameters
    //
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;
    //
    // Open the I2C bus for usage
    //
    I2C_Handle i2cHandle = I2C_open(CONFIG_I2C_0, &params);
    if (g_bEEPROM==false)
    {
        GPION->PUR |= (GPIO_PIN_5 | GPIO_PIN_4);
    }
    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = I2C_SLAVE_ADDRESS;

    //
    // Write array for data to write to the slave, which for the EEPROM read is the page address
    //
    char wData[256];
    //
    // Read data array
    //
    char ui8DataArray[I2C_EEPROM_MEMORY_SIZE+1];
    char data[256];

    int_fast16_t retStatus = 0;
    //
    // Store the Start of Expression and End of Expression for checking whether
    // they are present in read data from EEPROM
    //
    char* SOE = "#SOE;";
    char* EOE = "#EOE";
    char * end;         // Pointer used to mark the end of the string in the char array

    //
    // Initially set readEOE boolean to false. It will be set to true once EOE is
    // read from EEPROM.
    //
    bool readEOE = false;

    uint8_t ui8PageIndex = 0;
    uint16_t ui16MemIndex = 0;
    uint8_t * p_16 = (uint8_t *) &ui16MemIndex;

    //
    // Read the data from EEPROM.
    //
    // Make sure the first data is the end of the string for strcat() function when copying the data
    //
    ui8DataArray[0] = '\0';
    while (ui8PageIndex < (I2C_EEPROM_MEMORY_SIZE / I2C_EEPROM_PAGE_SIZE))
    {
        //
        // Read one page data, 1 page is 32 bytes and there are a total of 4096 pages
        //
        ui16MemIndex = ui8PageIndex * I2C_EEPROM_PAGE_SIZE;
        //
        // Convert the position of ui16MemIndex to the the EEPROM address
        //
        wData[0] = p_16[1];
        wData[1] = p_16[0];
        // Read from the I2C slave device
        transaction.readBuf = data;
        transaction.readCount = I2C_EEPROM_PAGE_SIZE;
        transaction.writeBuf = wData; // this is the page address to issue to the EEPROM to start reading the
        transaction.writeCount = sizeof(ui16MemIndex);
        retStatus = I2C_transfer(i2cHandle, &transaction);

        //
        // Return error, if read failed
        //
        if (retStatus == 0)
        {
            I2C_close(i2cHandle);
            json_error(ERROR_I2C_READ_WRITE_FAILED, "I2C EEPROM read failed");
            return -1;
        }

        //
        // Add the data array just collected to the total DataArray for EEPROM contents
        //
        strcat(ui8DataArray, data);
        //
        // Check SOE present in first page. If not present return error.
        //
        if (ui8PageIndex == 0)
        {
            if ((end = strstr((char *)ui8DataArray, SOE)) == NULL)
            {
                I2C_close(i2cHandle);
                json_error(ERROR_I2C_READ_WRITE_FAILED,"I2C EEPROM not programmed yet");
                return -2;
            }
        }

        ui8PageIndex++;

        //
        // Check EOE was received or not. If received stop the read operation.
        //
        ui8DataArray[ui8PageIndex * I2C_EEPROM_PAGE_SIZE] = '\0';
        if ((end = strstr((char *)&ui8DataArray[I2C_EEPROM_SOE_LEN], EOE)) != NULL)
        {
            *end = '\0';
            readEOE = true;
            break;
        }
    }
    I2C_close(i2cHandle);
    //
    // If EOE detected, send the read data, else send the error.
    //
    if (readEOE)
    {
       // TODO: This assumes that the #SOE; is always at the beginning of the array
       // json_console("EVM EEPROM Data: '%s'", &ui8DataArray[5]);  // Send string, beginning after '#SOE;'
        json_data("{\"eeprom\":\"%s\"}", &ui8DataArray[5]);   // Send string, beginning after '#SOE;'
    }
    else //TODO: Will this ever get to here?
    {
        json_error(ERROR_I2C_READ_WRITE_FAILED, "I2C EEPROM not programmed");
        return -2;
    }

    return 0;
}

//*****************************************************************************
//
//! \brief  Clears the EEPROM by writing all ones.
//!
//! \fn void  clearEEPROM(void)
//!
//! \return int8_t
//
//*****************************************************************************
int8_t clearEEPROM(void)
{
    //
    // Initialize the optional I2C bus parameters
    //
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;
    //
    // Open the I2C bus for usage
    //
    I2C_Handle i2cHandle = I2C_open(CONFIG_I2C_0, &params);
    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = I2C_SLAVE_ADDRESS;
    //
    // Turn off the EEPROM write protection
    //
    writeProtectEnable(false);

    //
    // Set the I2C EEPROM write address as 0x0000
    //
    uint16_t ui16RegAddress = 0x0000;
    int8_t count, retStatus = 0;
    uint8_t ui8PageIndex    = 0;
    uint8_t * p_16 = (uint8_t *) &ui16RegAddress;
    //
    // Write array for data to write to the slave, which for the EEPROM write is the page address
    //
    char wData[256];
    char data[32];
    //
    // Store all ones in the internal buffer along with register address.
    //
    for(count = 2; count<(I2C_EEPROM_PAGE_SIZE + 2); count++)
    {
        wData[count] = 0xFF;
    }

    while (ui8PageIndex < (I2C_EEPROM_MEMORY_SIZE / I2C_EEPROM_PAGE_SIZE))
    {
        //
        // Read one page data, 1 page is 32 bytes and there are a total of 4096 pages
        //
        ui16RegAddress = ui8PageIndex * I2C_EEPROM_PAGE_SIZE;
        //
        // Convert the position of ui16MemIndex to the the EEPROM address
        //
        wData[0] = p_16[1];
        wData[1] = p_16[0];
        // Read from the I2C slave device
        transaction.readBuf = data;
        transaction.readCount = 0;
        transaction.writeBuf = wData; // this is the page address to issue to the EEPROM to start reading the
        transaction.writeCount = sizeof(ui16RegAddress) + I2C_EEPROM_PAGE_SIZE;
        retStatus = I2C_transfer(i2cHandle, &transaction);

        //
        // Return error, if read failed
        //
        if (retStatus == false)
        {
            I2C_close(i2cHandle);
            writeProtectEnable(true);
            json_error(ERROR_I2C_READ_WRITE_FAILED,"I2C EEPROM read failed");
            return -1;
        }
        ui8PageIndex++;
        delay_us(5000);     // Wait for EEPROM to complete write operation.
    }
    // When done close the I2C bus
    I2C_close(i2cHandle);

    //
    // Turn on the EEPROM write protection
    //
    writeProtectEnable(true);
    return 0;
}

/**
 * @fn      void writeProtectEnable(bool enable)
 *
 * @brief   Enables/disables the EEPROM write protection
 *
 * @param   enable  Specifies whether Write protect needs to be enabled.
 *                  If the value is '0' write protection is disabled (allows EEPROM writes),
 *                  if the value is '1' write protection is enabled (EEPROM is read-only).
 *
 * @return  none
 */
int8_t writeProtectEnable(bool enable)
{
    //
    // Write protect is an active HIGH signal -> Set the GPIO output...
    //  ...HIGH to enable write protection or
    //  ...LOW to disable write protection.
    //
    if(enable)
        {
            GPIO_write(WE_CONST, 1);
        }
        else
        {
            GPIO_write(WE_CONST, 0);
        }
    return 0;
}

/**
 * @fn      int8_t readInternalEEPROM(void)
 *
 * @brief   Reads the data from the MSP432E's internal EEPROM peripheral
 *
 * @param   none
 *
 * @return  int8_t
 */
int8_t readInternalEEPROM(void)
{
    uint32_t retInitStatus, i, j;
    uint32_t data32[64];
    char data8[257];

    //
    // Initially set readEOE boolean to false. It will be set to true once EOE is
    // read from EEPROM.
    //
    bool readEOE = false;

    //
    // Store the Start of Expression and End of Expression for checking whether
    // they are present in read data from EEPROM
    //
    char* SOE = "#SOE;";
    char* EOE = "#EOE";
    char * end;


    // Enable the EEPROM Module and Initialize the EEPROM Block
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)))
    {
    }

    // Initialize EEPROM peripheral
    retInitStatus = EEPROMInit();
    if(retInitStatus != EEPROM_INIT_OK)
    {
        json_error(1, "MSP432E EEPROM did not initialize");
    }

    // Read the EEPROM to an array
    EEPROMRead(&data32[0], 0, 256);
    i=0;
    for (j = 0; j < 256; j +=4)
    {
        data8[j]   = (char) ((data32[i] & 0xFF000000) >> 24);
        data8[j+1] = (char) ((data32[i] & 0x00FF0000) >> 16);
        data8[j+2] = (char) ((data32[i] & 0x0000FF00) >> 8);
        data8[j+3] = (char) (data32[i] & 0x000000FF);
        i++;
    }
    data8[256] = '\0';


    //
    // Check SOE present in first page. If not present return error.
    //
    if ((end = strstr((char *)data8, SOE)) == NULL)
    {
        json_error(1, "Could not find SOE string");
        SysCtlPeripheralDisable(SYSCTL_PERIPH_EEPROM0);
        return -1;
    }

    //
    // Check EOE has received or not. If received stop the read operation.
    //
    if ((end = strstr((char *)&data8, EOE)) != NULL)
    {
        *end = '\0';
        readEOE = true;
    }


    // Send string to GUI
    if (readEOE)
    {
        json_data("{\"eeprom\":\"%s\"}", &data8[5]);   // Send string, beginning after '#SOE;'
    }
    else
    {
        json_error(1, "Could not find EOE string");
    }

    // Disable peripheral
    SysCtlPeripheralDisable(SYSCTL_PERIPH_EEPROM0);
    return 0;
}
/**
 * @fn      int8_t writeEEPROM(char *data, uint8_t dLen)
 *
 * @brief   Reads the data from the MSP432E's internal EEPROM peripheral
 *
 * @param   data pointer to array
 * @param   dLen length of array data
 *
 * @return  int8_t
 */
int8_t writeEEPROM(char *data, uint8_t dLen)
{
    // Configure I2C
    // Initialize the optional I2C bus parameters
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open the I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(CONFIG_I2C_0, &params);

    // Initialize the slave address for transactions
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = I2C_SLAVE_ADDRESS;

    // Establish an array to read/write the EEPROM data from the flash
    char* SOE = "#SOE;";
    char* EOE = "#EOE";
    uint16_t ui16RegAddress = 0x0000;
    uint8_t ui8PageIndex    = 0;
    uint8_t * p_16 = (uint8_t *) &ui16RegAddress;

    char rData[5];
    char wData[40];
    //
    // Array configured for a total transfer of 256 bytes including address
    //    and head/tail which limits the size of the data on input
    //
    // TODO: Need to verify how large the input 'data' can be when transferred from USB buffer
    char nData[256];
    uint8_t retStatus, i, j;

    // Set GPIO Write Enable (WE) for EEPROM low, enables writing to the EEPROM
    writeProtectEnable(false);
    // No data are read
    transaction.readBuf = rData;
    transaction.readCount = 0;

    // Write data array to EEPROM
    strcpy(nData,SOE);
    strcat(nData,data);
    strcat(nData,EOE);
    uint32_t sLength = strlen(nData);
    j = 0;
    while (ui8PageIndex < (I2C_EEPROM_MEMORY_SIZE / I2C_EEPROM_PAGE_SIZE))
    {
        for(i = 2; i<34; i++)
        {
            if(j < sLength)
            {
                wData[i] = nData[j];
            }
            else wData[i] = 0xFF;
            j++;
        }

        //
        // Read one page data, 1 page is 32 bytes and there are a total of 4096 pages
        //
        ui16RegAddress = ui8PageIndex * I2C_EEPROM_PAGE_SIZE;
        //
        // Convert the position of ui16MemIndex to the the EEPROM address
        //
        wData[0] = p_16[1];
        wData[1] = p_16[0];
        // Read from the I2C slave device
        transaction.readBuf = data;
        transaction.readCount = 0;
        transaction.writeBuf = wData; // this is the page address to issue to the EEPROM to start reading the
        transaction.writeCount = sizeof(ui16RegAddress) + I2C_EEPROM_PAGE_SIZE;
        retStatus = I2C_transfer(i2cHandle, &transaction);

        //
        // Return error, if read failed
        //
        if (retStatus == false)
        {
            I2C_close(i2cHandle);
            writeProtectEnable(true);
            json_error(ERROR_I2C_READ_WRITE_FAILED,"I2C EEPROM read failed");
            return -1;
        }
        ui8PageIndex++;
        delay_us(5000);     // Wait for EEPROM to complete write operation.
    }

    // When done close the I2C bus
    I2C_close(i2cHandle);

    //
    // Disable from writing to the EEPROM
    //
    writeProtectEnable(true);
    return 0;
}
/**
 * @fn      int8_t writeInternalEEPROM(char *data, uint8_t dLen)
 *
 * @brief   Writes the data from the MSP432E's internal EEPROM peripheral
 *
 * @param   data pointer to array
 * @param   dLen length of array data
 *
 * @return  int8_t
 */
int8_t writeInternalEEPROM(char *data, uint8_t dLen)
{
    //char cTemp[] = "#SOE;host:AEC;brd_name:DC081;dev_name:pamb;brd_rev:a;brd_sno:T19I19N1112;#EOE;";
    //
    // Establish an array to write head/tail to EEPROM
    //
    char* SOE = "#SOE;";
    char* EOE = "#EOE";
    //
    // Array configured for a total transfer of 256 bytes including address
    //    and head/tail which limits the size of the data on input
    //
    // TODO: Need to verify how large the input 'data' can be when transferred from USB buffer
    char data8[256];
    uint32_t data32[64];

    uint8_t retStatus, i, j;
    //
    // Write data array to EEPROM
    //
    strcpy(data8,SOE);
    strcat(data8,data);
    strcat(data8,EOE);

    //
    // Enable the EEPROM Module and Initialize the EEPROM Block
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)))
    {
    }

    retStatus = EEPROMInit();

    //
    // If EEPROM did not initialize then exit the program code
    //
    if(retStatus != EEPROM_INIT_OK)
    {
        json_error(-1,"Internal EEPROM initialization failure");
        return -1;
    }

    //
    // Erase the EEPROM
    //
    retStatus = MAP_EEPROMMassErase();

    if(retStatus != 0)
    {
        json_error(-2,"Internal EEPROM mass erase failure");
        return -1;
    }
    //
    // Reformat 8-bit char data to 32 bit unsigned integers then write
    //
    i=0;
    for(j=0; j<256; j+=4)
    {
        data32[i] = (data8[j] << 24) | (data8[j+1] << 16) | (data8[j+2] << 8) | data8[j+3];
        i++;
    }
    //
    // Write the unsigned 32 bit integer data array to the EEPROM
    //    0 refers to the beginning address of the EEPROM, so start at the begining
    //    256 refers to the number of bytes to be written as opposed to the number of words
    //
    retStatus = EEPROMProgram(&data32[0], 0, 256);
    if(retStatus != 0)
    {
        json_error(-3,"Internal EEPROM programming failure");
        return -1;
    }

    return 0;
}
/**
 * @fn      bool EEPROMcheck(void)
 *
 * @brief   Checks the state of EEPROM or if EVM is not connected.
 *
 * @return  bool, true EEPROM found or EVM connected or false not found.
 */
bool EEPROMcheck(void)
{
    if(readEEPROM()==0) return true;
    return false;
}

