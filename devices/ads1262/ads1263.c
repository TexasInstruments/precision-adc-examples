/**
 * \file ads1263.c
 *
 * \brief This header file contains all register map definitions for the ADS1262 and ADS1263.
 *
 * \note Macro naming conventions try to follow ADS1262 and ADS1263 data sheet naming definitions;
 *  however, future data sheet revisions may cause macro names to differ from the example code.
 *
 * \copyright Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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


#include "ads1263.h"



//****************************************************************************
//
// Macros
//
//****************************************************************************
/* Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/* Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)



//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/* Internal register map array (to recall current configuration) */
static uint8_t registerMap[NUM_REGISTERS];



//****************************************************************************
//
// Functions
//
//****************************************************************************


/**
 * \fn uint8_t getRegisterValue(uint8_t address)
 * \brief Getter function to access the registerMap array outside of this module
 * \param address The 8-bit register address
 * \return The 8-bit register value
 */
uint8_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
}


/**
 * \fn void adcStartupRoutine(void)
 * \brief Startup function to be called before communicating with the ADC
 */
void adcStartupRoutine(void)
{
    /* (OPTIONAL) Provide additional delay time for power supply settling */
    delay_ms(50);

    /* (REQUIRED) Set nRESET/nPWDN pin high for ADC operation */
    setPWDN(HIGH);

    /* (OPTIONAL) Start ADC conversions with HW pin control  */
    setSTART(HIGH);

    /* (REQUIRED) tWAKE delay */
    delay_ms(5);

    /* (OPTIONAL) Toggle nRESET pin to assure default register settings. */
    /* NOTE: This also ensures that the device registers are unlocked.   */
    toggleRESET();

    /* Ensure internal register array is initialized */
    restoreRegisterDefaults();

    /* (OPTIONAL) Configure initial device register settings here */
    uint8_t initRegisterMap[NUM_REGISTERS];
    initRegisterMap[REG_ADDR_ID]            =   0x00;            // Read-only register
    initRegisterMap[REG_ADDR_POWER]         =   POWER_DEFAULT;
    initRegisterMap[REG_ADDR_INTERFACE]     =   INTERFACE_DEFAULT;
    initRegisterMap[REG_ADDR_MODE0]         =   MODE0_DEFAULT;
    initRegisterMap[REG_ADDR_MODE1]         =   MODE1_DEFAULT;
    initRegisterMap[REG_ADDR_MODE2]         =   MODE2_DEFAULT;
    initRegisterMap[REG_ADDR_INPMUX]        =   INPMUX_DEFAULT;
    initRegisterMap[REG_ADDR_OFCAL0]        =   OFCAL0_DEFAULT;
    initRegisterMap[REG_ADDR_OFCAL1]        =   OFCAL1_DEFAULT;
    initRegisterMap[REG_ADDR_OFCAL2]        =   OFCAL2_DEFAULT;
    initRegisterMap[REG_ADDR_FSCAL0]        =   FSCAL0_DEFAULT;
    initRegisterMap[REG_ADDR_FSCAL1]        =   FSCAL1_DEFAULT;
    initRegisterMap[REG_ADDR_FSCAL2]        =   FSCAL2_DEFAULT;
    initRegisterMap[REG_ADDR_IDACMUX]       =   IDACMUX_DEFAULT;
    initRegisterMap[REG_ADDR_IDACMAG]       =   IDACMAG_DEFAULT;
    initRegisterMap[REG_ADDR_TDACP]         =   TDACP_DEFAULT;
    initRegisterMap[REG_ADDR_TDACN]         =   TDACN_DEFAULT;
    initRegisterMap[REG_ADDR_GPIOCON]       =   GPIOCON_DEFAULT;
    initRegisterMap[REG_ADDR_GPIODIR]       =   GPIODIR_DEFAULT;
    initRegisterMap[REG_ADDR_GPIODAT]       =   GPIODAT_DEFAULT;
#ifdef ADS1263_ONLY_FEATURES
    initRegisterMap[REG_ADDR_ADC2CFG]       =   ADC2CFG_DEFAULT;
    initRegisterMap[REG_ADDR_ADC2MUX]       =   ADC2MUX_DEFAULT;
    initRegisterMap[REG_ADDR_ADC2OFC0]      =   ADC2OFC0_DEFAULT;
    initRegisterMap[REG_ADDR_ADC2OFC1]      =   ADC2OFC1_DEFAULT;
    initRegisterMap[REG_ADDR_ADC2FSC0]      =   ADC2FSC0_DEFAULT;
    initRegisterMap[REG_ADDR_ADC2FSC1]      =   ADC2FSC1_DEFAULT;
#endif

    /* (OPTIONAL) Write to all (writable) registers */
    writeMultipleRegisters(REG_ADDR_POWER, NUM_REGISTERS - 1, initRegisterMap);

    /* (OPTIONAL) Read back all registers */
    readMultipleRegisters(REG_ADDR_ID, NUM_REGISTERS);

    /* (OPTIONAL) Start ADC conversions with the SPI command.
     * Not needed if the START pin has already been set HIGH.
     *
     * sendCommand(START_OPCODE);
     */
}

/**
 * \fn uint8_t readSingleRegister(uint8_t address)
 * \brief Reads contents of a single register at the specified address
 * \param address The address of the register to read
 * \return 8-bit register read result
 */
uint8_t readSingleRegister(uint8_t address)
{
    uint8_t DataTx[3] = { 0 };
    uint8_t DataRx[3] = { 0 };

    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    /* Build TX array and send it */
    DataTx[0] = OPCODE_RREG + (address & 0x1F);
    spiSendReceiveArrays(DataTx, DataRx, 3);

    /* Update register array and return read result*/
    registerMap[address] = DataRx[2];
    return DataRx[2];
}



/**
 * \fn void readMultipleRegisters(uint8_t startAddress, uint8_t count)
 * \brief Reads a group of registers starting at the specified address
 * \param startAddress register address from which we start reading
 * \param count number of registers we want to read
 * NOTE: Use getRegisterValue() to retrieve the read values
 */
void readMultipleRegisters(uint8_t startAddress, uint8_t count)
{
    /* Validate function arguments */
    assert(startAddress + count <= NUM_REGISTERS);


    //
    // SPI communication
    //
    setCS(LOW);

    uint8_t dataTx = OPCODE_RREG + (startAddress & 0x1F);
    spiSendReceiveByte(dataTx);

    dataTx = count - 1;
    spiSendReceiveByte(dataTx);

    uint8_t i;
    for (i = startAddress; i < startAddress + count; i++)
    {
        // Read register data bytes
        registerMap[i] = spiSendReceiveByte(0x00);
    }

    setCS(HIGH);
}


/**
 * \fn void writeSingleRegister(uint8_t address, uint8_t data)
 * \brief Write data to a single register at the specified address
 * \param address The address of the register to write
 * \param data The 8-bit data to write to the register
 */
void writeSingleRegister(uint8_t address, uint8_t data)
{
    uint8_t DataTx[3] = { 0 };
    uint8_t DataRx[3] = { 0 };

    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    /* Build TX array and send it */
    DataTx[0] = OPCODE_WREG + (address & 0x1F);
    //DataTx[1] = 0x00;
    DataTx[2] = data;
    spiSendReceiveArrays(DataTx, DataRx, 3);

    /* Update the global register map variable */
    //ADC_RegisterMap[addr] = data;
}



/**
 * \fn void writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[])
 * \brief Writes data to a group of registers
 * \param startAddress register address from which we start write
 * \param count number of registers we want to write to
 * \param regData Array that holds the data to write, where element zero is the data to write to the starting address.
 * NOTES:
 * - Use getRegisterValue() to retrieve the written values.
 * - Registers should be re-read after a write operaiton to ensure proper configuration.
 */
void writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[])
{
    /* Check that the register address and count are in range */
    assert(startAddress + count <= NUM_REGISTERS);

    /* Check that regData is not a NULL pointer */
    assert(regData);


    //
    // SPI communication
    //

    setCS(LOW);

    uint8_t dataTx = OPCODE_WREG + (startAddress & 0x1F);
    spiSendReceiveByte(dataTx);

    dataTx = count - 1;
    spiSendReceiveByte(dataTx);

    uint8_t i;
    for (i = startAddress; i < startAddress + count; i++)
    {
        // write register data bytes
        spiSendReceiveByte(regData[i]);

        /* Update register array */
        registerMap[i] = regData[i];
    }

    setCS(HIGH);
}


/**
 * \fn void sendCommand(uint8_t op_code)
 * \brief Sends the specified SPI command to the ADC
 * \param op_code SPI command byte
 */
void sendCommand(uint8_t op_code)
{
    /* Assert if this function is used to send any of the following commands */
    assert(OPCODE_RREG != op_code);     /* Use "readSingleRegister()"   */
    assert(OPCODE_WREG != op_code);     /* Use "writeSingleRegister()"  */
    assert(OPCODE_RDATA1 != op_code);   /* Use "readData()"             */
#ifdef ADS1263_ONLY_FEATURES
    assert(OPCODE_RDATA2 != op_code);   /* Use "readData2()"            */
#endif

    uint8_t DataTx[1] = { op_code };
    uint8_t DataRx[1] = { 0 };

    /* Build TX array and send it */
    spiSendReceiveArrays(DataTx, DataRx, 1);
}



/**
 * \fn int32_t readData(uint8_t status[], uint8_t data[], uint8_t crc[])
 * \brief Sends the RDATA command and retrieves STATUS, DATA, & CRC bytes
 * \param status[] pointer to address where STATUS byte will be stored
 * \param data[] pointer to starting address where data bytes will be stored
 * \param crc[] pointer to address where CRC byte will be stored
 * \return 32-bit sign-extended conversion result (data only)
 */
int32_t readData(uint8_t status[], uint8_t data[], uint8_t crc[])       //TODO: Implement read direct command (as a separate function!)
{
    uint8_t DataTx[7] = { 0 };
    uint8_t DataRx[7] = { 0 };

    /* Calculate command length (bytes) */
    uint8_t byteLength = 5 + (CRC_BYTE_ENABLED ? 1 : 0) + \
                                    (STATUS_BYTE_ENABLED ? 1 : 0);

    /* Build TX array and send it */
    DataTx[0] = OPCODE_RDATA1;
    spiSendReceiveArrays(DataTx, DataRx, byteLength);


    //
    // Parse returned SPI data
    //

    uint8_t dataPosition = 1;
    if (STATUS_BYTE_ENABLED)
    {
        /* Store STATUS byte to memory? */
        if (status != NULL) { status[0] = DataRx[dataPosition]; }

        /* Increment data position counter */
        dataPosition++;
    }

    /* Store data bytes to memory? */
    if (data != NULL)
    {
        data[0] = DataRx[dataPosition];
        data[1] = DataRx[dataPosition + 1];
        data[2] = DataRx[dataPosition + 2];
        data[3] = DataRx[dataPosition + 3];
    }

    /* Store CRC byte to memory? */
    if (CRC_BYTE_ENABLED && (crc != NULL)) { crc[0] = DataRx[dataPosition + 4]; }


    /* Return the 32-bit sign-extended conversion result */
    int32_t upperByte       = ((int32_t) DataRx[dataPosition + 0] & 0xFF) << 24;
    int32_t upperMidByte    = ((int32_t) DataRx[dataPosition + 1] & 0xFF) << 16;
    int32_t lowerMidByte    = ((int32_t) DataRx[dataPosition + 2] & 0xFF) << 8;
    int32_t lowerByte       = ((int32_t) DataRx[dataPosition + 3] & 0xFF) << 0;
    return (upperByte | upperMidByte | lowerMidByte | lowerByte);
}



/**
 * \fn void restoreRegisterDefaults(void)
 * \brief Updates the registerMap[] array to its default values.
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADC register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device (either by
 * hardware pin control or SPI software command).
 *
 * - Reading back all of the registers after resetting the device will
 * accomplish the same result.
 */
void restoreRegisterDefaults(void)
{
    registerMap[REG_ADDR_ID]            =   0x00;           // Value of 0x00 indicates that we have not yet read the ID register
    registerMap[REG_ADDR_POWER]         =   POWER_DEFAULT;
    registerMap[REG_ADDR_INTERFACE]     =   INTERFACE_DEFAULT;
    registerMap[REG_ADDR_MODE0]         =   MODE0_DEFAULT;
    registerMap[REG_ADDR_MODE1]         =   MODE1_DEFAULT;
    registerMap[REG_ADDR_MODE2]         =   MODE2_DEFAULT;
    registerMap[REG_ADDR_INPMUX]        =   INPMUX_DEFAULT;
    registerMap[REG_ADDR_OFCAL0]        =   OFCAL0_DEFAULT;
    registerMap[REG_ADDR_OFCAL1]        =   OFCAL1_DEFAULT;
    registerMap[REG_ADDR_OFCAL2]        =   OFCAL2_DEFAULT;
    registerMap[REG_ADDR_FSCAL0]        =   FSCAL0_DEFAULT;
    registerMap[REG_ADDR_FSCAL1]        =   FSCAL1_DEFAULT;
    registerMap[REG_ADDR_FSCAL2]        =   FSCAL2_DEFAULT;
    registerMap[REG_ADDR_IDACMUX]       =   IDACMUX_DEFAULT;
    registerMap[REG_ADDR_IDACMAG]       =   IDACMAG_DEFAULT;
    registerMap[REG_ADDR_TDACP]         =   TDACP_DEFAULT;
    registerMap[REG_ADDR_TDACN]         =   TDACN_DEFAULT;
    registerMap[REG_ADDR_GPIOCON]       =   GPIOCON_DEFAULT;
    registerMap[REG_ADDR_GPIODIR]       =   GPIODIR_DEFAULT;
    registerMap[REG_ADDR_GPIODAT]       =   GPIODAT_DEFAULT;
#ifdef ADS1263_ONLY_FEATURES
    registerMap[REG_ADDR_ADC2CFG]       =   ADC2CFG_DEFAULT;
    registerMap[REG_ADDR_ADC2MUX]       =   ADC2MUX_DEFAULT;
    registerMap[REG_ADDR_ADC2OFC0]      =   ADC2OFC0_DEFAULT;
    registerMap[REG_ADDR_ADC2OFC1]      =   ADC2OFC1_DEFAULT;
    registerMap[REG_ADDR_ADC2FSC0]      =   ADC2FSC0_DEFAULT;
    registerMap[REG_ADDR_ADC2FSC1]      =   ADC2FSC1_DEFAULT;
#endif
}



/**
 * \fn void startConversions()
 * \brief Wakes the device from power-down and starts continuous conversions
 */
void startConversions(void)
{
    /* Ensure device is not in PWDN mode */
    setPWDN(HIGH);

    /* Begin continuous conversions */
    setSTART(HIGH);
}



/**
 * \fn uint8_t calculateCRC(uint8_t *dataByte, uint8_t numBytes)
 * \brief Calculates the 8-bit CRC for data words, up to 4 bytes
 *
 * NOTE: This calculation is shown as an example and is not optimized for speed.
 * On a TM4C1294NCPDT, operating with a 120 MHz system clock, this function's
 * execution time is between 5-10 us.
 *
 *
 * --- CRC Calculation Description ---
 * The CRC shift register is initialized to 0xFF and all data is shifted in MSB first, as shown in the
 * diagram below. 'Bit 7' of the CRC shifted register is XOR'd with the next data bit and the result
 * is placed into 'Bit 0' of shift register and also used in additional XOR operations. 'Bit 1' takes
 * the value of 'Bit 0' XOR'd with the result of the first XOR operation. Similarly, 'Bit 2' takes
 * the value of 'Bit 1' XOR'd with the result of the first XOR operation. All other bits in the CRC
 * shift register are shifted, such that 'Bit N' takes on the previous value of 'Bit N-1'.
 *
 * NOTE: If the first XOR operation results in a '0', all other XOR operations retain the value of the
 * previous bit; and as such, can be ignored.
 *
 *                       Bit    Bit    Bit    Bit    Bit    Bit           Bit           Bit
 * CRC Shift register:   |7| << |6| << |5| << |4| << |3| << |2| << XOR << |1| << XOR << |0|
 *                        |                                         ^             ^      ^
 *                        V                                         |             |      |
 * Data IN (MSB 1st) ->> XOR -------------------------------------------------------------
 *
 *
 *
 * \param *dataByte pointer to first element in the data byte array
 * \param numBytes number of bytes (between 3 and 4) used in CRC calculation
 * \return calculated CRC byte
 */
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numBytes)
{
    /* Check that "numBytes" is between 1 and 4 */
    assert((numBytes >= 3) && (numBytes <= 4));

    /* Check that "dataBytes" is not a null pointer */
    assert(dataBytes != NULL);

    /* NOTE:
     * Using "uint_fast8_t" types here instead of "uint8_t" to avoid unnecessary
     * implicit type conversions. Reference this E2E thread for additional info:
     * https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/679200
     */
    uint_fast8_t i;
    uint_fast8_t crc        = 0xFFu;        /* Initial value of crc register     */
    uint_fast8_t crcMSb;                    /* Most significant bit of crc byte  */
    const uint_fast8_t poly = 0x07u;        /* CRC polynomial byte               */
    uint_fast8_t shift_by   = 0u;           /* Intermediate variable             */
    uint32_t data           = 0u;           /* Data storage variable             */
    uint32_t msbMask        = 0x80000000u;  /* Points to the next data bit       */
    uint32_t dataMSb;                       /* Most significant bit of data int  */

    /* Construct data word from data bytes */
    for (i = 0; i < numBytes; i++)
    {
        shift_by = 8 * (numBytes - i - 1);
        data |= (((uint32_t) dataBytes[i]) << shift_by);
    }

    /* Determine the location of the first data byte */
    shift_by = 8 * (4 - numBytes);
    msbMask >>= shift_by;

    /* CRC algorithm */
    while (msbMask > 0)
    {
        // Check MSB's of data and crc
        dataMSb = data & msbMask;
        crcMSb  = crc & 0x80u;

        // Shift crc byte
        crc <<= 1;

        // Check if XOR operation of MSbs results in additional XOR operation
        if (dataMSb ^ crcMSb)   { crc ^= poly; }

        /* Shift MSb pointer */
        msbMask >>= 1;
    }

    return crc;
}


uint8_t calculateChecksum(const uint8_t dataBytes[], uint8_t numBytes)
{
    /* Check that "numBytes" is between 3 and 4 */
    assert((numBytes >= 3) && (numBytes <= 4));

    /* Check that "dataBytes" is not a null pointer */
    assert(dataBytes != NULL);

    /* NOTE:
     * Using "uint_fast8_t" types here instead of "uint8_t" to avoid unnecessary
     * implicit type conversions. Reference this E2E thread for additional info:
     * https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/679200
     */

    /* Checksum initial value */
    uint_fast8_t checksum   = 0x9Bu;

    /* Checksum calculation */
    uint8_t i;
    for (i = 0; i < numBytes; i++)
    {
        checksum = (checksum + dataBytes[i]) & 0xFFu;
    }

    return checksum;
}






/*
 * Use Case Examples
 */

/**
 * \fn void ads1261_enable_internal_reference_example(void)
 * \brief Enables and selects the internal reference for ADC measurements
 */
//void ads1261_enable_internal_reference_example(void)
//{
//    /* Configure REF register: INT REF selected & enabled */
//    uint8_t regData = (REF_REFENB_MASK | REF_RMUXP_INT_P | REF_RMUXN_INT_N);
//    writeSingleRegister(REG_ADDR_REF, regData);
//
//    /* Wait for internal reference to settle */
//    delay_ms(50);
//
//    /* (OPTIONAL) In case of a large in-rush current when charging the REFOUT
//     * capacitor, read the STATUS register to make sure the device wasn't reset.
//     */
//}


/**
 * \fn void ads1261_measure_internal_temperature_example(void)
 * \brief Measures and computes the internal temperature in degrees Celsius
 * \return the _________TODO:
 */
//float ads1261_measure_internal_temperature_example(void)
//{
//    /* Enable the internal reference */
//    ads1261_enable_internal_reference_example();
//
//    /* Configure INPBIAS register: default settings  */
//    writeSingleRegister(REG_ADDR_INPBIAS, INPBIAS_DEFAULT);
//
//    /* Configure PGA register: default settings  */
//    writeSingleRegister(REG_ADDR_PGA, PGA_DEFAULT);
//
//    /* Configure input MUX: Internal temperature monitor  */
//    uint8_t regData = (INPMUX_MUXP_TEMP_P | INPMUX_MUXN_TEMP_N);
//    writeSingleRegister(REG_ADDR_INPMUX, regData);
//
//    /* Collect data */
//    ads1261_startConversions();
//    pollForDRDY(100);
//    int32_t adcData = readData(NULL, NULL, NULL);
//
//    /* Convert to temperature (deg C) */
//    float uVolts = (float) adcData * 0.298023224;
//    float temp_C = 25.0 + ((uVolts - 122400.0) / 420.0);
//
//    return temp_C;
//}



//// Reads data and writes to registers concurrently
//void ADS126xREADandWRITE(int NumDatBytes, int StartAddress, int NumRegs, unsigned char * pdata)
//{
//  uint8_t ADC_Bytes[6];
//  int32_t ADC_Data_Only;
//
//  WaitForDRDY();
//
//  set_adc_CS(0);
//
//  ADS126xXferByte(RDATA1);
//
//  ADS126xXferByte(0x40+StartAddress);
//  ADS126xXferByte(NumRegs-1);
//  for(int i=0;i<NumRegs;i++)
//  {
//      ADS126xXferByte(pdata[i]);
//  }
//  set_adc_CS(1);
//}



//unsigned char ADS126xReadADC2Data(bufferType_t *readbuffer){
//
//  set_adc_CS(0);
//  ADS126xXferByte(RDATA2);
//  readbuffer->data.status=ADS126xXferByte(0);
//  readbuffer->data.payload.bytes[3]=ADS126xXferByte(0);
//  readbuffer->data.payload.bytes[2]=ADS126xXferByte(0);
//  readbuffer->data.payload.bytes[1]=ADS126xXferByte(0);
//  readbuffer->data.payload.bytes[0]=ADS126xXferByte(0);
//  readbuffer->data.checksum=ADS126xXferByte(0);
//  readbuffer->data.adcindicator=0x80|g_packetCounter;
//  if(g_packetCounter<0x3F)
//      g_packetCounter++;
//  else
//      g_packetCounter=0;
//  set_adc_CS(1);
//  return readbuffer->data.checksum; //return CRC
//}
