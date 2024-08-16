#include "evm/hal.h"
#include "ti/devices/msp/peripherals/hw_spi.h"
#include "ti_msp_dl_config.h"


/*
 *  Helper function to transmit data from the Controller to the Peripheral.
 *  This function assumes that the command has already been transmitted, so
 *  the CD line should already be set HIGH indicating that data is being
 *  transmitted.
 *
 *  data         The data to send to the Peripheral.
 *               Example: DATA_TYPE_0
 *  dataLength   The number of data bytes to send.
 *               Example: DATA_TYPE_0_LENGTH
 */



void SPI_Controller_transmitData(uint8_t *data, uint8_t dataLength)
{
    int i = 0;
    for (i = 0; i < dataLength; i++) {
        while (DL_SPI_isBusy(SPI_0_INST))
            ;
        DL_SPI_transmitData8(SPI_0_INST, data[i]);
    }
}


void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t bufferLength)
{
    
    int i = 0;
    for (i = 0; i < bufferLength; i++) {
        
        DL_SPI_transmitData8(SPI_0_INST, dataTx[i]);
    }
    
    for (i = 0; i < bufferLength; i++) {
        
        dataRx[i] = DL_SPI_receiveDataBlocking8(SPI_0_INST);
    }


}