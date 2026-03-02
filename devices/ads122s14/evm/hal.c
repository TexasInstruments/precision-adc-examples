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


//****************************************************************************
//
// SPI Communication
//
//****************************************************************************

void spiSendReceiveArrays(uint8_t dataTx[], uint8_t dataRx[], uint8_t bufferLength)
// this funciton will operate the MSPM0 SPI port.  Because the SPI FIFO on M0 is 16-bit,
// the dataTX and dataRX buffers are packed into 16-bit elements before being tranfered to/from the SPI FIFO. 
// this transformation improves data readback when SPI frames include SPI and CRC words ( 6-byte frame )

{
    DL_SPI_drainRXFIFO8(SPI_0_INST, dataRx, 4);


    uint8_t i = 0;
    uint8_t j = 0;


    DL_GPIO_clearPins(GPIOA, GPIO_GRP_0_CSn_PIN);

    while (i < bufferLength) {
        
        if(!DL_SPI_isTXFIFOFull(SPI_0_INST)) {      // add to fifo if not full
            DL_SPI_transmitData8(SPI_0_INST, dataTx[i++]);
        }
        if(!DL_SPI_isRXFIFOEmpty(SPI_0_INST)) {     // read from fifo if not empty
            dataRx[j++] = DL_SPI_receiveData8(SPI_0_INST);    
        }
            
    }
    
    while (i != j)  {
        if(!DL_SPI_isRXFIFOEmpty(SPI_0_INST)) {
            dataRx[j++] = DL_SPI_receiveData8(SPI_0_INST);    
        }
        
    }

    DL_GPIO_setPins(GPIOA, GPIO_GRP_0_CSn_PIN);


     
}