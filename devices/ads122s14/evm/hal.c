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

//*****************************************************************************
//
//                   I2C Communications
//
//*****************************************************************************

void I2C_Write ( uint8_t dataTx[], uint8_t wLength) 
{     
    DL_I2C_resetControllerTransfer(I2C_0_INST);    // reset I2C controller settings  
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, dataTx, wLength);        // fill the Tx FIFO

    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX,
        wLength, DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_ENABLE);      // send the Tx data
 
    // wait for idle.  
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));             

    // Flush Tx Buffer afterwords
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);                                                  

        

}

void I2C_Read( uint8_t dataTx[], uint8_t dataRx[], uint8_t rLength) 
{
    // reset I2C controller settings    
    DL_I2C_resetControllerTransfer(I2C_0_INST);                                                     
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);       

    // Begin writing
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX,
        1, DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_ENABLE);   
    
    // wait for idle.
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  
    
    // Begin reading
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX,
        rLength, DL_I2C_CONTROLLER_START_DISABLE, DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE); 

    // wait for idle.
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  

    for (int i = 0; i < rLength; i++)
    {
        dataRx[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    
    // Flush Rx Buffer afterwords
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);                          
  
}