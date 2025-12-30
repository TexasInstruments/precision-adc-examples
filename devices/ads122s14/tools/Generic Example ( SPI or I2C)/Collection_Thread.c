/*
 * Collection_Thread.c
 *
 *  Created on: Mar 27, 2024
 *      Author: Patrick Edwards
 */


#include "evm/hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "interrupts.h"

//****************************************************
//Internal Variables
//****************************************************

//****************************************************
//Functions
//****************************************************

void collectData (void)
{
    uint32_t samplesCollected;
//  static uint32_t adcData[numSamples];
    const uint8_t dataTx[3] = {0};
    uint8_t dataRx[3] = {0};
    static uint8_t numWords = 3;
    uint8_t timeout = 0;

#ifdef SPI_COMMS
    enableCSpulldown();
#endif

#ifdef I2C_COMMS
    uint8_t ADCData[3];
    bool DRDY = false;
        writeSingleRegister(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_STATUS_EN_MASK); // Enable Status words
        transferI2CData(dataTx, 0, dataRx, 5);
        DRDY = dataRx[0] & STATUS_MSB_DRDY_DATA_NEW;



#endif

   for (samplesCollected = 0; samplesCollected != g_num_samples; samplesCollected++)
   {
        // Read data command is a 1 frame operation
        // DIN should be set to 0x000000 while clocking 24 SCLK bits
        // FYI, max data rate is 128KSPS, which converts to a 7.8us delay between samples in continuous mode.


        // Read conversion results
#ifdef SPI_COMMS
       waitForDRDYinterrupt(1000);
       spiSendReceiveArrays(dataTx, dataRx, numWords);
       USBBufferWrite(BULK_TX_BUFFER, dataRx, numWords);    // Print out the conversion result to USB_BULK

#endif

#ifdef I2C_COMMS

       if(!DRDY)
               {
                   timeout = 0;
                   do{
                       delay_us(100);
                       transferI2CData(dataTx, 0, dataRx, 5);          // get new i2c data
                       DRDY = dataRx[0] & STATUS_MSB_DRDY_DATA_NEW;    // check if data is new
                       timeout++;
                   }
                   while (!DRDY | timeout==10 );                        // exit loop if drdy is new, or loop exceeds ~1000ms.
               }
                       ADCData[0] = dataRx[2];
                       ADCData[1] = dataRx[3];
                       ADCData[2] = dataRx[4];
                       DRDY = false;
       USBBufferWrite(BULK_TX_BUFFER, ADCData, numWords);    // Print out the conversion result to USB_BULK
#endif


   }

#ifdef SPI_COMMS
   enableCSpullup();
#endif

#ifdef I2C_COMMS
   writeSingleRegister(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_STATUS_EN_MASK ); // Disable Status words
#endif


    return;
}
