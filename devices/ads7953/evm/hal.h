#ifndef EVM_HAL_H
#define EVM_HAL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/**
 * @brief Controller transmit function
 * 
 * Transmits data from the controller to the peripheral
 * CS is automatically managed (set low before transmission, high after)
 * 
 * @param data Pointer to data array to transmit
 * @param dataLength Number of bytes to transmit
 */
void SPI_Controller_transmitData(uint16_t *data, uint16_t dataLength);

/**
 * @brief Send and receive data arrays via SPI
 * 
 * CS is automatically managed (set low before transmission, high after)
 * 
 * @param dataTx Array of bytes to transmit
 * @param dataRx Array to store received bytes
 * @param bufferLength Number of bytes to transfer
 */
void spiSendReceiveArrays(const uint16_t dataTx[], uint16_t dataRx[], const uint8_t bufferLength);

#endif /* EVM_HAL_H */