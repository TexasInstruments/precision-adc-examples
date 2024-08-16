#include "ti_msp_dl_config.h"

// Function Prototypes
void SPI_Controller_transmitData(uint8_t *data, uint8_t dataLength);
void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t bufferLength);
