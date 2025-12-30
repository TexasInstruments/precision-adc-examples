#include "ti_msp_dl_config.h"

//I2C address
#define I2C_ADDRESS 0x00

// Function Prototypes
void    spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t bufferLength);
int8_t  transferI2CData( uint8_t *writeBuff, uint8_t wLength, uint8_t *readBuff, uint8_t rLength);
void    I2C_Write ( uint8_t dataTx[], uint8_t wLength);
void    I2C_Read( uint8_t dataTx[], uint8_t dataRx[], uint8_t rLength);
