#include "ti_msp_dl_config.h"

//I2C address
#define I2C_ADDRESS 0x40

// Function Prototypes
void    I2C_Write (uint8_t dataTx[], uint8_t wLength);
void    I2C_Read(uint8_t dataRx[], uint8_t rLength);
