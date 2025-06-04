/**
* @file ads79xx.h
* @brief Header file for ADS79xx series ADCs
* 
* This file provides an abstraction layer for the ADS79xx family of ADCs
* Compatible with all resolution (12/10/8-bit) and channel (16/12/8/4) variants
*/
#ifndef ADS79XX_H
#define ADS79XX_H

#include <stdint.h>
#include <stdbool.h>
#include "evm/hal.h"

/************************** Device Selection **************************/
/* Uncomment ONE of the following device definitions */

/* 12-bit devices */
// #define ADS7950    /* 4-channel, 12-bit */
// #define ADS7951    /* 8-channel, 12-bit */
// #define ADS7952    /* 12-channel, 12-bit */
#define ADS7953    /* 16-channel, 12-bit */

/* 10-bit devices */
// #define ADS7954    /* 4-channel, 10-bit */
// #define ADS7955    /* 8-channel, 10-bit */
// #define ADS7956    /* 12-channel, 10-bit */
// #define ADS7957    /* 16-channel, 10-bit */

/* 8-bit devices */
// #define ADS7958    /* 4-channel, 8-bit */
// #define ADS7959    /* 8-channel, 8-bit */
// #define ADS7960    /* 12-channel, 8-bit */
// #define ADS7961    /* 16-channel, 8-bit */

/* Package selection (uncomment ONE) */
#define PACKAGE_TSSOP    /* TSSOP package with 4 GPIOs */
// #define PACKAGE_VQFN     /* VQFN package with 1 GPIO */

/* Resolution and channel count configuration */
#if defined(ADS7950) || defined(ADS7951) || defined(ADS7952) || defined(ADS7953)
 #define RESOLUTION     12
#elif defined(ADS7954) || defined(ADS7955) || defined(ADS7956) || defined(ADS7957)
 #define RESOLUTION     10
#elif defined(ADS7958) || defined(ADS7959) || defined(ADS7960) || defined(ADS7961)
 #define RESOLUTION     8
#endif

#if defined(ADS7950) || defined(ADS7954) || defined(ADS7958)
 #define CHANNELS       4
#elif defined(ADS7951) || defined(ADS7955) || defined(ADS7959)
 #define CHANNELS       8
#elif defined(ADS7952) || defined(ADS7956) || defined(ADS7960)
 #define CHANNELS       12
#elif defined(ADS7953) || defined(ADS7957) || defined(ADS7961)
 #define CHANNELS       16
#endif

/* Package selection */
#if defined(PACKAGE_TSSOP)
 #define GPIO_COUNT     4
#else /* PACKAGE_VQFN */
 #define GPIO_COUNT     1
#endif

/* Static buffer size for conversions - can be modified by user */
#define BUFFER_SIZE  32

/************************** Device Mode Definitions **************************/

/* Input Range Options */
#define RANGE_VREF     0 /* Range 1: 0 to VREF */
#define RANGE_2VREF    1 /* Range 2: 0 to 2xVREF */

/* Alarm Types */
#define HIGH_ALARM_SEL     0 /* Alarm high threshold */
#define LOW_ALARM_SEL      1 /* Alarm low threshold */

/* Channel Sequencing Modes */
typedef enum {
    MODE_MANUAL  = 0x1, /* Manual channel selection */
    MODE_AUTO1   = 0x2, /* Auto sequence through programmed channels */
    MODE_AUTO2   = 0x3, /* Auto sequence from CH0 to programmed last channel */
    MODE_CONT    = 0x0  /* Continue operation in current mode */
} Mode_t;

/* GPIO Configuration Options for Each Pin */
/* GPIO0 Configuration Options */
typedef enum {
    GPIO0_INPUT      = 0, /* Configure as input */
    GPIO0_OUTPUT     = 1, /* Configure as output */
    GPIO0_HIGH_ALARM = 2, /* High alarm indicator */
    GPIO0_ANY_ALARM  = 3  /* High or low alarm indicator */
} GPIO0_Config_t;

/* GPIO1 Configuration Options */
typedef enum {
    GPIO1_INPUT     = 0, /* Configure as input */
    GPIO1_OUTPUT    = 1, /* Configure as output */
    GPIO1_LOW_ALARM = 2  /* Low alarm indicator */
} GPIO1_Config_t;

/* GPIO2 Configuration Options */
typedef enum {
    GPIO2_INPUT  = 0, /* Configure as input */
    GPIO2_OUTPUT = 1, /* Configure as output */
    GPIO2_RANGE  = 2  /* Input range selection */
} GPIO2_Config_t;

/* GPIO3 Configuration Options */
typedef enum {
    GPIO3_INPUT  = 0, /* Configure as input */
    GPIO3_OUTPUT = 1, /* Configure as output */
    GPIO3_PD     = 2  /* Power down function */
} GPIO3_Config_t;

/* GPIO Configuration Structure */
typedef struct {
    GPIO0_Config_t gpio0; /* GPIO0 configuration */
    GPIO1_Config_t gpio1; /* GPIO1 configuration */
    GPIO2_Config_t gpio2; /* GPIO2 configuration */
    GPIO3_Config_t gpio3; /* GPIO3 configuration */
    bool device_reset;    /* Reset all registers to default state */
} GPIOConfig_t;

/* Channel Sequence Structure for Auto-1 Mode */
typedef struct {
    uint16_t enabled_channels; /* Bit mask of enabled channels (up to 16 channels) */
} ChannelSeq_t;

/* Alarm Configuration Structure */
typedef struct {
    uint8_t channel;      /* Channel number (0 to CHANNELS-1) */
    uint8_t alarm_level;  /* Select LOW_ALARM or HIGH_ALARM for programming */
    uint16_t threshold;   /* Alarm threshold value */
} AlarmConfig_t;

/* Conversion Result Structure */
typedef struct {
    uint8_t channel;      /* Channel number */
    uint16_t value;       /* Conversion result */
} ConversionResult_t;

/************************** Function Prototypes **************************/

/**
 * @brief Put the device into power down mode
 * 
 * @param enable True to enable power down, false to wake up
 */
void PowerDown(bool enable);

/**
 * @brief Set the ADC input range
 * 
 * @param range Range selection (RANGE_VREF or RANGE_2VREF)
 */
void SetRange(uint8_t range);

/**
 * @brief Set the operating mode of the device
 * 
 * @param mode Operating mode (Manual, Auto-1, Auto-2, or Continue current mode)
 * @param reset_channel True to reset the channel counter (for Auto modes)
 */
void SetMode(Mode_t mode, bool reset_channel);

/**
 * @brief Configure all GPIOs in a single operation
 * 
 * This function sends the configuration to the device
 * 
 * @param config Pointer to the GPIO configuration structure
 * @return true if the configuration was successful
 */
bool ConfigureGPIOs(const GPIOConfig_t *config);

/**
 * @brief Write GPIO output value (for pins configured as output)
 * 
 * @param gpio_num GPIO number (0-3 for TSSOP, 0 only for VQFN)
 * @param value Output value (0 or 1)
 * @return true if successful, false if error
 */
bool WriteGPIO(uint8_t gpio_num, uint8_t value);

/**
 * @brief Read GPIO input value (for pins configured as input)
 * 
 * @param gpio_num GPIO number (0-3 for TSSOP, 0 only for VQFN)
 * @param value Pointer to store the GPIO value
 * @return true if successful, false if error
 */
bool ReadGPIO(uint8_t gpio_num, uint8_t *value);

/**
 * @brief Configure Auto-1 mode channel sequence
 * 
 * @param seq Pointer to channel sequence structure
 * @return true if successful, false if error
 */
bool ConfigureAuto1Sequence(const ChannelSeq_t *seq);

/**
 * @brief Configure Auto-2 mode last channel
 * 
 * @param last_channel Last channel number (0 to CHANNELS-1)
 * @return true if successful, false if error
 */
bool ConfigureAuto2LastChannel(uint8_t last_channel);

/**
 * @brief Configure alarm threshold for a channel
 * 
 * @param alarm_config Pointer to alarm configuration structure
 * @return true if successful, false if error
 */
bool ConfigureAlarm(const AlarmConfig_t *alarm_config);

/**
 * @brief Manually select a channel for the next conversion (Manual mode)
 * 
 * @param channel Channel number (0 to CHANNELS-1)
 * @return true if successful, false if error
 */
bool SelectChannel(uint8_t channel);

/**
 * @brief Perform a single conversion and read the result
 * 
 * @param result Pointer to store the conversion result
 * @return true if successful, false if error
 */
bool ReadConversion(ConversionResult_t *result);

/**
 * @brief Capture conversions in the current mode and store in buffer
 * 
 * @param num_conversions Number of conversions to capture (limited by buffer size)
 * @return true if successful, false if error
 */
bool CaptureConversions(uint16_t num_conversions);

/**
 * @brief Get conversion results from the internal buffer
 * 
 * @param results Array to store the conversion results
 * @param max_results Maximum number of results to retrieve (size of results array)
 * @return Number of results copied
 */
uint16_t GetResults(ConversionResult_t *results, uint16_t max_results);

/**
 * @brief Get the number of valid conversion results in the buffer
 * 
 * @return Number of valid results in the buffer
 */
uint16_t GetResultCount(void);

/**
 * @brief Clear all conversion results from the buffer
 */
void ClearConversionBuffer(void);

#endif /* ADS79XX_H */