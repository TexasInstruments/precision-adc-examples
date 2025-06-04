/**
 * @file ads79xx.c
 * @brief Implementation for ADS79xx series ADCs
 * 
 * This file provides an abstraction layer for the ADS79xx family of ADCs
 * Compatible with all resolution (12/10/8-bit) and channel (16/12/8/4) variants
 */

#include "ads79xx.h"
#include <stdint.h>
#include <stdbool.h>

/************************** Private Definitions **************************/

/* Mode Control Register Bit Definitions */
#define MCR_MODE_POS       12  /* Mode selection bits position */
#define MCR_ENABLE_PROG    (1 << 11)  /* Enable programming */
#define MCR_RESET_CHAN     (1 << 10)  /* Reset channel counter */
#define MCR_CHAN_POS       7   /* Channel selection bits position */
#define MCR_RANGE_POS      6   /* Range selection bit position */
#define MCR_POWERDOWN      (1 << 5)  /* Power down bit */
#define MCR_GPIO_READ      (1 << 4)  /* GPIO read enable bit */
#define MCR_GPIO_MASK      0x000F  /* GPIO data mask */

/* Program Register Selectors */
#define PRG_AUTO1          0x8  /* Auto-1 mode programming */
#define PRG_AUTO2          0x9  /* Auto-2 mode programming */
#define PRG_GPIO           0x4  /* GPIO programming */
#define PRG_ALARM_GROUP0   0xC  /* Alarm group 0 (Ch 0-3) */
#define PRG_ALARM_GROUP1   0xD  /* Alarm group 1 (Ch 4-7) */
#define PRG_ALARM_GROUP2   0xE  /* Alarm group 2 (Ch 8-11) */
#define PRG_ALARM_GROUP3   0xF  /* Alarm group 3 (Ch 12-15) */

/* Alarm Register Bit Definitions */
#define ALARM_CHAN_POS     14  /* Channel position in alarm register */
#define ALARM_HIGH         (1 << 13)  /* High alarm select */
#define ALARM_LOW          (0 << 13)  /* Low alarm select */
#define ALARM_EXIT         (1 << 12)  /* Exit alarm programming */
#define ALARM_THRESH_MASK  0x03FF  /* Threshold mask (10 bits) */

/* GPIO Register Bit Definitions */
#define GPIO_RESET         (1 << 9)  /* Reset device */
#define GPIO_PD_FUNC       (1 << 8)  /* Configure GPIO3 as power down */
#define GPIO_RANGE_FUNC    (1 << 7)  /* Configure GPIO2 as range select */
#define GPIO_LOW_ALARM     (1 << 6)  /* Configure GPIO1 as low alarm */
#define GPIO_HIGH_ALARM    (1 << 5)  /* Configure GPIO0 as high alarm */
#define GPIO_ANY_ALARM     (1 << 4)  /* Configure GPIO0 as any alarm */

/* Data Bit Masks based on resolution */
#if RESOLUTION == 12
    #define DATA_MASK      0x0FFF  /* 12-bit mask: bits 11-0 */
    #define DATA_SHIFT     0       /* No shift needed */
#elif RESOLUTION == 10
    #define DATA_MASK      0x03FF  /* 10-bit mask: bits 9-0 */
    #define DATA_SHIFT     0       /* No shift needed */
#else /* RESOLUTION == 8 */
    #define DATA_MASK      0x00FF  /* 8-bit mask: bits 7-0 */
    #define DATA_SHIFT     0       /* No shift needed */
#endif

/* Max alarm threshold values based on resolution */
#if RESOLUTION == 12
    #define MAX_ALARM_THRESHOLD  4092  /* Max threshold for 12-bit devices */
#elif RESOLUTION == 10
    #define MAX_ALARM_THRESHOLD  1023  /* Max threshold for 10-bit devices */
#else /* RESOLUTION == 8 */
    #define MAX_ALARM_THRESHOLD  255   /* Max threshold for 8-bit devices */
#endif

/* Auto-2 last channel position and mask */
#define AUTO2_LAST_CHAN_POS  6   /* Last channel position in Auto-2 register */
#define AUTO2_LAST_CHAN_MASK 0x0F /* 4-bit mask for last channel */

/* Helper macros for command composition */
#define CMD_MODE(mode)     ((mode & 0xF) << MCR_MODE_POS)
#define CMD_CHANNEL(ch)    ((ch & 0xF) << MCR_CHAN_POS)
#define CMD_RANGE(range)   ((range & 0x1) << MCR_RANGE_POS)
#define CMD_GPIO(gpio)     (gpio & MCR_GPIO_MASK)

/* Internal buffer size */
#define ADS79XX_BUFFER_SIZE BUFFER_SIZE

/************************** Private Variables **************************/

/* Current device state */
static struct {
    Mode_t current_mode;            /* Current operating mode */
    uint8_t current_range;          /* Current input range */
    uint8_t last_selected_channel;  /* Last manually selected channel */
    uint8_t auto2_last_channel;     /* Auto-2 mode last channel */
    uint8_t gpio_output_values[GPIO_COUNT]; /* GPIO output values */
    bool gpio_is_output[GPIO_COUNT]; /* GPIO direction (true if output) */
    bool gpio_special_function[GPIO_COUNT]; /* True if GPIO has special function */
    ChannelSeq_t auto1_sequence;    /* Auto-1 channel sequence configuration */
} device_state;

/* Static conversion buffer */
static ConversionResult_t conversion_buffer[ADS79XX_BUFFER_SIZE];
static uint16_t buffer_count = 0;  /* Number of valid results in buffer */

/* Flag to indicate if device is initialized */
static bool is_initialized = false;

/* Default SPI transfer buffers */
static uint16_t tx_data[1];
static uint16_t rx_data[1];

/************************** Private Helper Functions **************************/

/**
 * @brief Perform SPI transfer of a single command
 * 
 * @param command Command to send to the device
 * @return uint16_t Response from the device
 */
static uint16_t spi_transfer(uint16_t command) {
    tx_data[0] = command;
    spiSendReceiveArrays(tx_data, rx_data, 1);
    return rx_data[0];
}

/**
 * @brief Extract channel and conversion result from SPI response
 * 
 * @param response SPI response from the device
 * @param result Pointer to store the conversion result
 */
static void extract_conversion_result(uint16_t response, ConversionResult_t *result) {
    /* Extract channel number from response (bits 15-12) */
    result->channel = (response >> 12) & 0x0F;
    
    /* Extract conversion result based on resolution */
#if RESOLUTION == 12
    /* For 12-bit devices, data is in bits 11-0 */
    result->value = response & DATA_MASK;
#elif RESOLUTION == 10
    /* For 10-bit devices, data is in bits 9-0 */
    result->value = response & DATA_MASK;
#else /* RESOLUTION == 8 */
    /* For 8-bit devices, data is in bits 7-0 */
    result->value = response & DATA_MASK;
#endif
}

/**
 * @brief Get a command for the current mode and settings
 * 
 * @param keep_mode True to keep current mode, false to switch to MODE_CONT
 * @return uint16_t Command word with current settings
 */
static uint16_t get_current_settings_command(bool keep_mode) {
    /* Start with current mode or MODE_CONT */
    uint16_t command = keep_mode ? 
                     CMD_MODE(device_state.current_mode) : 
                     CMD_MODE(MODE_CONT);
    
    /* Add programming enable bit and current range */
    command |= MCR_ENABLE_PROG | CMD_RANGE(device_state.current_range);
    
    /* Prepare GPIO output values */
    uint8_t gpio_bits = 0;
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        if (device_state.gpio_is_output[i] && device_state.gpio_output_values[i]) {
            gpio_bits |= (1 << i);
        }
    }
    
    /* Add GPIO output values */
    command |= CMD_GPIO(gpio_bits);
    
    return command;
}

/**
 * @brief Wait for valid data from the pipeline
 * 
 * This function performs two SPI transfers to ensure the pipeline
 * is properly initialized before valid data can be read.
 */
static void initialize_pipeline(void) {
    uint16_t command = get_current_settings_command(true);
    
    /* First transfer initiates channel selection */
    spi_transfer(command);
    
    /* Second transfer starts acquisition, no valid data yet */
    spi_transfer(command);
}

/**
 * @brief Validate channel number against device capabilities
 * 
 * @param channel Channel number to validate
 * @return true if valid, false if invalid
 */
static bool validate_channel(uint8_t channel) {
    return (channel < CHANNELS);
}

/**
 * @brief Add a conversion result to the buffer
 * 
 * @param result Conversion result to add
 * @return true if successful, false if buffer is full
 */
static bool add_to_buffer(ConversionResult_t *result) {
    if (buffer_count >= ADS79XX_BUFFER_SIZE) {
        return false; /* Buffer is full */
    }
    
    conversion_buffer[buffer_count] = *result;
    buffer_count++;
    
    return true;
}

/************************** Public Function Implementations **************************/

void PowerDown(bool enable) {
    if (!is_initialized) {
        return;
    }
    
    uint16_t command = get_current_settings_command(true);
    
    if (enable) {
        /* Set power down bit */
        command |= MCR_POWERDOWN;
    } else {
        /* Clear power down bit */
        command &= ~MCR_POWERDOWN;
    }
    
    /* Send command */
    spi_transfer(command);
}

void SetRange(uint8_t range) {
    if (!is_initialized) {
        return;
    }
    
    /* Validate range */
    if (range != RANGE_VREF && range != RANGE_2VREF) {
        return;
    }
    
    /* Update device state */
    device_state.current_range = range;
    
    /* Build command with current settings */
    uint16_t command = get_current_settings_command(true);
    command &= ~(1 << MCR_RANGE_POS);  /* Clear range bit */
    command |= CMD_RANGE(range);       /* Set new range */
    
    /* Send command */
    spi_transfer(command);
}

void SetMode(Mode_t mode, bool reset_channel) {
    if (!is_initialized) {
        return;
    }
    
    /* Only update mode if not continuing current mode */
    if (mode != MODE_CONT) {
        device_state.current_mode = mode;
    }
    
    /* Build command with current settings */
    uint16_t command = CMD_MODE(mode) | 
                     MCR_ENABLE_PROG | 
                     CMD_RANGE(device_state.current_range);
    
    if (reset_channel) {
        command |= MCR_RESET_CHAN;
    }
    
    /* Prepare GPIO output values */
    uint8_t gpio_bits = 0;
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        if (device_state.gpio_is_output[i] && device_state.gpio_output_values[i]) {
            gpio_bits |= (1 << i);
        }
    }
    
    /* Add GPIO values */
    command |= CMD_GPIO(gpio_bits);
    
    /* Send command */
    spi_transfer(command);
    
    /* If mode changed, initialize pipeline for new mode */
    if (mode != MODE_CONT) {
        initialize_pipeline();
    }
}

bool ConfigureGPIOs(const GPIOConfig_t *config) {
    if (!is_initialized || config == NULL) {
        return false;
    }
    
    /* Build GPIO program register command */
    uint16_t command = (PRG_GPIO << 12);  /* GPIO programming register select */
    
    /* Add reset bit if requested */
    if (config->device_reset) {
        command |= GPIO_RESET;
    }
    
    /* Configure GPIO3 (TSSOP only) */
#if GPIO_COUNT > 1
    if (config->gpio3 == GPIO3_PD) {
        command |= GPIO_PD_FUNC;
        device_state.gpio_special_function[3] = true;
        device_state.gpio_is_output[3] = false;
    } else if (config->gpio3 == GPIO3_OUTPUT) {
        command |= (1 << 3);  /* Set direction bit for output */
        device_state.gpio_special_function[3] = false;
        device_state.gpio_is_output[3] = true;
    } else {
        device_state.gpio_special_function[3] = false;
        device_state.gpio_is_output[3] = false;
    }
    
    /* Configure GPIO2 (TSSOP only) */
    if (config->gpio2 == GPIO2_RANGE) {
        command |= GPIO_RANGE_FUNC;
        device_state.gpio_special_function[2] = true;
        device_state.gpio_is_output[2] = false;
    } else if (config->gpio2 == GPIO2_OUTPUT) {
        command |= (1 << 2);  /* Set direction bit for output */
        device_state.gpio_special_function[2] = false;
        device_state.gpio_is_output[2] = true;
    } else {
        device_state.gpio_special_function[2] = false;
        device_state.gpio_is_output[2] = false;
    }
    
    /* Configure GPIO1 (TSSOP only) */
    if (config->gpio1 == GPIO1_LOW_ALARM) {
        command |= GPIO_LOW_ALARM;
        device_state.gpio_special_function[1] = true;
        device_state.gpio_is_output[1] = false;
    } else if (config->gpio1 == GPIO1_OUTPUT) {
        command |= (1 << 1);  /* Set direction bit for output */
        device_state.gpio_special_function[1] = false;
        device_state.gpio_is_output[1] = true;
    } else {
        device_state.gpio_special_function[1] = false;
        device_state.gpio_is_output[1] = false;
    }
#endif
    
    /* Configure GPIO0 (available on all packages) */
    if (config->gpio0 == GPIO0_HIGH_ALARM) {
        command |= GPIO_HIGH_ALARM;
        device_state.gpio_special_function[0] = true;
        device_state.gpio_is_output[0] = false;
    } else if (config->gpio0 == GPIO0_ANY_ALARM) {
        command |= GPIO_ANY_ALARM;
        device_state.gpio_special_function[0] = true;
        device_state.gpio_is_output[0] = false;
    } else if (config->gpio0 == GPIO0_OUTPUT) {
        command |= (1 << 0);  /* Set direction bit for output */
        device_state.gpio_special_function[0] = false;
        device_state.gpio_is_output[0] = true;
    } else {
        device_state.gpio_special_function[0] = false;
        device_state.gpio_is_output[0] = false;
    }
    
    /* Send command to device */
    spi_transfer(command);
    
    return true;
}

bool WriteGPIO(uint8_t gpio_num, uint8_t value) {
    if (!is_initialized) {
        return false;
    }
    
    /* Validate GPIO number */
    if (gpio_num >= GPIO_COUNT) {
        return false;
    }
    
#if defined(PACKAGE_VQFN)
    /* VQFN package only has GPIO0 */
    if (gpio_num != 0) {
        return false;
    }
#endif
    
    /* Only write to GPIO configured as output */
    if (!device_state.gpio_is_output[gpio_num] || device_state.gpio_special_function[gpio_num]) {
        return false;
    }
    
    /* Update GPIO output value */
    device_state.gpio_output_values[gpio_num] = (value > 0) ? 1 : 0;
    
    /* Send command with updated GPIO values */
    uint16_t command = get_current_settings_command(true);
    
    spi_transfer(command);
    
    return true;
}

bool ReadGPIO(uint8_t gpio_num, uint8_t *value) {
    if (!is_initialized || value == NULL) {
        return false;
    }
    
    /* Validate GPIO number */
    if (gpio_num >= GPIO_COUNT) {
        return false;
    }
    
#if defined(PACKAGE_VQFN)
    /* VQFN package only has GPIO0 */
    if (gpio_num != 0) {
        return false;
    }
#endif
    
    /* For output pins, just return the stored value */
    if (device_state.gpio_is_output[gpio_num] && !device_state.gpio_special_function[gpio_num]) {
        *value = device_state.gpio_output_values[gpio_num];
        return true;
    }
    
    /* Enable GPIO read in command */
    uint16_t command = get_current_settings_command(true);
    command |= MCR_GPIO_READ;
    
    /* Send command and read response */
    uint16_t response = spi_transfer(command);
    
    /* Extract GPIO values from upper 4 bits (DO15-DO12) */
    uint8_t gpio_values = (response >> 12) & 0x0F;
    
    /* Extract requested GPIO value */
    *value = (gpio_values >> gpio_num) & 0x01;
    
    return true;
}

bool ConfigureAuto1Sequence(const ChannelSeq_t *seq) {
    if (!is_initialized || seq == NULL) {
        return false;
    }
    
    /* Save sequence configuration */
    device_state.auto1_sequence = *seq;
    
    /* Send Auto1 program sequence start command */
    uint16_t command = (PRG_AUTO1 << 12);
    spi_transfer(command);
    
    /* Send channel enable mask - limit to available channels */
    uint16_t channel_mask = seq->enabled_channels;
    
    /* Ensure only valid channels for this device variant are enabled */
#if CHANNELS == 4
    channel_mask &= 0x000F;  /* Only channels 0-3 */
#elif CHANNELS == 8
    channel_mask &= 0x00FF;  /* Only channels 0-7 */
#elif CHANNELS == 12
    channel_mask &= 0x0FFF;  /* Only channels 0-11 */
#endif
    
    /* Send channel mask in second frame */
    spi_transfer(channel_mask);
    
    return true;
}

bool ConfigureAuto2LastChannel(uint8_t last_channel) {
    if (!is_initialized) {
        return false;
    }
    
    /* Validate channel number */
    if (last_channel >= CHANNELS) {
        return false;
    }
    
    /* Update device state */
    device_state.auto2_last_channel = last_channel;
    
    /* Send Auto2 program command */
    uint16_t command = (PRG_AUTO2 << 12);
    
    /* Insert last channel value in bits D09-D06 per datasheet Table 6 */
    command |= ((last_channel & AUTO2_LAST_CHAN_MASK) << AUTO2_LAST_CHAN_POS);
    
    spi_transfer(command);
    
    return true;
}

bool ConfigureAlarm(const AlarmConfig_t *alarm_config) {
    if (!is_initialized || alarm_config == NULL) {
        return false;
    }
    
    /* Validate channel number */
    if (alarm_config->channel >= CHANNELS) {
        return false;
    }
    
    /* Validate alarm level */
    if (alarm_config->alarm_level != HIGH_ALARM_SEL && alarm_config->alarm_level != LOW_ALARM_SEL) {
        return false;
    }
    
    /* Validate threshold value */
    if (alarm_config->threshold > MAX_ALARM_THRESHOLD) {
        return false;
    }
    
    /* Determine alarm group based on channel number */
    uint8_t alarm_group = alarm_config->channel / 4;
    uint8_t group_channel = alarm_config->channel % 4;
    
    /* Send alarm group select command (first frame) */
    uint16_t command = ((PRG_ALARM_GROUP0 + alarm_group) << 12);
    spi_transfer(command);
    
    /* Prepare alarm configuration command (second frame) */
    command = (group_channel << ALARM_CHAN_POS);
    
    if (alarm_config->alarm_level == HIGH_ALARM_SEL) {
        command |= ALARM_HIGH;
    } else {
        command |= ALARM_LOW;
    }
    
    /* Exit alarm programming after this command */
    command |= ALARM_EXIT;
    
    /* Add threshold value */
    command |= (alarm_config->threshold & ALARM_THRESH_MASK);
    
    /* Send command */
    spi_transfer(command);
    
    return true;
}

bool SelectChannel(uint8_t channel) {
    if (!is_initialized) {
        return false;
    }
    
    /* Validate channel number */
    if (!validate_channel(channel)) {
        return false;
    }
    
    /* Update device state */
    device_state.last_selected_channel = channel;
    
    /* Prepare command for manual mode with channel selection */
    uint16_t command = CMD_MODE(MODE_MANUAL) |
                     MCR_ENABLE_PROG |
                     CMD_CHANNEL(channel) |
                     CMD_RANGE(device_state.current_range);
    
    /* Prepare GPIO output values */
    uint8_t gpio_bits = 0;
    for (uint8_t i = 0; i < GPIO_COUNT; i++) {
        if (device_state.gpio_is_output[i] && device_state.gpio_output_values[i]) {
            gpio_bits |= (1 << i);
        }
    }
    
    /* Add GPIO values */
    command |= CMD_GPIO(gpio_bits);
    
    spi_transfer(command);
    
    return true;
}

bool ReadConversion(ConversionResult_t *result) {
    if (!is_initialized || result == NULL) {
        return false;
    }
    
    /* Send command to continue current mode */
    uint16_t command = get_current_settings_command(true);
    
    /* Perform transfer and get response */
    uint16_t response = spi_transfer(command);
    
    /* Extract conversion result */
    extract_conversion_result(response, result);
    
    return true;
}

bool CaptureConversions(uint16_t num_conversions) {
    if (!is_initialized || num_conversions == 0) {
        return false;
    }
    
    /* Clear the conversion buffer */
    ClearConversionBuffer();
    
    /* Limit number of conversions to buffer size */
    if (num_conversions > ADS79XX_BUFFER_SIZE) {
        num_conversions = ADS79XX_BUFFER_SIZE;
    }
    
    /* Prepare command for continuing in current mode */
    uint16_t command = get_current_settings_command(true);
    
    /* Initialize pipeline if needed (first time) */
    if (buffer_count == 0) {
        initialize_pipeline();
    }
    
    /* Perform conversions and store results */
    for (uint16_t i = 0; i < num_conversions; i++) {
        uint16_t response = spi_transfer(command);
        ConversionResult_t result;
        
        /* Extract conversion result */
        extract_conversion_result(response, &result);
        
        /* Add to buffer */
        if (!add_to_buffer(&result)) {
            break; /* Buffer is full */
        }
    }
    
    return true;
}

uint16_t GetResults(ConversionResult_t *results, uint16_t max_results) {
    if (!is_initialized || results == NULL || max_results == 0) {
        return 0;
    }
    
    /* Calculate number of results to return */
    uint16_t count = (buffer_count < max_results) ? buffer_count : max_results;
    
    /* Copy results to output array */
    for (uint16_t i = 0; i < count; i++) {
        results[i] = conversion_buffer[i];
    }
    
    return count;
}

uint16_t GetResultCount(void) {
    return buffer_count;
}

void ClearConversionBuffer(void) {
    buffer_count = 0;
}