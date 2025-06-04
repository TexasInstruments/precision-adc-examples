/**
 * @file ads79xx-main.c
 * @brief Example application using the ADS79xx ADC driver
 */

#include <stdint.h>
#include <stdbool.h>
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "evm/ads79xx.h"

int main(void) {
    ConversionResult_t results[10];
    uint16_t count;
    uint8_t gpio_value;
    
    /* System initialization */
    SYSCFG_DL_init();
    
    /* ----- DEVICE CONFIGURATION ----- */
    
    /* Start with setting the device mode and range */
    SetMode(MODE_MANUAL, true);  // Manual mode with channel reset
    SetRange(RANGE_VREF);        // 0-VREF input range
    
    /* Configure GPIO0 as high alarm output */
    GPIOConfig_t gpio_config;
    
    /* Initialize with default values */
    gpio_config.gpio0 = GPIO0_HIGH_ALARM;
#if GPIO_COUNT > 1
    gpio_config.gpio1 = GPIO1_INPUT;
    gpio_config.gpio2 = GPIO2_INPUT;
    gpio_config.gpio3 = GPIO3_INPUT;
#endif
    gpio_config.device_reset = false;
    
    /* Apply GPIO configuration */
    ConfigureGPIOs(&gpio_config);
    
    /* Configure alarm for channel 0 */
    AlarmConfig_t alarm;
    alarm.channel = 0;
    alarm.alarm_level = HIGH_ALARM_SEL;
    alarm.threshold = 2000;  // Adjust based on expected signals
    
    ConfigureAlarm(&alarm);
    
    /* Clear any previous results */
    ClearConversionBuffer();
    
    /* ----- MANUAL MODE OPERATION ----- */
    
    /* Select channel 0 for conversion */
    SelectChannel(0);
    
    /* Capture 5 conversions in manual mode */
    CaptureConversions(5);
    
    /* Retrieve conversion results */
    count = GetResults(results, 5);
    
    /* ----- AUTO-1 MODE OPERATION ----- */
    
    /* Configure Auto-1 to scan channels 0, 2, and 4 only */
    ChannelSeq_t seq;
    seq.enabled_channels = (1 << 0) | (1 << 2) | (1 << 4);
    
    ConfigureAuto1Sequence(&seq);
    
    /* Switch to Auto-1 mode with channel reset */
    SetMode(MODE_AUTO1, true);
    
    /* Clear previous results and capture new conversions */
    ClearConversionBuffer();
    CaptureConversions(10);
    
    /* Get Auto-1 results */
    count = GetResults(results, 10);
    
    /* ----- AUTO-2 MODE OPERATION ----- */
    
    /* Configure Auto-2 to scan channels 0-3 */
    ConfigureAuto2LastChannel(3);
    
    /* Switch to Auto-2 mode with channel reset */
    SetMode(MODE_AUTO2, true);
    
    /* Clear buffer and capture new conversions */
    ClearConversionBuffer();
    CaptureConversions(10);
    
    /* Get Auto-2 results */
    count = GetResults(results, 10);
    
    /* ----- RANGE SWITCHING ----- */
    
    /* Switch back to manual mode on channel 0 */
    SetMode(MODE_MANUAL, true);
    SelectChannel(0);
    
    /* Change to extended input range (0-2VREF) */
    SetRange(RANGE_2VREF);
    
    /* Capture samples with new range */
    ClearConversionBuffer();
    CaptureConversions(5);
    
    /* Get results with new range */
    count = GetResults(results, 5);
    
    /* ----- GPIO OPERATIONS ----- */
    
    /* Reconfigure GPIO0 as output */
    gpio_config.gpio0 = GPIO0_OUTPUT;
    ConfigureGPIOs(&gpio_config);
    
    /* Toggle GPIO output */
    WriteGPIO(0, 1);  // Set high
    
    /* Use the built-in millisecond delay function */
    //DL_Common_delayMilliseconds(500);
    
    WriteGPIO(0, 0);  // Set low
    
    /* Read GPIO value */
    ReadGPIO(0, &gpio_value);
    
    /* ----- POWER MANAGEMENT ----- */
    
    /* Put device in power-down mode to save power */
    PowerDown(true);
    
    /* Wake up device when needed again */
    PowerDown(false);
    
    /* Main application loop */
    while (1) {
        /* Application code here */
        //delay_cycles(cycles);
    }
    
    return 0;
}