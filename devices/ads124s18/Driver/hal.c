/**
 * @file hal.c
 * @brief Hardware Abstraction Layer implementation for MSPM0G3507.
 *
 * Implements the platform-specific hardware interface for the ADS124S18 ADC
 * driver on the Texas Instruments MSPM0G3507 microcontroller.
 *
 * Hardware Features:
 * - SPI Interface: Configurable speed, 16-bit FIFO with byte packing
 * - GPIO Interrupts: DRDY (Data Ready) signal handling via GROUP1
 * - Timing: Cycle-counted delays for microsecond/millisecond precision
 * - Clock Reference: 31 MHz system clock (MSPM0 typical frequency)
 *
 * Timing Calibration:
 * The delay functions use the formula: delayTime = (31e6 / precision) * desired_delay
 * Where precision is 1000 for ms, 1000000 for µs.
 * This accounts for ~31 cycles per microsecond at 31 MHz operation.
 *
 * SPI Communication:
 * The MSPM0 SPI peripheral has a 16-bit FIFO, but ADS124S18 uses 8-bit frames.
 * spiSendReceiveArrays() handles packing 2x8-bit values into 16-bit FIFO elements
 * and unpacking responses transparently.
 *
 * Interrupt-Driven DRDY:
 * Instead of polling, DRDY (nDRDY) is monitored via GPIO interrupts.
 * Falling edge on DRDY sets flag_nDRDY_INTERRUPT, waking waitForDRDYinterrupt().
 * This minimizes CPU overhead during conversions.
 *
 * Configuration Dependencies (from ti_msp_dl_config.h):
 * - GPIOA setup with DRDY as input with falling-edge interrupt
 * - SPI peripheral configured with correct CLK, MOSI, MISO pins
 * - GROUP1 interrupt vector routed to GROUP1_IRQHandler
 * - System clock configured for 31 MHz operation
 *
 * @copyright Copyright (c) 2024-2026, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Driver/hal.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_spi.h"
#include "ti/driverlib/dl_spi.h"
#include "ti_msp_dl_config.h"

volatile bool flag_nDRDY_INTERRUPT = false;  //!< Interrupt flag set by DRDY GPIO ISR

//****************************************************************************/
//
// Timing Functions - Cycle-based delays using 31 MHz reference clock
//
// Timing formula: delay_cycles(N) where N = frequency_hz / precision * desired_delay
// At 31 MHz: 31 * 1e6 = 31,000,000 cycles/second
// Example: 1 ms delay = (31e6 / 1000) * 1 = 31,000 cycles
//
/*****************************************************************************/

/**
 * @brief Millisecond delay using cycle counting.
 * Calculates cycle count for specified ms at 31 MHz clock and blocks.
 * Accuracy: ±1ms (subject to interrupt jitter)
 */
void delay_ms(const uint32_t delay_time_ms)
{
    // 31 MHz clock: 31,000 cycles per millisecond
    uint32_t delayTime = 31e+6 / (1000u);
    delayTime = delayTime * delay_time_ms;
    delay_cycles(delayTime);
}

/**
 * @brief Microsecond delay using cycle counting.
 * Calculates cycle count for specified µs at 31 MHz clock and blocks.
 * Accuracy: ±1µs (subject to instruction execution variations)
 * Minimum practical delay: ~20-50µs due to function overhead
 */
void delay_us(const uint32_t delay_time_us)
{
    // 31 MHz clock: ~31 cycles per microsecond
    uint32_t delayTime = 31e+6 / (1000000u);
    delayTime = delayTime * delay_time_us;
    delay_cycles(delayTime);
}

/**
 * @brief Wait for DRDY interrupt with timeout detection.
 * Blocks until falling edge detected on DRDY GPIO or timeout occurs.
 * Uses interrupt-driven signaling for efficiency.
 */
bool waitForDRDYinterrupt(uint32_t timeout_ms)
{
    // Calculate timeout cycles: 31 MHz * desired timeout in ms
    uint32_t timeout = 31e+6 / (1000u);
    timeout = timeout_ms * timeout;

    // Reset interrupt flag and enable GPIO interrupt
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_0_DRDYn_PIN);
    flag_nDRDY_INTERRUPT = false;
    DL_GPIO_enableInterrupt(GPIOA, GPIO_GRP_0_DRDYn_PIN);

    // Wait for DRDY interrupt or timeout expiration
    // ISR sets flag_nDRDY_INTERRUPT when DRDY falling edge detected
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT);

    // Return true if interrupt occurred before timeout, false if timeout expired
    return (timeout > 0);
}

//****************************************************************************/
//
// GPIO Interrupt Handler - DRDY Signal Processing
//
// This ISR is triggered on falling edge of DRDY (Data Ready) signal from ADC.
// Sets a flag to wake the waiting task and prevents repeated interrupts.
//
/*****************************************************************************/

/**
 * @brief GPIO GROUP1 interrupt handler - DRDY falling edge detection.
 * Called on DRDY falling edge. Sets flag_nDRDY_INTERRUPT and disables
 * further interrupts until next waitForDRDYinterrupt() call.
 */
void GROUP1_IRQHandler(uint32_t numCaptured, uint32_t streamRx[])
{
    // Identify which interrupt in GROUP1 fired (may have multiple)
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_GRP_0_INT_IIDX:
            // DRDY interrupt detected (falling edge)
            // Set flag to wake waiting task
            flag_nDRDY_INTERRUPT = true;
            // Disable interrupt to prevent re-triggering until re-enabled
            DL_GPIO_disableInterrupt(GPIOA, GPIO_GRP_0_DRDYn_PIN);
            break;
    }
}

//****************************************************************************/
//
// SPI Communication Interface
//
// The MSPM0 SPI peripheral uses 16-bit FIFO elements, but ADS124S18 requires
// 8-bit byte communication. This function handles transparent packing/unpacking
// of bytes into 16-bit FIFO frames for efficient data transfer.
//
/*****************************************************************************/

/**
 * @brief SPI transmit and receive with 16-bit FIFO packing.
 *
 * Packs dataTx[] bytes into 16-bit FIFO frames, transmits to ADS124S18,
 * and unpacks received 16-bit frames back into dataRx[] bytes.
 *
 * FIFO Packing Strategy:
 * - Pairs of 8-bit bytes [byte0, byte1] → packed into 16-bit word
 * - Odd length buffers: last byte transmitted with MSB padding
 * - Received data follows same packing format
 *
 * SPI Frame Format (typical ADS124S18):
 * - First byte: Command or data MSB
 * - Second byte: Data LSB or status
 * - Optional third byte: CRC (if SPI_CRC_ENABLED)
 */
void spiSendReceiveArrays(uint8_t dataTx[], uint8_t dataRx[], uint8_t bufferLength)
// SPI transfers via 16-bit FIFO with automatic byte packing/unpacking 
// this transformation improves data readback when SPI frames include SPI and CRC words ( 6-byte frame )

{
    DL_SPI_drainRXFIFO8(SPI_0_INST, dataRx, 4);


    uint8_t i = 0;
    uint8_t j = 0;


    DL_GPIO_clearPins(GPIOB, GPIO_GRP_1_CSn_PIN);

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

    DL_GPIO_setPins(GPIOB, GPIO_GRP_1_CSn_PIN);


     
}