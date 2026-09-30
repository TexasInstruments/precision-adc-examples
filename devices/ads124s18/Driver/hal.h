/**
 * @file hal.h
 * @brief Hardware Abstraction Layer for MSPM0 microcontroller.
 *
 * Provides platform-specific implementations for the MSPM0G3507 microcontroller
 * including SPI communication, timing, and GPIO interrupt handling for the
 * ADS124S18 ADC interface.
 *
 * Platform: MSPM0G3507 (Arm Cortex-M0+ @ 32 MHz)
 *
 * Key interfaces:
 * - SPI communication: Configurable speed, 16-bit FIFO packing
 * - Timing: Microsecond and millisecond delays with 31MHz clock reference
 * - GPIO interrupts: DRDY (Data Ready) signal handling on GPIO pin
 *
 * Hardware Connections:
 * - SPI interface for ADS124S18 communication
 * - GPIO pin for DRDY (Data Ready) interrupt
 * - CS (Chip Select) line for SPI slave control
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

#include "ti/driverlib/m0p/dl_core.h"

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

/**
 * @brief Transmit and receive SPI data with byte-level framing.
 * @param dataTx[] Array of bytes to transmit (must be bufferLength bytes)
 * @param dataRx[] Array to receive SPI response (must be bufferLength bytes)
 * @param bufferLength Number of bytes to exchange
 *
 * Sends and receives data over SPI interface. The MSPM0 SPI peripheral
 * uses a 16-bit FIFO, so bytes are packed/unpacked for transfer. This
 * function handles the packing transparently.
 *
 * Frame format: Standard SPI 8-bit mode with CPOL=0, CPHA=0.
 * Typical SPI clock: 1-10 MHz (configure in ti_msp_dl_config.h)
 *
 * @note Blocking operation - waits for SPI completion before returning.
 */
void spiSendReceiveArrays(uint8_t dataTx[], uint8_t dataRx[], uint8_t bufferLength);

/**
 * @brief Delay execution for specified microseconds.
 * @param delay_time_us Delay duration in microseconds
 *
 * Blocking microsecond delay using cycle counting. Clock frequency
 * assumed: 31 MHz (typical MSPM0G3507 configuration).
 *
 * Accuracy: ±1 microsecond (subject to instruction timing variations)
 * Overhead: ~10-20µs from function entry/exit
 * Typical usage: Wait for device settling times, CS hold times
 *
 * @note For delays > 100ms, use delay_ms() instead for better accuracy.
 */
void delay_us(const uint32_t delay_time_us);

/**
 * @brief Delay execution for specified milliseconds.
 * @param delay_time_ms Delay duration in milliseconds
 *
 * Blocking millisecond delay using cycle counting. Clock frequency
 * assumed: 31 MHz (typical MSPM0G3507 configuration).
 *
 * Accuracy: ±1 millisecond
 * Typical usage: Power mode transitions, acquisition intervals
 *
 * @note Suitable for delays up to several seconds.
 */
void delay_ms(const uint32_t delay_time_ms);

/**
 * @brief Wait for ADC Data Ready (DRDY) interrupt with timeout.
 * @param timeout_ms Maximum wait time in milliseconds
 * @return true if DRDY occurred before timeout, false if timeout expired
 *
 * Blocks until DRDY interrupt (falling edge on GPIO pin) occurs or
 * timeout expires. Uses interrupt-driven signaling for minimal CPU overhead.
 *
 * GPIO Configuration: DRDY on specific MSPM0 GPIO port/pin (set via pinmux)
 * Timeout range: 0-4294 ms (uint32_t milliseconds)
 *
 * ISR Handler: GROUP1_IRQHandler() sets flag on DRDY falling edge
 *
 * Typical usage:
 * - Wait before each readData() call
 * - Typical DRDY latency: 100-500µs after conversion complete
 * - Maximum wait: ~100ms for slowest OSR settings
 *
 * @note Returns false if timeout occurs - check return value to detect stuck ADC.
 */
bool waitForDRDYinterrupt(uint32_t timeout_ms);

/**
 * @brief GPIO interrupt handler for DRDY signal.
 * @param numCaptured Not used (legacy parameter)
 * @param streamRx[] Not used (legacy parameter)
 *
 * Called by MSPM0 interrupt vector when DRDY GPIO pin transitions.
 * Sets the global flag_nDRDY_INTERRUPT flag and disables further interrupts
 * until next waitForDRDYinterrupt() call.
 *
 * This is the ISR for GROUP1 interrupts - configured in ti_msp_dl_config.h
 * for the specific GPIO port handling DRDY.
 *
 * @note ISR context - keep processing minimal. Do not call blocking functions.
 */
void GROUP1_IRQHandler(uint32_t numCaptured, uint32_t streamRx[]);