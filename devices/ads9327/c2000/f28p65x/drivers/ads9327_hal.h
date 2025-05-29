/**
 *
 * \copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ADS9327_HAL_H_
#define ADS9327_HAL_H_

#ifdef __cplusplus

extern "C" {
#endif




//
// Included Files
//

#include "driverlib.h"
#include <ads9327_regmap.h>
#include "device.h"
#include <stdint.h>



// ADCdata arrays for interrupt functions
extern uint16_t temp_var1;
extern uint16_t temp_var2;
extern uint16_t ADCA_data;
extern uint16_t ADCB_data;
extern uint16_t data_now_A_MSB, data_now_A_LSB, data_now_B_MSB, data_now_B_LSB;
extern uint16_t fifoADC_A[MAX_ENTRIES];
extern uint16_t fifoADC_B[MAX_ENTRIES];
extern uint16_t fifoIndexA;
extern uint16_t fifoIndexB;

//*****************************************************************************
//
// ADC Functions
//
//*****************************************************************************
void ADS9327_HAL_DeviceInit();
void ADS9327_HAL_configSpiExtAdcDataRead();
void ADS9327_HAL_delay(int milliseconds);
void ADS9327_HAL_writeRegister(uint8_t reg_addr, uint16_t reg_value);
void ADS9327_HAL_setupInterrupts();

//
//==============================================================================
// ADS9327_HAL_1LaneModeISR1 - Interrupt function ISR1 for 1 Lane Mode
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS9327_HAL_1LaneModeISR1)
static inline void ADS9327_HAL_1LaneModeISR1(void){
    
    SPI_writeDataNonBlocking(mySPI0_BASE, 0x0000);
    SPI_writeDataNonBlocking(mySPI0_BASE, 0x0000);
    SPI_writeDataNonBlocking(mySPI0_BASE, 0x0000);


}

//
//==============================================================================
// ADS9327_HAL_1LaneModeISR2 - Interrupt function ISR2 for 1 Lane Mode
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS9327_HAL_1LaneModeISR2)
static inline void ADS9327_HAL_1LaneModeISR2(void){
    
    ADCA_data = SPI_readDataNonBlocking(mySPI0_BASE);
    temp_var1 = SPI_readDataNonBlocking(mySPI0_BASE);
    temp_var2 = SPI_readDataNonBlocking(mySPI0_BASE);

    // arrange ADCB data into the correct format 
    ADCB_data = (temp_var1 << 8)|(temp_var2 >> 8);  

    if (fifoIndexA < MAX_ENTRIES) {
        fifoADC_A[fifoIndexA++] = ADCA_data;
        fifoADC_B[fifoIndexB++] = ADCB_data;
    }

}

//
//==============================================================================
// ADS9327_HAL_4LaneModeISR1 - Interrupt function ISR1 for 4 Lane Mode
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS9327_HAL_4LaneModeISR1)
static inline void ADS9327_HAL_4LaneModeISR1(void){

    //
    // ISR is triggered by the EPWM
    //

    data_now_A_MSB = CLB_getRegister(CLB5_BASE, CLB_REG_HLC_R0);
    data_now_A_LSB = CLB_getRegister(CLB5_BASE, CLB_REG_HLC_R1);

    data_now_B_MSB = CLB_getRegister(CLB6_BASE, CLB_REG_HLC_R0);
    data_now_B_LSB = CLB_getRegister(CLB6_BASE, CLB_REG_HLC_R1);

    if (fifoIndexA < MAX_ENTRIES) {
        fifoADC_A[fifoIndexA++] = (uint16_t)(data_now_A_MSB << 8)|(data_now_A_LSB);
        fifoADC_B[fifoIndexB++] = (uint16_t)(data_now_B_MSB << 8)|(data_now_B_LSB); 
    }


    //
    // Write data to the transmit buffer.
    //
    HWREGH(mySPI0_BASE + SPI_O_TXBUF) = 0x0000;

   
}

//
//==============================================================================
// ADS9327_HAL_clearAckISR1 - Clear interrupts ACK for ISR1
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS9327_HAL_clearAckISR1)
static inline void ADS9327_HAL_clearAckISR1(void){



    EPWM_clearEventTriggerInterruptFlag(ADS9327_EXT_ADC1_CONVST_EPWM_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

}

//
//==============================================================================
// ADS9327_HAL_clearAckISR2 - Clear interrupts ACK for ISR2
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS9327_HAL_clearAckISR2)
static inline void ADS9327_HAL_clearAckISR2(void){



    SPI_clearInterruptStatus(mySPI0_BASE, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);

}

void ADS9327_HAL_setupEpwmExtAdcStart();
void ADS9327_HAL_setupGpioExtAdcStart();
void ADS9327_HAL_enableEpwmCounting();
void ADS9327_HAL_disableEpwmCounting();

//*****************************************************************************
//
// Interrupt Functions
//
//*****************************************************************************
__interrupt void ISR1(void);
__interrupt void ISR2(void);


//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************
//
// GPIO48 - GPIO Settings
//
#define myGPIO0_GPIO_PIN_CONFIG GPIO_48_GPIO48
//
// GPIO31 - GPIO Settings
//
#define myGPIO1_GPIO_PIN_CONFIG GPIO_31_GPIO31
//
// GPIO10 - GPIO Settings
//
#define CLB_input1_GPIO_PIN_CONFIG GPIO_10_GPIO10
//

#if DATA_LANES_SELECT == 4
    // GPIO94 - GPIO Settings
    //
    #define nChip_select_GPIO_PIN_CONFIG GPIO_94_GPIO94
#endif

//
// GPIO115 - GPIO Settings
//
#define ISR_check_GPIO_PIN_CONFIG GPIO_115_GPIO115
//
// GPIO11 - GPIO Settings
//
#define CLB_input2_GPIO_PIN_CONFIG GPIO_11_GPIO11
//
// GPIO8 - GPIO Settings
//
#define CLB_input3_GPIO_PIN_CONFIG GPIO_8_GPIO8
//
// GPIO9 - GPIO Settings
//
#define CLB_input4_GPIO_PIN_CONFIG GPIO_9_GPIO9

//
// SPID -> mySPI0 Pinmux
//
//
// SPID_PICO - GPIO Settings
//
#define GPIO_PIN_SPID_PICO 91
#define mySPI0_SPIPICO_GPIO 91
#define mySPI0_SPIPICO_PIN_CONFIG GPIO_91_SPID_PICO
//
// SPID_POCI - GPIO Settings
//
#define GPIO_PIN_SPID_POCI 92
#define mySPI0_SPIPOCI_GPIO 92
#define mySPI0_SPIPOCI_PIN_CONFIG GPIO_92_SPID_POCI
//
// SPID_CLK - GPIO Settings
//
#define GPIO_PIN_SPID_CLK 93
#define mySPI0_SPICLK_GPIO 93
#define mySPI0_SPICLK_PIN_CONFIG GPIO_93_SPID_CLK

#if DATA_LANES_SELECT == 1
    #define GPIO_PIN_SPID_PTE 94
    #define mySPI0_SPIPTE_GPIO 94
    #define mySPI0_SPIPTE_PIN_CONFIG GPIO_94_SPID_PTE
#endif  


//*****************************************************************************
//
// CLB Configurations
//
//*****************************************************************************
#define D3_D2_BASE CLB5_BASE
void D3_D2_init();
#define D1_D0_BASE CLB6_BASE
void D1_D0_init();
//
// Tile Configurations for all CLBs are in this file
//

//*****************************************************************************
//
// CLBINPUTXBAR Configurations
//
//*****************************************************************************
#define D3_SOURCE 10
#define D3_INPUT XBAR_INPUT1
void D3_init();
#define D2_SOURCE 11
#define D2_INPUT XBAR_INPUT2
void D2_init();
#define D1_SOURCE 8
#define D1_INPUT XBAR_INPUT3
void D1_init();
#define D0_SOURCE 9
#define D0_INPUT XBAR_INPUT4
void D0_init();

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define myGPIO0 48
void myGPIO0_init();
#define myGPIO1 31
void myGPIO1_init();
#define CLB_input1 10
void CLB_input1_init();

#if DATA_LANES_SELECT == 4
#define nChip_select 94
void nChip_select_init();
#endif

#define ISR_check 115
void ISR_check_init();
#define CLB_input2 11
void CLB_input2_init();
#define CLB_input3 8
void CLB_input3_init();
#define CLB_input4 9
void CLB_input4_init();

//*****************************************************************************
//
// MEMCFG Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// SPI Configurations
//
//*****************************************************************************
void mySPI0_init();

//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	ADS9327_HAL_Board_init();
void	CLB_init();
void	CLB_INPUTXBAR_init();
void	GPIO_init();
void	MEMCFG_init();
void	SPI_init();
void	SYSCTL_init();
void	PinMux_init();

//*****************************************************************************
//
// ePWM - ADC CONVST
//
//*****************************************************************************

void ADS9327_HAL_disableEpwmCounting(void);
void ADS9327_HAL_enableEpwmCounting(void);

void ADS9327_HAL_setupGpioExtAdcStart(void);
void ADS9327_HAL_setupEpwmExtAdcStart(void);

//*****************************************************************************
//
// CLB configuration
//
//*****************************************************************************

// HLC Instruction Register Field definitions
#define HLC_OPCODE_R0 0x0
#define HLC_OPCODE_R1 0x1
#define HLC_OPCODE_R2 0x2
#define HLC_OPCODE_R3 0x3
#define HLC_OPCODE_C0 0x4
#define HLC_OPCODE_C1 0x5
#define HLC_OPCODE_C2 0x6

#define HLC_OPCODE_MOV    0x00
#define HLC_OPCODE_MOV_T1 0x01
#define HLC_OPCODE_MOV_T2 0x02
#define HLC_OPCODE_PUSH   0x03
#define HLC_OPCODE_PULL   0x04
#define HLC_OPCODE_ADD    0x05
#define HLC_OPCODE_SUB    0x06
#define HLC_OPCODE_INTR   0x07

//---------------------------------------------------------------------------
// TILE0
//---------------------------------------------------------------------------
#define TILE0_PIPELINE_MODE 0
#define TILE0_CFG_OUTLUT_0 0x0
#define TILE0_CFG_OUTLUT_1 0x0
#define TILE0_CFG_OUTLUT_2 0x0
#define TILE0_CFG_OUTLUT_3 0x0
#define TILE0_CFG_OUTLUT_4 0x0
#define TILE0_CFG_OUTLUT_5 0x0
#define TILE0_CFG_OUTLUT_6 0x0
#define TILE0_CFG_OUTLUT_7 0x0

#define TILE0_CFG_LUT4_IN0   0x18
#define TILE0_CFG_LUT4_IN1   0x19
#define TILE0_CFG_LUT4_IN2   0x0
#define TILE0_CFG_LUT4_IN3   0x0
#define TILE0_CFG_LUT4_FN10  ((0x00000) | 0x2222)
#define TILE0_CFG_LUT4_FN2   0x0

#define TILE0_CFG_FSM_EXT_IN0      0x0
#define TILE0_CFG_FSM_EXT_IN1      0x0
#define TILE0_CFG_FSM_EXTRA_IN0    0x0
#define TILE0_CFG_FSM_EXTRA_IN1    0x0
#define TILE0_CFG_FSM_NEXT_STATE_0 ((0x00000) | 0x0)
#define TILE0_CFG_FSM_NEXT_STATE_1 ((0x00000) | 0x0)
#define TILE0_CFG_FSM_NEXT_STATE_2 ((0x00000) | 0x0)
#define TILE0_CFG_FSM_LUT_FN10     ((0x00000) | 0x0)
#define TILE0_CFG_FSM_LUT_FN2      0x0
#define TILE0_FSM_MISC_CONTROL     0x0

#define TILE0_CFG_COUNTER_RESET   0x3bd
#define TILE0_CFG_COUNTER_EVENT   0x39b
#define TILE0_CFG_COUNTER_MODE_0  0xe7
#define TILE0_CFG_COUNTER_MODE_1  0x108
#define TILE0_CFG_TAP_SEL          0x0
#define TILE0_CFG_MISC_CONTROL    (0x624 | TILE0_FSM_MISC_CONTROL)

#define TILE0_COUNTER_0_MATCH1_VAL  0
#define TILE0_COUNTER_0_MATCH2_VAL  0
#define TILE0_COUNTER_0_LOAD_VAL    0
#define TILE0_COUNTER_1_MATCH1_VAL  0
#define TILE0_COUNTER_1_MATCH2_VAL  0
#define TILE0_COUNTER_1_LOAD_VAL    0
#define TILE0_COUNTER_2_MATCH1_VAL  0
#define TILE0_COUNTER_2_MATCH2_VAL  0
#define TILE0_COUNTER_2_LOAD_VAL    0


#define TILE0_SPI_EN 0

#define TILE0_HLC_EVENT_SEL 0x1a
#define TILE0_HLC_R0_INIT 0
#define TILE0_HLC_R1_INIT 0
#define TILE0_HLC_R2_INIT 0
#define TILE0_HLC_R3_INIT 0

#define TILE0_HLC_FIFO0_INIT 0
#define TILE0_HLC_FIFO1_INIT 0
#define TILE0_HLC_FIFO2_INIT 0
#define TILE0_HLC_FIFO3_INIT 0

#define TILE0_HLCINSTR_0	(0 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_C0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_1	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_C1<<3 | HLC_OPCODE_R1)
#define TILE0_HLCINSTR_2	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_3	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_4	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_5	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_6	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_7	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_8	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_9	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_10	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_11	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_12	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_13	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_14	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_15	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_16	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_17	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_18	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_19	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_20	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_21	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_22	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_23	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_24	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_25	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_26	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_27	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_28	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_29	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_30	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)
#define TILE0_HLCINSTR_31	(1 << 11 | HLC_OPCODE_MOV << 6 | HLC_OPCODE_R0<<3 | HLC_OPCODE_R0)




#define TILE0_OUTPUT_COND_CTR_0 0x0
#define TILE0_OUTPUT_COND_CTR_1 0x0
#define TILE0_OUTPUT_COND_CTR_2 0x0
#define TILE0_OUTPUT_COND_CTR_3 0x0
#define TILE0_OUTPUT_COND_CTR_4 0x0
#define TILE0_OUTPUT_COND_CTR_5 0x0
#define TILE0_OUTPUT_COND_CTR_6 0x0
#define TILE0_OUTPUT_COND_CTR_7 0x0

void initTILE0(uint32_t base);

#ifdef __cplusplus
}
#endif  

#endif /* ADS9327_HAL_H_ */
