/*
 * @copyright Copyright (C) 2025-2026 Texas Instruments Incorporated - http://www.ti.com/
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
 */

//
//=============================================================================
// Includes
//=============================================================================
//
#include <ads93xxv.h>

//
// ADS93XXV_regBankSel - Selects register bank
//
void ADS93XXV_regBankSel(uint8_t bank)
{
    //
    // Select desired register bank
    // Bank0: Common Registers
    // Bank1: Isns Registers
    // Bank2: Vsns Registers
    //
    if (bank == 0)
    {
        //
        // Select Bank 0
        //
        ADS93XXV_HAL_writeRegister(BANK0_BANK_SEL_ADDRESS,
                                   BANK0_BANK_SEL_COMMONREGISTERS);
    }
    else if (bank == 1) 
    {
        //
        // Select bank 1
        //
        ADS93XXV_HAL_writeRegister(BANK0_BANK_SEL_ADDRESS,
                                   BANK0_BANK_SEL_CHANNELREGISTERSAIN1AIN8);
    }
    else if (bank == 2)
    {
        //
        // Select bank 2
        //
        ADS93XXV_HAL_writeRegister(BANK0_BANK_SEL_ADDRESS,
                                   BANK0_BANK_SEL_CHANNELREGISTERSAIN9AIN16);
    } 
}

//
// ADS93XXV_swResetAdc - Reset ADC
//
void ADS93XXV_swResetAdc()
{
    
    //reset ADC and all ADC registers
    ADS93XXV_regBankSel(0);
    ADS93XXV_HAL_writeRegister(BANK0_GEN_CFG1_ADDRESS,
                               ((1 << BANK0_GEN_CFG1_SW_RST_BITOFFSET)
                                       & BANK0_GEN_CFG1_SW_RST_MASK));
}

//
// Configure ADC is single lane mode: ADC output is SDOUT pin
//
void ADS93XXV_setupAdcOutOnSdoutSize_16b(void){
    ADS93XXV_HAL_writeRegister(0x02, 0x0001);
    ADS93XXV_HAL_writeRegister(0x0A, 0x0032);
}

void ADS93XXV_setupAdcOutOnSdoutSize_24b(void){
    ADS93XXV_HAL_writeRegister(0x02, 0x0001);
    ADS93XXV_HAL_writeRegister(0x0A, 0x003A);
}

//
// Configure ADC is single lane mode: ADC output is D7
// SDOUT: Register read output
//

void ADS93XXV_setupRegReadOutOnSdout(void){
    ADS93XXV_HAL_writeRegister(0x02, 0x0001);
    ADS93XXV_HAL_writeRegister(0x0A, 0x0030);
}

//
// ADC initialization (must be done every time at powerup)
//
void ADS93XXV_initalization(void)
{

    ADS93XXV_HAL_delay_ms(1);

    ADS93XXV_HAL_writeRegister(0x02, 0x0001);
    ADS93XXV_HAL_writeRegister(0x11, 0x0001);

    ADS93XXV_HAL_writeRegister(0x02, 0x0001);
    ADS93XXV_HAL_writeRegister(0x08, 0x5050);
    ADS93XXV_HAL_writeRegister(0x09, 0x5050);
    ADS93XXV_HAL_writeRegister(0x0A, 0x5050);
    ADS93XXV_HAL_writeRegister(0x0B, 0x5050);

    ADS93XXV_HAL_writeRegister(0x02, 0x0004);
    ADS93XXV_HAL_writeRegister(0x08, 0x5050);
    ADS93XXV_HAL_writeRegister(0x09, 0x5050);
    ADS93XXV_HAL_writeRegister(0x0A, 0x5050);
    ADS93XXV_HAL_writeRegister(0x0B, 0x5050);

    //
    // Delay for reference settling
    //


    ADS93XXV_HAL_delay_ms(20);
}

void ADS93XXV_setOsr(void)
{

//    ADS93XXV_HAL_writeRegister(BANK0_BANK_SEL_ADDRESS,
//                               BANK0_BANK_SEL_COMMONREGISTERS);
//    ADS93XXV_HAL_readModiFyWriteRegister(BANK0_DIG_FILTER_ADDRESS,
//                                         BANK0_DIG_FILTER_BLK_AVG_OSR_8SAMPLESAVERAGED,
//                                         BANK0_DIG_FILTER_BLK_AVG_OSR_MASK);
//
//    ADS93XXV_HAL_readModiFyWriteRegister(BANK0_DIG_FILTER_ADDRESS,
//                                         BANK0_DIG_FILTER_AVG_AUTO_TRIGGER_EN_ENABLED,
//                                         BANK0_DIG_FILTER_AVG_AUTO_TRIGGER_EN_MASK);
//
//    ADS93XXV_HAL_readModiFyWriteRegister(BANK0_DIG_FILTER_ADDRESS,
//                                         0x0001,
//                                         BANK0_DIG_FILTER_DIGITAL_FILTER_RST_MASK);

    ADS93XXV_HAL_writeRegister(0x02,0x01);
    ADS93XXV_HAL_writeRegister(0x14,0x41);

}





void ADS93XXV_setRangeVsns_6V25(void){
    ADS93XXV_HAL_writeRegister(0x02, 0x0002);
    ADS93XXV_HAL_writeRegister(0x08, 0x5353);
    ADS93XXV_HAL_writeRegister(0x09, 0x5353);
    ADS93XXV_HAL_writeRegister(0x0A, 0x5353);
    ADS93XXV_HAL_writeRegister(0x0B, 0x5353);
    ADS93XXV_HAL_writeRegister(0x02, 0x0004);
    ADS93XXV_HAL_writeRegister(0x08, 0x5353);
    ADS93XXV_HAL_writeRegister(0x09, 0x5353);
    ADS93XXV_HAL_writeRegister(0x0A, 0x5353);
    ADS93XXV_HAL_writeRegister(0x0B, 0x5353);
}


void ADS93XXV_initExtAdc(){

}

void ADS93XXV_disableExtAdcTestPattern(){

    ADS93XXV_HAL_writeRegister(0x02, 0x01);
    ADS93XXV_HAL_writeRegister(0x15, 0x00);
    ADS93XXV_HAL_writeRegister(0x02, 0x2);
    ADS93XXV_HAL_writeRegister(0x2E,0x00);// Enable constant test pattern
    ADS93XXV_HAL_writeRegister(0x02, 0x4);
    ADS93XXV_HAL_writeRegister(0x2E,0x00);// Enable constant test pattern
    ADS93XXV_HAL_writeRegister(0x02, 0x1);
}

void ADS93XXV_setupTestPattern(){


    ADS93XXV_HAL_writeRegister(0x02, 0x01);
    ADS93XXV_HAL_writeRegister(0x15, 0x0F);
    // select bank
    ADS93XXV_HAL_writeRegister(0x02, 0x2); // Bank1 - channel 1 - channel 8
    ADS93XXV_HAL_writeRegister(0x2F,0x1818);// test pattern
    ADS93XXV_HAL_writeRegister(0x30,0x2727);// test pattern
    ADS93XXV_HAL_writeRegister(0x31,0x3636);// test pattern
    ADS93XXV_HAL_writeRegister(0x32,0x4545);// test pattern
    ADS93XXV_HAL_writeRegister(0x33,0x5454);// test pattern
    ADS93XXV_HAL_writeRegister(0x34,0x6363);// test pattern
    ADS93XXV_HAL_writeRegister(0x35,0x7272);// test pattern
    ADS93XXV_HAL_writeRegister(0x36,0x8181);// test pattern
    ADS93XXV_HAL_writeRegister(0x2E,0x05);// enable const test patt


    // select bank -2
    ADS93XXV_HAL_writeRegister(0x02, 0x4); // Bank1 - channel 1 - channel 8
    ADS93XXV_HAL_writeRegister(0x2F,0x1818);// test pattern
    ADS93XXV_HAL_writeRegister(0x30,0x2727);// test pattern
    ADS93XXV_HAL_writeRegister(0x31,0x3636);// test pattern
    ADS93XXV_HAL_writeRegister(0x32,0x4545);// test pattern
    ADS93XXV_HAL_writeRegister(0x33,0x5454);// test pattern
    ADS93XXV_HAL_writeRegister(0x34,0x6363);// test pattern
    ADS93XXV_HAL_writeRegister(0x35,0x7272);// test pattern
    ADS93XXV_HAL_writeRegister(0x36,0x8181);// test pattern

    ADS93XXV_HAL_writeRegister(0x2E,0x05);// enable constant test pattern

    ADS93XXV_HAL_writeRegister(0x02, 0x1);

}

