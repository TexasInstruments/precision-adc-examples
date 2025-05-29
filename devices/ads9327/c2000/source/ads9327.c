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

#include <ads9327.h>

void ADS9327_unlockRegisterMap()
{
    // to unlock register map
    ADS9327_HAL_writeRegister(REGISTER_FEH_ADDRESS, REGISTER_FEH_REG_LOCK_UNLOCKKEY0);
    ADS9327_HAL_writeRegister(REGISTER_FEH_ADDRESS, REGISTER_FEH_REG_LOCK_UNLOCKKEY1);
}

void ADS9327_lockRegisterMap()
{
    // to lock resgiter map, write any value to addr 0xFE that is different than 0xB38F and 0xABCD 
    ADS9327_HAL_writeRegister(REGISTER_FEH_ADDRESS, REGISTER_FEH_REG_LOCK_LOCKKEY);
}

void ADS9327_regBankSel(uint8_t bank)
{
    // select desired register bank
    if (bank == 1)
    {
        //select bank 1
        ADS9327_HAL_writeRegister(REGISTER_02H_ADDRESS, REGISTER_02H_REG_BANK_SEL_REGBANK1);
    }

    else if (bank == 0) 
    {
        //select bank 0
        ADS9327_HAL_writeRegister(REGISTER_02H_ADDRESS, REGISTER_02H_REG_BANK_SEL_REGBANK0);    
    }

}

void ADS9327_resetAdcAndRegisters()
{
    ADS9327_unlockRegisterMap();
    
    //reset ADC and all ADC registers
    ADS9327_HAL_writeRegister(REGISTER_01H_ADDRESS, REGISTER_01H_RESET_RESET);

    ADS9327_lockRegisterMap();
}



// ADC initialization (must be done every time at powerup)
void ADS9327_initalization()
{
    ADS9327_HAL_writeRegister(REGISTER_FEH_ADDRESS, REGISTER_FEH_REG_LOCK_UNLOCKKEY0); //register map unlock sequence 1
    ADS9327_HAL_writeRegister(REGISTER_FEH_ADDRESS, REGISTER_FEH_REG_LOCK_UNLOCKKEY1); //register map unlock sequence 2
    ADS9327_HAL_writeRegister(REGISTER_02H_ADDRESS, REGISTER_02H_REG_BANK_SEL_REGBANK1); //select register bank 1
    ADS9327_HAL_writeRegister(REGISTER_0CH_ADDRESS,0x1001);
    ADS9327_HAL_writeRegister(REGISTER_FEH_ADDRESS, REGISTER_FEH_REG_LOCK_LOCKKEY); //lock register map
}
