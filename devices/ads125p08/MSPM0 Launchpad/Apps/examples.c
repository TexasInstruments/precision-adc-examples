/*
 * Copyright (c) 2024-2026, Texas Instruments Incorporated
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

#include "Driver/ads125p08.h"
#include "Driver/hal.h"
#include "Driver/crc.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

REG_IO ADC_Register;                // struct to hold register values, STATUS, and CRC info. 
#define numSamples  64              // allocated number of samples for M0
ADC_IO ADC_readings[numSamples];    // struct to hold 16/24 bit conversion, STATUS, and CRC info.
STATUS_IO ADC_STATUS;               // struct to hold the STATUS_MSB and STATUS_LSB flags.
bool g_collectData = true;          // a global variable used to begin/end continuous data collection.

//**********************************************************************************
//  Simple ADC Example:  In this example the ADC is setup in a simple configuration, 
//  measuring an internal short. The sequencer is disabled and the ADC is 
//  converting continuously.  DRDYn will NEGEDGE upon every new reading. 
//  After initialization, this function will enter a while loop to collect numSamples
//  of ADC readings, then enter a low power mode and wait 10ms.  
//   
//  Adjustable parameters: numSamples, step0_settings, delay times, etc. 
//  SPI_CRC, STATUS Header, and REGMAP_CRC options are supported. 
//**********************************************************************************


void simpleADC(void)
{
    initCRC();                      // generate lookup tables for CRC functions. 
    resetDevice();                  // issue an SPI reset command and wait for reset. 
    initADC();                      // using the generated _RegisterDetails.h file, the 
                                    // register map configuration is loaded. 
    enableRegisterMapCrc(true);     // compute CRC solution for each memory page
                                    // and enable the CRC memory check function. 
    
        while (g_collectData)    {
            
            ADC_STATUS = checkStatus(); // check for any errors before beginning. 
            startAdcConversion();    // start conversion, read 8 samples and stop. 
            
            for(uint8_t i = 0; i<numSamples; i++) {
                ADC_readings[i] = readData();
            }
            
            stopAdcConversion();        // stop converting
            powerDownMode();            // enter a low power mode
            
            delay_ms(10);               
            
            activeMode();               // wakeup
        }
}

//**********************************************************************************
//  Sequencer Example:  In this example the ADC is setup as a channel scanner, using
//  an 8 step sequencer. Each of the 8 channels will sample a unique input and will
//  use a different combinations of OSR and NUM_CONVERSIONS values.  DRDYn will 
//  NEGEDGE upon every new reading. After the sequence is completed, the ADC will 
//  stop conversion and enter an idle state.  
// 
//  After initialization, this function will enter a while loop to collect all the 
//  samples in the sequence and wait 10ms before beginning another capture sequence.  
//   
//  Adjustable parameters: All stepX_settings, delay times, etc. 
//  SPI_CRC, STATUS Header, and REGMAP_CRC options are supported. 
//
//  NOTE: if adjusting the number of steps, or the NUM_CONVERSIONS per step, you
//  must adjust the numSeqSamples varaible.  
//
//**********************************************************************************



void sequencerExample(void)
{
    initCRC();
    resetDevice();          // issue an SPI reset command and wait for reset. 
    
    //General settings 
    writeSingleRegister(GENERAL_SETTINGS, SEQUENCER_CFG_ADDRESS, SEQ_MODE_SEQUENCERENABLEDEXECUTESEQUENCEONCE);
    writeSingleRegister(GENERAL_SETTINGS, SEQUENCE_STEP_EN_0_ADDRESS, 0xFF);    // enable steps 0-7
    writeSingleRegister(GENERAL_SETTINGS, GPIO_CFG_ADDRESS, GPIO1_CFG_PINOPERATESASDRDYNOUTPUT);  // set GPIO1 as DRDY output

    // step0 settings
    writeSingleRegister(STEP_0, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_0, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_0, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_1CONVERSION); 
    writeSingleRegister(STEP_0, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR128);	
    writeSingleRegister(STEP_0, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_INTERNALSHORT);

    // step1 settings
    writeSingleRegister(STEP_1, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_1, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_1, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_2CONVERSIONS); 
    writeSingleRegister(STEP_1, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR1024);	
    writeSingleRegister(STEP_1, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_CAPA);

    // step2 settings
    writeSingleRegister(STEP_2, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_2, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_2, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_4CONVERSIONS); 
    writeSingleRegister(STEP_2, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR4000);	
    writeSingleRegister(STEP_2, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_TEMPERATURESENSOR);

    // step3 settings
    writeSingleRegister(STEP_3, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_3, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_3, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_8CONVERSIONS); 
    writeSingleRegister(STEP_3, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR256);	
    writeSingleRegister(STEP_3, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_AVDD_AVSS_DIV_3);

    // step4 settings
    writeSingleRegister(STEP_4, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_4, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_4, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_16CONVERSIONS); 
    writeSingleRegister(STEP_4, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR256);	
    writeSingleRegister(STEP_4, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_CAPD);

    // step5 settings
    writeSingleRegister(STEP_5, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_5, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_5, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_16CONVERSIONS); 
    writeSingleRegister(STEP_5, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR1024);	
    writeSingleRegister(STEP_5, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_IOVDD_DIV_3);

    // step6 settings
    writeSingleRegister(STEP_6, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_AIN0);
    writeSingleRegister(STEP_6, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_AIN1);
    writeSingleRegister(STEP_6, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_1CONVERSION); 
    writeSingleRegister(STEP_6, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR1024);	
    writeSingleRegister(STEP_6, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_OFF);

    // step7 settings
    writeSingleRegister(STEP_7, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_AIN2);
    writeSingleRegister(STEP_7, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_AIN3);
    writeSingleRegister(STEP_7, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE | STEPX_NUM_CONV_1CONVERSION); 
    writeSingleRegister(STEP_7, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR128);	
    writeSingleRegister(STEP_7, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_OFF);

    uint16_t numSeqSamples = 37;       // this is the total number of conversion in the sequence.

    ADC_Register = readSingleRegister(GENERAL_SETTINGS, DEVICE_ID_ADDRESS);
     
    while (g_collectData)    {
        
        startAdcConversion();       
        
        for(uint8_t i = 0; i<numSeqSamples; i++) {
            ADC_readings[i] = readData();
        }
    delay_ms(200);
    }
}

//**********************************************************************************
//  FIFO Example:  In this example the ADC is setup in a simple configuration, 
//  measuring an internal short at an OSR of 16000.  The sequencer is disabled
//  and the ADC is converting continuously.  DRDYn will NEGEDGE upon every new
//  reading, and readings will be stored in the FIFO.  
//  
//  Adjustable parameters: FIFO_buffer_size, FIFOcollectionThreshold, step0_settings
//  SPI_CRC, STATUS Header, and REGMAP_CRC options are supported.   
//**********************************************************************************

void FIFOexample(void)
{
    uint16_t FIFOcollectionThreshold = 128; 
    int16_t FIFO_Depth;        // this value is read from the ADC and can be as large as 511; 
    uint16_t FIFO_buffer_size = 128;
    ADC_IO FIFO_Readings[FIFO_buffer_size]; 

    initCRC();
    resetDevice();          // issue an SPI reset command and wait for reset. 
   
    //General settings 
    writeSingleRegister(GENERAL_SETTINGS, SEQUENCER_CFG_ADDRESS, SEQ_MODE_SEQUENCERDISABLEDSTEPREPEATEDINDEFINITELY);
    writeSingleRegister(GENERAL_SETTINGS, GPIO_CFG_ADDRESS, GPIO1_CFG_PINOPERATESASDRDYNOUTPUT);  // set GPIO1 as DRDY output
    writeSingleRegister(GENERAL_SETTINGS, FIFO_CFG_ADDRESS, FIFO_EN_FIFOISENABLED);
    
 // step0 settings
    writeSingleRegister(STEP_0, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_OPEN);
    writeSingleRegister(STEP_0, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_OPEN);
    writeSingleRegister(STEP_0, STEPX_ADC_REF_CFG_ADDRESS, STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE ); 
    writeSingleRegister(STEP_0, STEPX_FLTR1_CFG_ADDRESS, STEPX_FLTR_OSR_SINCXOSR16000);	
    writeSingleRegister(STEP_0, STEPX_OW_SYSMON_CFG_ADDRESS, STEPX_SYS_MON_INTERNALSHORT);

    startAdcConversion(); 

    while ( g_collectData ) {
        
        do {
            delay_ms(200);    
            
            // poll the FIFO depth indicator until the collection threshold is exceeded. 
            ADC_Register = readSingleRegister(GENERAL_SETTINGS, FIFO_DEPTH_MSB_ADDRESS); 
            FIFO_Depth   = ((uint16_t) ADC_Register.Register_data  << 8);
            ADC_Register = readSingleRegister(GENERAL_SETTINGS, FIFO_DEPTH_LSB_ADDRESS); 
            FIFO_Depth = FIFO_Depth + ADC_Register.Register_data;
   
        } while (FIFO_Depth < FIFOcollectionThreshold);
     
        for (uint16_t FIFO_index = 0; FIFO_index < FIFO_Depth; FIFO_index++) { // read from the FIFO.  
            FIFO_Readings[FIFO_index] = readFIFO();      
        }    
           FIFO_Depth = 0;  // reset the depth        
    }
}