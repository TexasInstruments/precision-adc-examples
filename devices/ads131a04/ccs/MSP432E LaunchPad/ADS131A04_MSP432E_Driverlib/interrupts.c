/*
 * interrupts.c
 *
 *  Created on: Jan 17, 2019
 *      Author: a0282860
 */


#include "interrupts.h"


//****************************************************************************
//
// Global variables
//
//****************************************************************************
volatile bool       g_bUSBConfigured    = false;        // Configured or connected?
volatile uint32_t   g_ui32Flags         = 0;

uint32_t            g_num_samples       = 0;
