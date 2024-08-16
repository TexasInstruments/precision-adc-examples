/*
 * commands.h
 *
 *  Created on: Aug 18, 2018
 *      Author: a0282860
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_



/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* DriverLib Includes */
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "lib/ustdlib.h"
#include "lib/cmdline.h"
#include "lib/terminal.h"

#include "unit_tests.h"
#include "settings.h"


void IOReadCharacters(void);
void PrintLogo(void);

#endif /* COMMANDS_H_ */
