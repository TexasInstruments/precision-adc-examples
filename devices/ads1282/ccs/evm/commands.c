/**
 * \copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
#include "commands.h"

// Select function to use for printing to IO
#define PRINT json_console

// Define the number of commands that we want to hide from the USER/GUI,
// these should be defined at the beginning of the 'tCmdLineEntry' table!
#define NUMBER_HIDDEN_FUNCTIONS    (9)

// Data collection counter
//TODO: Add function documentation
//****************************************************************************
//
// Function prototypes for command line functions.
//
// All command function prototypes MUST have the format:
//
//      "int <function_name>(const int argc, const char *argv[])"
//
// ...so that that can be called by "CmdLineProcess()" in "cmdline.c".
// These functions cannot be 'static' since they are called from cmdline.c
//
//  "argc" is the number of arguments provided with the command string,
//      where the command string itself is counted as the first argument.
//
//  "*argv[]" is an array of char pointers which points to each string argument.
//      argv[0] is the command, and argv[1] is the first following argument.
//
//****************************************************************************

/* Hidden EVM Functions */
int Cmd_bsl(const int argc, const char *argv[]);
int Cmd_debug(const int argc, const char *argv[]);
int Cmd_delay(const int argc, const char *argv[]);
int Cmd_erase(const int argc, const char *argv[]);
int Cmd_unitTest(const int argc, const char *argv[]);
int Cmd_rprom(const int argc, const char *argv[]);
int Cmd_wprom(const int argc, const char *argv[]);
int Cmd_cprom(const int argc, const char *argv[]);
int Cmd_wpenprom(const int argc, const char *argv[]);

/* Standard EVM Functions */
int Cmd_id(const int argc, const char *argv[]);
int Cmd_help(const int argc, const char *argv[]);
int Cmd_rreg(const int argc, const char *argv[]);
int Cmd_rregs(const int argc, const char *argv[]);
int Cmd_wreg(const int argc, const char *argv[]);
int Cmd_regmap(const int argc, const char *argv[]);
int Cmd_collect(const int argc, const char *argv[]);
int Cmd_stop(const int argc, const char *argv[]);
int Cmd_null(const int argc, const char *argv[]);
int Cmd_rdata(const int argc, const char *argv[]);
int Cmd_spisend(const int argc, const char *argv[]);
int Cmd_reset(const int argc, const char *argv[]);
int Cmd_readAll(const int argc, const char *argv[]);
int Cmd_test(const int argc, const char *argv[]);

/* Helper functions */
bool    addressOutOfRange(const uint8_t address);
int8_t  argcExact(const uint8_t argc, const uint8_t expectedCount, const char *argv[]);
int8_t  argcAtLeast(const uint8_t argc, const uint8_t minCount, const char *argv[]);
int8_t  argcAtMost(const uint8_t argc, const uint8_t maxCount, const char *argv[]);
int8_t  argcLimit(const uint8_t argc);



//****************************************************************************
//
// This is the table that holds the command names, implementing functions, and
// brief description.
//
//****************************************************************************
tCmdLineEntry g_psCmdTable[] =
{
    /* HIDDEN FUNCTIONS - NOT DISPLAYED TO USER */
	{ "bsl",            Cmd_bsl,            "Re-enumerate to ROM DFU mode for firmware updates" },
    { "debug",          Cmd_debug,          "Displays firmware information for debugging"},
    { "delay",          Cmd_delay,          "Displays for a specified number of ms"},
    { "erase",          Cmd_erase,          "Erases PAMB FLASH memory"},
    { "unittest",       Cmd_unitTest,       "Runs unit tests"},
	{ "rprom",          Cmd_rprom,          "Reads from EEPROM"},
    { "wprom",          Cmd_wprom,          "Writes to EEPROM"},
    { "cprom",          Cmd_cprom,          "Clears the EEPROM" },
    { "wpenprom",       Cmd_wpenprom,       "Enables/disables EEPROM write protect"  },

    /* STANDARD FUNCTIONS - DO NOT REMOVE! */
    { "id",             Cmd_id,             "Displays EVM name and firmware version" },
    { "help",           Cmd_help,           "Displays a list of firmware commands" },
    { "rreg",           Cmd_rreg,           "Reads from a single register address" },
    { "rregs",          Cmd_rregs,          "Reads multiple registers" },
    { "wreg",           Cmd_wreg,           "Writes to a single register address" },
    { "regmap",         Cmd_regmap,         "Reads and displays all register data" },
    { "collect",        Cmd_collect,        "Starts data collection" },
    { "stop",           Cmd_stop,           "Aborts data collection or other operations" },
    { "null",           Cmd_null,           "Sends the NULL command and prints the response" },
    { "rdata",          Cmd_rdata,          "Starts and conversion and sends the results" },
    /* CUSTOM FUNCTIONS */
    //
    // ADD NEW COMMANDS HERE, USING THE FOLLOWING FORMAT:
    //   { <COMMAND STRING>, <FUNCTION POINTER>, <DESCRIPTION STRING> }
    //

    { "spisend",        Cmd_spisend,        "Sends a sequence of SPI command bytes" },
    { "reset",          Cmd_reset,          "Toggles nRESET pin" },
    { "read",           Cmd_readAll,        "For testing I2C communication" },
    { "test",           Cmd_test,           "For running custom tests" },


    /* END OF COMMAND TABLE - DO NOT REMOVE! */
    { 0, 0, 0 }
};



//****************************************************************************
//
// This function implements the "bsl" command.
//
//****************************************************************************
int Cmd_bsl(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Re-enumerate in ROM boot-loader mode
    JumpToBootLoader();

    return CMDLINE_EXIT;
}



//****************************************************************************
//
// This function implements the "debug" command.
//
//****************************************************************************
int Cmd_debug(const int argc, const char *argv[])
{
    // Ignore command arguments
// TODO: To use the DEBUG the correct interface must be selected
    // Print internal firmware flags
//    PRINT("--SPI Communication--");
//    PRINT("CRC IN enabled:\t %s", (SPI_CRC_ENABLED ? "TRUE" : "FALSE"));
//    PRINT("WLENGTH value :\t 0x%02X", WLENGTH);
//    PRINT("CRC_TYPE :\t %s", (SPI_CRC_TYPE ? "CCITT" : "ANSI"));

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "delay" command.
//
//****************************************************************************
int Cmd_delay(const int argc, const char *argv[])
{
    // Check argument count
    if (argc != 2) { return CMDLINE_BAD_CMD; }

    // Parse command argument
    char *end;
    uint32_t time_us = strtoul(argv[1], &end, 10);

    // Delay
    PRINT("Starting timer...");
    delay_us(time_us);
    PRINT("Timer expired!");

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "erase" command.
//
//****************************************************************************
int Cmd_erase(const int argc, const char *argv[])
{
    // Parse key from 2nd argument
	if (argc != 2) { return CMDLINE_BAD_CMD; }
    char *end;
    uint8_t key = strtoul(argv[1], &end, 16);

    // Validate key
    if (key != 0xBC)  { return CMDLINE_BAD_CMD; }

    // NOTE: Application will no longer run after erasing FLASH,
    // and a reset will be required to enter ROM boot loader mode!
    erase_flash();

    return CMDLINE_EXIT;
}



//****************************************************************************
//
// This function implements the "unittest" command.
//
//****************************************************************************
int Cmd_unitTest(const int argc, const char *argv[])
{
    // Ignore command arguments


    // TODO: Call run_self_tests()


    return CMDLINE_EXIT;
}
//****************************************************************************
//
// This function implements the "RPROM" command.
//
//****************************************************************************
int Cmd_rprom(const int argc, const char *argv[])
{
    //
    // Ignore command arguments
    //

    if (argc == 2 && (strcmp(argv[1], "pamb") == 0))
    {
        //
        // Read the data from the internal EEPROM and send the response.
        //
        readInternalEEPROM();
    }

    else
    {

        //
        // Read the data from I2C EEPROM and send the response.
        //
        readEEPROM();
    }

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "WPROM" command.
//
//****************************************************************************
int Cmd_wprom(const int argc, const char *argv[])
{
    //
    // Check for at least two arguments (including command)
    int8_t error = argcAtLeast(argc, 2, argv);
    if (error) { return error; }

    //
    // Parse the data to be written in EEPROM
    //
    char delim[] = " ";

    if (argc >= 2 && (strcmp(argv[1], "pamb") == 0))
    {
        char *data = strtok((char *)argv[2], delim);
        return writeInternalEEPROM(data, strlen(data));
    }
    else // write to EVM EEPROM
    {
        char *data = strtok((char *)argv[1], delim);
        return writeEEPROM(data, strlen(data));
    }
}

//****************************************************************************
//
// This function implements the "CPROM" command erasing the EEPROM on the EVM
//
//****************************************************************************
int Cmd_cprom(const int argc, const char *argv[])
{
    //
    // Ignore command arguments
    //

    //
    // Clears the I2C EEPROM and send the response.
    //
    clearEEPROM();

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function is used to enable or disable write protection for the EEPROM
// chip, present in the EVM board
//
//****************************************************************************
int Cmd_wpenprom(const int argc, const char *argv[])
{
    //
    // Check for three arguments (including command)
    //
    int8_t error = argcExact(argc, 2, argv);
    if (error) { return 0; }

    char enable;
    char *end;

    enable = strtoul(argv[1], &end, 16);

    writeProtectEnable(enable);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "id" command.
//
//****************************************************************************
int Cmd_id(const int argc, const char *argv[])
{
    // Check for one argument (command is counted as an argument)
    int8_t error = argcExact(argc, 1, argv);
    if (error) { return 0; }

    //
    // TODO: Check if Device is responsive before reading registers...
    //

    //
    // Read device ID
    //
    //  VER_MAJOR
    //  VER_MINOR
    //  VER_REVISION
    //  VER_BUILD

    // TODO: Verify what GC is expecting for the ID
    char firmware_name[] = "ADS1118";
    char firmware_version[] = "1.0.0.0";

    //
    // Print ID string
    //
    json_id(firmware_name, firmware_version, __DATE__, __TIME__);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "help" command.  It prints a simple list of
// the available commands with a brief description.
//
//****************************************************************************
int Cmd_help(const int argc, const char *argv[])
{

    //
    // Determine whether to show hidden commands
    //
    int startingIndex = NUMBER_HIDDEN_FUNCTIONS;
    if (argc == 2 && (strcmp(argv[1], "showHidden") == 0))
    {
        startingIndex = 0;
    }

    //
    // Point at the beginning of the command table.
    //
    tCmdLineEntry *pEntry;
    pEntry = &g_psCmdTable[startingIndex];    // Point at the starting command that we want to display to users

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        json_command(pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "RREG" command.
//
//****************************************************************************
int Cmd_rreg(const int argc, const char *argv[])
{
    // Check for two arguments (including command)
    int8_t error = argcExact(argc, 2, argv);
    if (error) { return 0; }

    // Parse register address
    char *end;
    uint8_t regAddr = strtoul(argv[1], &end, 16);

    // Check if address is out of range
    if (addressOutOfRange(regAddr)) { return(CMDLINE_INVALID_ARG); }

    // Read register command (command will send RREG + NULL)
    uint16_t value = readSingleRegister(regAddr);

    json_register(regAddr, value);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "RREGS" command.
//
//****************************************************************************
int Cmd_rregs(const int argc, const char *argv[])
{
    // Check for three arguments (including command)
    int8_t error = argcExact(argc, 3, argv);
    if (error) { return CMDLINE_EXIT; }

    // Parse register address
    char *end;
    uint8_t startAddr = strtoul(argv[1], &end, 16);
    uint8_t count = strtoul(argv[2], &end, 10);

    // Check if address is out of range
    if (addressOutOfRange(startAddr + count - 1)) { return(CMDLINE_INVALID_ARG); }

    // Read registers command
    readMultipleRegisters(startAddr, count);

    // Print results
    uint8_t i;
    for (i = startAddr; i < startAddr + count; i++)
    {
        json_register(i, getRegisterValue(i));
    }

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "WREG" command.
//
//****************************************************************************
int Cmd_wreg(const int argc, const char *argv[])
{
    // TODO: This must be adjusted for the device used
    // Check for three arguments (including command)
    int8_t error = argcExact(argc, 3, argv);
    if (error) { return CMDLINE_EXIT; }

    // Parse register address and value
    char *end;
    uint8_t regAddr = strtoul(argv[1], &end, 16);        // TODO: Consider parsing "0x#" or "#h" strings. Create a function to do so?
    uint16_t writeVal = strtoul(argv[2], &end, 16);

    //TODO: If writing to MODE register, make a note that we are enforcing certain settings!!

    // Check if address is out of range
    if (addressOutOfRange(regAddr)) { return(CMDLINE_INVALID_ARG); }

    // Write register command
    writeSingleRegister(regAddr, writeVal);

    // Send response
    json_console("Writing 0x%02X to register 0x%02X", writeVal, regAddr);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "RREGS" command.
//
//****************************************************************************
int Cmd_wregs(const int argc, const char *argv[])
{
    // Check for three arguments (including command)
    int8_t error = argcAtLeast(argc, 3, argv);
    if (error) { return CMDLINE_EXIT; }

    // Parse register address
    char *end;
    uint8_t startAddr = strtoul(argv[1], &end, 16);
    uint8_t count = strtoul(argv[2], &end, 10);
    error = argcExact(argc, 3 + count, argv);
    if (error) { return CMDLINE_EXIT; }

    // Check if address is out of range
    if (addressOutOfRange(startAddr + count - 1)) { return(CMDLINE_INVALID_ARG); }

    // TODO: If writing to MODE register, make a note that we are enforcing certain settings!!

    // Parse & print values
    uint8_t i, address, writeVal, data[NUM_REGISTERS];
    for (i = 0; i < count; i++)
    {
        address = startAddr + i;
        writeVal = data[address] = strtoul(argv[i + 3], &end, 16);
        json_console("Writing 0x%02X to register 0x%02X", writeVal, address);
    }

    // Write registers command
    writeMultipleRegisters(startAddr, count, &data[startAddr]);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "regmap" command.
// It prints the current register configurations.
//
//****************************************************************************
int Cmd_regmap(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Read registers command
    readMultipleRegisters(0x00, NUM_REGISTERS);

    // Print register names and values
    int i;
    for(i = 0; i < NUM_REGISTERS; i++)
    {
        json_register(i, getRegisterValue(i));
    }

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "collect" command.
//
//****************************************************************************
int Cmd_collect(const int argc, const char *argv[])
{
//    // Check for no more than two arguments (including command)
//    int8_t error = argcAtMost(argc, 2, argv);
//    if (error) { return 0; }
//
//    // 1 argument => collect block of data and stop
//    if (argc >= 2)
//    {
//        char *end;
//        g_num_samples = strtoul(argv[1], &end, 10);   // Number of samples should be an unsigned number
//    }
//    else
//    {
//        g_num_samples = 1;
//    }
////TODO: This could be set to a streaming mode for slower speed devices
//    /* Consideration of really how we want to collect and transmit data
//     * which could be considerably different between SAR and DS devices.
//     * Also need to be clear on format and what is sent as a frame ID
//     * or if we want to identify as JSON format strings.
//     *
//     * Collect should really just be a setup with the actual data collected
//     * in the main function based on DRDY flag for DS devices.
//     * This also will allow for a STOP command to reset the number of samples
//     * to collect to 0, which will stop collection.
//     */
//    // TODO: This must be adjusted for the device used
//    // No arguments => stream data (OR we could provide a separate command for this case)
//    if(g_num_samples >0)
//    {
//        // Set collect flag
//        g_ui32Flags |= COLLECT_MODE;
//    }
//    else
//    {
//        g_ui32Flags |= STREAM_MODE;
//    }
//
//
//
//    //Need to start conversion based on continuous conversion mode.....or if in SS mode need to restart the conversion
//    // Need register configuration to start conversion is in SS mode
//    uint16_t regConfig = startAdcConversion();
//    // Enable nDRDY Interrupt
//
//    enableDRDYinterrupt(true);
//
//    /* Set the nCS pin LOW */
//    setCS(LOW);
    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "stop" command.
//
//****************************************************************************
int Cmd_stop(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Clear collect flag
    g_ui32Flags &= ~COLLECT_MODE;

    json_evmState(IDLE);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function sends the "null" command.
//
//****************************************************************************
int Cmd_null(const int argc, const char *argv[])
{
//    // Ignore command arguments
//
//    // Send NULL command
//    uint8_t value = sendCommand(OPCODE_NULL);
//
//    // Print response
//    PRINT("NULL Response: 0x%02X", value);
//
    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function sends the "rdata" command.
//
//****************************************************************************
int Cmd_rdata(const int argc, const char *argv[])
{
    int8_t error = argcAtLeast(argc, 2, argv);
    if (error)
    {
        PRINT("Second argument required to select read mode: 0 = continuous, 1 = by command");
        return 0;
    }

    char *end;
    int mode = strtoul(argv[1], &end, 10);

    int32_t value;

    if (mode) { value = example_readDataByCommand(); }
    else { value = example_readDataContinuously(); }

    // Print response
    PRINT("Conversion Result: 0x%08X", value);

    return CMDLINE_EXIT;
}

//****************************************************************************
//
// This function implements the "spisend" command.  It sends a single SPI
// frame, as defined by the list of arguments.
//
//****************************************************************************
int Cmd_spisend(const int argc, const char *argv[])
{
    // Check for two arguments (including command)
    int8_t error = argcAtLeast(argc, 2, argv);
    if (error) { return 0; }

    // Initialize RX and TX buffers
    // CONSIDER: Increasing buffer size, parsing all user data, and sending data all at once
    uint8_t dataTx[1] = { 0 };
    uint8_t dataRx[1] = { 0 };

    char *end;
    uint_fast8_t i;
    for (i = 0; i < argc - 1; i++)
    {
        dataTx[0] = strtoul(argv[i + 1], &end, 16);
        spiSendReceive(dataTx, dataRx, 1);
        json_console("Sent: 0x%02X, Received: 0x%02X", dataTx[0], dataRx[0]);
    }

    return CMDLINE_EXIT;
}

int Cmd_reset(const int argc, const char *argv[])
{
    PRINT("Toggling nRESET...!");
    toggleRESET();
    return CMDLINE_EXIT;
}

int Cmd_readAll(const int argc, const char *argv[])
{
    uint8_t i, value;
    for (i = 0; i < PCA9535_NUM_REGISTERS; i++)
    {
        value = PCA9535_readRegister(i);
        PRINT("Register %d\t= 0x%02x", i, value);
    }

    return CMDLINE_EXIT;
}






int Cmd_test(const int argc, const char *argv[])
{
    toggleRESET();

	// TODO: Test waitForDRDY!

    return CMDLINE_EXIT;
}



#if 0
int Cmd_unitTest(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Print header to console
    PRINT("Running units tests...   \tPASS\tFAIL");
    PRINT("-------------------------\t----\t----");
    PRINT("Test nSYNC/nRESET pin... \t%s", test_SYNC_RESET_pin()      ? "PASS" : "\tFAIL");
    PRINT("Test nCS pin...          \t%s", test_CS_pin()              ? "PASS" : "\tFAIL");
    PRINT("Test nDRDY interrupt...  \t%s", test_DRDY_interrupt()      ? "PASS" : "\tFAIL");
    PRINT("Test RREG command...     \t%s", test_read_register()       ? "PASS" : "\tFAIL");
    PRINT("Test WREG command...     \t%s", test_write_register()      ? "PASS" : "\tFAIL");
    PRINT("Test RESET command...    \t%s", test_reset_command()       ? "PASS" : "\tFAIL");
    //PRINT("Test MUL commands...   \t%s", test_multiple_read_write() ? "PASS" : "\tFAIL");
    //PRINT("Test RDATA command...  \t%s", test_read_data()           ? "PASS" : "\tFAIL");
    PRINT("Test ADC noise...        \t%s", test_noise()               ? "PASS" : "\tFAIL");

    //
    // Return success.
    //
    return(0);
}
#endif



//****************************************************************************
//
// Helper functions
//
//****************************************************************************

bool addressOutOfRange(const uint8_t address)
{
    bool outOfRange = (address >= NUM_REGISTERS);

    // Check if address is out-of-range
    if (outOfRange)
    {
        // Print error message
        PRINT("Error: Address (0x%02X) is out-of-range! (Max: 0x%02X)", address, NUM_REGISTERS);
    }

    return (outOfRange);
}


//
// NOTE: TO THE USER, WE ARE NOT COUNTING THE COMMAND STRING AS PART OF THE ARGUMENT COUNT...
//

// TODO: Pass char array pointer to function to improve error message context

int8_t argcExact(const uint8_t argc, const uint8_t expectedCount, const char *argv[])
{
    if (argc == expectedCount)
    {
        return (0);
    }
    else
    {
        PRINT("Error: \'%s\' command requires %d argument(s), received %d", argv[0], expectedCount - 1, argc - 1);
        return (argc > expectedCount) ? CMDLINE_TOO_MANY_ARGS : CMDLINE_TOO_FEW_ARGS;
    }
}


int8_t argcAtLeast(const uint8_t argc, const uint8_t minCount, const char *argv[])
{
    if (argcLimit(argc)) { return CMDLINE_TOO_MANY_ARGS; }
    else if (argc >= minCount)
    {
        return (0);
    }
    else
    {
        PRINT("Error: \'%s\' command requires at least %d argument(s), received %d", argv[0], minCount - 1, argc - 1);
        return CMDLINE_TOO_FEW_ARGS;
    }
}


int8_t argcAtMost(const uint8_t argc, const uint8_t maxCount, const char *argv[])
{
    if (argcLimit(argc)) { return CMDLINE_TOO_MANY_ARGS; }
    else if (argc > maxCount)
    {
        PRINT("Error: \'%s\' command requires no more than %d argument(s), received %d", argv[0], maxCount - 1, argc - 1);
        return CMDLINE_TOO_FEW_ARGS;
    }
    else
    {
        return (0);
    }
}


int8_t argcLimit(const uint8_t argc)
{
    if (argc >= CMDLINE_MAX_ARGS - 1)
    {
        PRINT("Error: Exceeded firmware argument count limit (%d arguments)", CMDLINE_MAX_ARGS - 1);
        return CMDLINE_TOO_MANY_ARGS;
    }

    return 0;
}



#ifdef ENABLE_TERMINAL_PROCESSING

void PrintLogo(void);
int Cmd_clc(int argc, char *argv);

{ "clc",            Cmd_clc,        ": Clears the terminal screen" },


//****************************************************************************
//
// This function prints the TI logo, application name, versions and (c) to
// the terminal / console.
//
//****************************************************************************
void PrintLogo(void)
{
    // Clear Screen
    Cmd_clc(1,0);
// TODO: Fix if console application will be used
    PRINT("\033[31m\r");    // Changes foreground color (font) to bright red
    PRINT("----------------------------------\r\n");
    PRINT("     _\r\n");
    PRINT("   _| |_  Texas Instruments\r\n");
    PRINT("   \\.  _} ADS1115EVM Console Application\r\n");
    PRINT("     \\(   v1.0.0, (C) 2019\r\n\n");
    PRINT("----------------------------------\n");
    PRINT("\033[0m\r"); // Resets all attributes
}



//****************************************************************************
//
// This function implements the "clc" command.  It clears the terminal.
//
//****************************************************************************
int Cmd_clc(const int argc, const char *argv[])
{
    // Check for one argument (command is counted as an argument)
    int8_t error = argcExact(argc, 1, *argv);
    if (error) { return 0; }

    //
    // Print CSI sequence to clear the screen
    //
    PRINT("\033[2J", 4);

    //
    // Print CSI sequence to delete the scrollback buffer
    //
    PRINT("\033[3J", 4);

    return CMDLINE_EXIT;
}

#endif
