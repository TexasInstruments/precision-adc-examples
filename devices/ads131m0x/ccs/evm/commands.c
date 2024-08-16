#include "commands.h"

// Select function to use for printing to IO
#define PRINT json_console

// Define the number of commands that we want to hide from the USER/GUI,
// these should be defined at the beginning of the 'tCmdLineEntry' table!
#define NUMBER_HIDDEN_FUNCTIONS    (3)

// Data collection counter
//static uint32_t collectCounter; // TODO: Add getter and setter functions

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

// Hidden EVM Functions
int Cmd_debug(const int argc, const char *argv[]);
int Cmd_unitTest(const int argc, const char *argv[]);

// Standard EVM Functions
int Cmd_id(const int argc, const char *argv[]);
int Cmd_bsl(const int argc, const char *argv[]);
int Cmd_erase(const int argc, const char *argv[]);  // TODO: Delete
int Cmd_help(const int argc, const char *argv[]);
int Cmd_rreg(const int argc, const char *argv[]);
int Cmd_wreg(const int argc, const char *argv[]);
int Cmd_regmap(const int argc, const char *argv[]);
int Cmd_collect(const int argc, const char *argv[]);
int Cmd_spisend(const int argc, const char *argv[]);
int Cmd_stop(const int argc, const char *argv[]);

// Device Functions
int Cmd_crc(const int argc, const char *argv[]);
int Cmd_null(const int argc, const char *argv[]);
int Cmd_lock(const int argc, const char *argv[]);
int Cmd_unlock(const int argc, const char *argv[]);
int Cmd_reset(const int argc, const char *argv[]);
int Cmd_standby(const int argc, const char *argv[]);
int Cmd_sync(const int argc, const char *argv[]);
int Cmd_wakeup(const int argc, const char *argv[]);

// Helper functions
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
    { "debug",          Cmd_debug,          "Displays firmware information for debugging"},
    { "erase",          Cmd_erase,          "Erases FLASH memory" },
    { "unittest",       Cmd_unitTest,       "Runs unit tests"},

    /* STANDARD FUNCTIONS - DO NOT REMOVE! */
    { "id",             Cmd_id,             "Displays EVM name and firmware version" },
//  { "bsl",            Cmd_bsl,            "Call boot loader to update the firmware" },  // TODO: Re-implement with ROM boot loader
    { "help",           Cmd_help,           "Displays a list of firmware commands" },
    { "rreg",           Cmd_rreg,           "Reads from a single register address" },
    { "wreg",           Cmd_wreg,           "Writes to a single register address" },
    { "regmap",         Cmd_regmap,         "Reads and displays all register data" },
    { "collect",        Cmd_collect,        "Starts data collection" },
    { "spisend",        Cmd_spisend,        "Sends a sequence of SPI command bytes" },
    { "stop",           Cmd_stop,           "Aborts data collection or other operations" },

    /* CUSTOM FUNCTIONS */
    { "crc",            Cmd_crc,            "Computes CRC of arguments with current polynomial" },
    { "null",           Cmd_null,           "Sends the NULL command and prints the response" },
    { "lock",           Cmd_lock,           "Locks the registers, disabling the WREG command" },
    { "unlock",         Cmd_unlock,         "Unlocks the registers, enabling the WREG command" },
    { "reset",          Cmd_reset,          "Resets the ADC to default register settings" },
    { "standby",        Cmd_standby,        "Places the ADC in a low-power standby mode" },
    { "sync",           Cmd_sync,           "Performs synchronization by toggling the /SYNC pin" },
    { "wakeup",         Cmd_wakeup,         "Returns the ADC to normal conversion mode" },

    /// ADD NEW COMMANDS HERE, USING THE FOLLOWING FORMAT:
    // { <COMMAND STRING>, <FUNCTION POINTER>, <DESCRIPTION STRING> }

    /* END OF COMMAND TABLE - DO NOT REMOVE! */
    { 0, 0, 0 }
};



//****************************************************************************
//
// This function implements the "debug" command.
//
//****************************************************************************
int Cmd_debug(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Print internal firmware flags
    PRINT("--SPI Communication--");
    PRINT("CRC IN enabled:\t %s", (SPI_CRC_ENABLED ? "TRUE" : "FALSE"));
    PRINT("WLENGTH value :\t 0x%02X", WLENGTH);
    PRINT("CRC_TYPE :\t %s", (SPI_CRC_TYPE ? "CCITT" : "ANSI"));

    //
    // Return success.
    //
    return(0);
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



    //
    // Return success.
    //
    return(0);
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

    // TODO: Check if Device is responsive before reading registers...

    // Read device ID
    uint16_t value = readSingleRegister(ID_ADDRESS);
    char device_id[] = "ADS131M0x";
    device_id[8] = (char) (CHANCNT + '0');

    //
    // Print ID string
    //
    json_id(device_id, __DATE__, __TIME__);
    //PRINT("%s %s %s\r\n", device_id, __DATE__, __TIME__);

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "bsl" command.
//
//****************************************************************************
int Cmd_bsl(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Set flag for state machine to run boot loader
    g_ui32Flags |= BOOTLOADER_MODE;

    //
    // Return success. NOTE: Code should never reach this point.
    //
    return(0);
}




//****************************************************************************
//
// This function implements the "erase" command.
//
//****************************************************************************
int Cmd_erase(const int argc, const char *argv[])
{
    // Check for key
    if (argc != 2) { return CMDLINE_BAD_CMD; }

    // Parse key from 2nd argument
    char *end;
    uint8_t key = strtoul(argv[1], &end, 16);

    // Validate key
    if (key != 0xBC)  { return CMDLINE_BAD_CMD; }

    // Set flag for state machine to run boot loader
    g_ui32Flags |= ERASE_MODE;

    // Return success
    return(0);
}



//****************************************************************************
//
// This function implements the "help" command.  It prints a simple list of
// the available commands with a brief description.
//
//****************************************************************************
int Cmd_help(const int argc, const char *argv[])
{
    // Check for one argument (command is counted as an argument)
    int8_t error = argcExact(argc, 1, argv);
    if (error) { return 0; }

    // TODO: Consider adding a second argument, which when valid will show even the hidden commands


    //
    // Print some header text (CONSOLE ONLY).
    //
    //PrintLogo();
    //PRINT("\r\nAvailable commands\r\n");
    //PRINT("------------------\r\n");

    //
    // Point at the beginning of the command table.
    //
    tCmdLineEntry *pEntry;
    pEntry = &g_psCmdTable[NUMBER_HIDDEN_FUNCTIONS];    // Point at the starting command that we want to display to users

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        //PRINT("%10s", pEntry->pcCmd);
        //PRINT(pEntry->pcHelp);
        //PRINT("\r\n");
        json_command(pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
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

    json_regmap(regAddr, value);
    //PRINT("Read 0x%04X from %s register\r\n", value, adcRegisterNames[regAddr]);

    //
    // Return success.
    //
    return(0);
}


//****************************************************************************
//
// This function implements the "WREG" command.
//
//****************************************************************************
int Cmd_wreg(const int argc, const char *argv[])
{
    // Check for three arguments (including command)
    int8_t error = argcExact(argc, 3, argv);
    if (error) { return 0; }

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
    json_console("Writing 0x%04X to %s register", writeVal, adcRegisterNames[regAddr]);

    //
    // Return success.
    //
    return(0);
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

    int i;
    uint16_t registerValues[NUM_REGISTERS] = { 0 };

    // TODO: For ADS131M04, do NOT read registers 48-62, need to skip these here...

    // Read all registers
    for(i = 0; i < NUM_REGISTERS; i++)
    {
        registerValues[i] = readSingleRegister(i);
    }

    // Print register names and values
    for(i = 0; i < NUM_REGISTERS; i++)
    {
        json_regmap(i, registerValues[i]);
        // PRINT("0x%02X %12s :\t 0x%04X\r\n", i, adcRegisterNames[i], registerValues[i]);
    }

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "collect" command.
//
//****************************************************************************
int Cmd_collect(const int argc, const char *argv[])
{
    // Check for no more than two arguments (including command)
    int8_t error = argcAtMost(argc, 2, argv);
    if (error) { return 0; }

    // 1 argument => collect block of data and stop
    if (argc >= 2)
    {
        char *end;
        g_num_samples = strtoul(argv[1], &end, 10);   // Number of samples should be an unsigned number
    }
    else
    {
        g_num_samples = 1;
    }

#if 0
    // No arguments => stream data (OR we could provide a separate command for this case)
    else
    {
        g_ui32Flags |= STREAM_MODE;
    }

    // Send a NULL command to ensure that the next response is STATUS
    sendCommand(OPCODE_NULL);

    // Set collect flag
    g_ui32Flags |= COLLECT_MODE;


    // Enable nDRDY Interrupt
    enableDRDYinterrupt(true);

#endif

#if 0 // SLOWEST DATA READ IMPLEMENTATION

    // Read data
    adc_channel_data chData;
    uint32_t i;
    for(i = 0; i < num_samples; i++)
    {
        waitForDRDYinterrupt(100);
        readData(&chData);

        USBBufferWrite(BULK_TX_BUFFER, chData.response, strlen(json_console_start));

        PRINT("STATUS: 0x%04X", chData.response);
        PRINT("CH0: 0x%08X", chData.channel0);
#if (CHANNEL_COUNT > 1)
        PRINT("CH1: 0x%08X", chData.channel1);
#endif
#if (CHANNEL_COUNT > 2)
        PRINT("CH2: 0x%08X", chData.channel2);
#endif
#if (CHANNEL_COUNT > 3)
        PRINT("CH3: 0x%08X", chData.channel3);
#endif
#if (CHANNEL_COUNT > 4)
        PRINT("CH4: 0x%08X", chData.channel4);
#endif
#if (CHANNEL_COUNT > 5)
        PRINT("CH5: 0x%08X", chData.channel5);
#endif
#if (CHANNEL_COUNT > 6)
        PRINT("CH6: 0x%08X", chData.channel6);
 #endif
 #if (CHANNEL_COUNT > 7)
        PRINT("CH7: 0x%08X", chData.channel7);
#endif
        PRINT("CRC: 0x%04X", chData.crc);
    }





#endif


#if 0 // INLINE READ FUNCTION WITHOUT CRC CHECKING, TODO: Decide whether to print STATUS and CRC words...


    //uint16_t OCRvalue;
    //OCRvalue = (getRegisterValue(CLOCK_ADDRESS)>>2) & 0x0007;

    // Build Command
    uint8_t i, j;
    uint8_t DataTx[8]       = { 0 };
    uint8_t DataRx[4]       = { 0 };
    uint8_t bytesPerWord    = 3; // TODO: Update for different modes

    // TODO: Check OSR, if >= 256, then data reads need to get saved to memory
    if(OCRvalue<=1)
    {
        PRINT("Standby for data collection...");
        if(num_samples > 8192) num_samples = 8192;
//        uint32_t junk;
//            while (SSIDataGetNonBlocking(SSI3_BASE, &junk));        // TODO: can we remove this??

        for(j = 0; j < num_samples; j++)
        {

            waitForDRDYinterrupt(100);

            /* Set the nCS pin LOW */
            setCS(LOW);

            // [WORD1] Send NULL, receive response word
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            //status = combineDataBytes(&DataRx[0]);

            // [WORD2] Send 2nd word (we calculated this earlier), receive channel 1 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(DataTx[i + bytesPerWord]);
            }
            RXarray0[j] = combineDataBytes(&DataRx[0]);

            // [WORD3] Send 3rd word, receive channel 2 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            RXarray1[j] = combineDataBytes(&DataRx[0]);

            // [WORD4] Send 4th word, receive channel 3 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            RXarray2[j] = combineDataBytes(&DataRx[0]);

            // [WORD5] Send 5th word, receive channel 4 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            RXarray3[j] = combineDataBytes(&DataRx[0]);

            // [WORD6] Send 6th word, receive CRC out
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            //crc = combineDataBytes(&DataRx[0]);

            /* Set the nCS pin HIGH */
            setCS(HIGH);
        }

        for(i=0; i<num_samples;i++)
        {
            PRINT("CH1: 0x%06X\tCH2: 0x%06X\tCH3: 0x%06X\tCH4: 0x%06X\r\n", RXarray0[i], RXarray1[i], RXarray2[i], RXarray3[i]);
        }

    }
    else
    {

        #ifdef ENABLE_CRC_IN
        // Build CRC word (only if "RX_CRC_EN" register bit is enabled)
        uint16_t crcWord = calculateCRC(&DataTx[0], bytesPerWord * 2, 0xFFFF);
        DataTx[bytesPerWord]     = upperByte(crcWord);
        DataTx[bytesPerWord + 1] = lowerByte(crcWord);
        #endif

        // Read data
        //int32_t status;
        int32_t channelData[4];
        //int32_t crc;
        for(j = 0; j < num_samples; j++)
        {
            waitForDRDYinterrupt(100);

            /* Set the nCS pin LOW */
            setCS(LOW);

            // [WORD1] Send NULL, receive response word
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            //status = combineDataBytes(&DataRx[0]);

            // [WORD2] Send 2nd word (we calculated this earlier), receive channel 1 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(DataTx[i + bytesPerWord]);
            }
            channelData[0] = combineDataBytes(&DataRx[0]);

            // [WORD3] Send 3rd word, receive channel 2 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            channelData[1] = combineDataBytes(&DataRx[0]);

            // [WORD4] Send 4th word, receive channel 3 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            channelData[2] = combineDataBytes(&DataRx[0]);

            // [WORD5] Send 5th word, receive channel 4 data
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            channelData[3] = combineDataBytes(&DataRx[0]);

            // [WORD6] Send 6th word, receive CRC out
            for (i = 0; i < bytesPerWord; i++)
            {
                DataRx[i] = spiSendReceiveByte(0x00);
            }
            //crc = combineDataBytes(&DataRx[0]);

            /* Set the nCS pin HIGH */
            setCS(HIGH);

            PRINT("CH1: 0x%06X\tCH2: 0x%06X\tCH3: 0x%06X\tCH4: 0x%06X\r\n", channelData[0], channelData[1], channelData[2], channelData[3]);
        }
    }

#endif

    //
    // Return success.
    //
    return(0);
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

    int i;
    uint8_t din;
    uint8_t dout;
    char *end;

    //
    // SPI communication
    //
    setCS(LOW);

    for (i = 0; i < argc - 1; i++)
    {
        din = strtoul(argv[i + 1], &end, 16);
        dout = spiSendReceiveByte(din);

        json_console("Sent: 0x%02X, Received: 0x%02X", din, dout);
        //PRINT("Sent: 0x%02X, Received: 0x%02X", din, dout);
    }

    setCS(HIGH);


    //
    // Return success.
    //
    return(0);
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

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "crc" command.
//
//****************************************************************************
int Cmd_crc(const int argc, const char *argv[])
{
    // Check for three arguments (including command)
    int8_t error = argcAtLeast(argc, 2, argv);
    if (error) { return 0; }

    // Parse all arguments, up to 29 bytes
    uint8_t i, data[29];                            // TODO: Dynamically assign this memory!
    char *end;                                      // dummy pointer
    for (i = 0; i < (argc - 1); i++)
    {
        data[i] = strtoul(argv[i + 1], &end, 16);
    }

    // Calculate CRC
    uint16_t crc = calculateCRC(data, argc - 1, 0xFFFF);

    // Print response
    PRINT("%s CRC equals \"0x%04X\"", (SPI_CRC_TYPE ? "CCITT" : "ANSI"), crc);

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function sends the "null" command.
//
//****************************************************************************
int Cmd_null(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Send NULL command
    uint16_t value = sendCommand(OPCODE_NULL);

    // Print response
    PRINT("NULL Response: 0x%04X", value);

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "LOCK" command.
//
//****************************************************************************
int Cmd_lock(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Send LOCK command
    lockRegisters();
    PRINT("Locking the device");

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "UNLOCK" command.
//
//****************************************************************************
int Cmd_unlock(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Send UNLOCK command
    unlockRegisters();
    PRINT("Unlocking the device");

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function sends the "RESET" command.
//
//****************************************************************************
int Cmd_reset(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Send RESET command
    resetDevice();

    // Print acknowledgment
    PRINT("Resetting device...");

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "STANDBY" command.
//
//****************************************************************************
int Cmd_standby(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Send STANDBY command
    sendCommand(OPCODE_STANDBY);                // TODO: Do we need to set a flag to know if we are in this mode???
    PRINT("Entering standby mode");

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "status" command.  It prints the host state.
//
//****************************************************************************
int Cmd_sync(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Print response
    PRINT("Toggling the /SYNC pin");

    // Toggle nSYNC pin
    toggleSYNC();

    //
    // Return success.
    //
    return(0);
}



//****************************************************************************
//
// This function implements the "WAKEUP" command.
//
//****************************************************************************
int Cmd_wakeup(const int argc, const char *argv[])
{
    // Ignore command arguments

    // Send WAKEUP command
    sendCommand(OPCODE_WAKEUP);
    PRINT("Waking up the device");

    //
    // Return success.
    //
    return(0);
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
    else if (argc >= maxCount)
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




#if 0 //CH: Currently not using, and not sure if it is necessary
static void itoa(long unsigned int value, char* result, int base)
    {
      // check that the base if valid
      if (base < 2 || base > 36) { *result = '\0';}

      char* ptr = result, *ptr1 = result, tmp_char;
      int tmp_value;

      do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
      } while ( value );

      // Apply negative sign
      if (tmp_value < 0) *ptr++ = '-';
      *ptr-- = '\0';
      while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
      }

    }
#endif


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

    PRINT("\033[31m\r");    // Changes foreground color (font) to bright red
    PRINT("----------------------------------\r\n");
    PRINT("     _\r\n");
    PRINT("   _| |_  Texas Instruments\r\n");
    PRINT("   \\.  _} ADS131M04EVM Console Application\r\n");
    PRINT("     \\(   v1.0.0, (C) 2018\r\n\n");
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

    //
    // Return success.
    //
    return(0);
}

#endif
