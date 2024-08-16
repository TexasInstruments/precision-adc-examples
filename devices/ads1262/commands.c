/*
 * commands.c
 *
 *  Created on: Aug 18, 2018
 *      Author: a0282860
 */



#include "commands.h"

// Select function to use for printing to IO
#define PRINT USBprintf

//****************************************************************************
//
// Console definitions and variables
//
//****************************************************************************

const char *adcRegisterNames[NUM_REGISTERS] = {"ID", "POWER", "INTERFACE", "MODE0", "MODE1", "MODE2", "INPMUX",
                                                "OFCAL0", "OFCAL1", "OFCAL2", "FSCAL0", "FSCAL1", "FSCAL2",
                                                "IDACMUX", "IDACMAG", "REFMUX", "TDACP", "TDACN", "GPIOCON", "GPIODIR", "GPIODAT",
                                                "ADC2CFG", "ADC2MUX", "ADC2OFC0", "ADC2OFC1", "ADC2FSC0", "ADC2FSC1"};



//****************************************************************************
//
// Function prototypes for command line functions. The function definitions
// are at the bottom of this file. Note, these functions are not static
// since they are called from cmdline.c (TODO: determine if they can be static)
//
//****************************************************************************
int Cmd_help(int argc, char *argv[]);
int Cmd_clc(int argc, char *argv[]);
int Cmd_spi(int argc, char *argv[]);
int Cmd_rreg(int argc, char *argv[]);
int Cmd_wreg(int argc, char *argv[]);
int Cmd_regmap(int argc, char *argv[]);
int Cmd_setpin(int argc, char *argv[]);
int Cmd_start1(int argc, char *argv[]);
int Cmd_stop1(int argc, char *argv[]);
int Cmd_rdata(int argc, char *argv[]);
int Cmd_noise(int argc, char *argv[]);
int Cmd_sfocal1(int argc, char *argv[]);
int Cmd_syocal1(int argc, char *argv[]);

int Cmd_collect(int argc, char *argv[]);
int Cmd_testconfig(int argc, char *argv[]);
int Cmd_runtest(int argc, char *argv[]);
//int Cmd_status(int argc, char *argv[]);
int Cmd_runtest2(int argc, char *argv[]);


//****************************************************************************
//
// This is the table that holds the command names, implementing functions, and
// brief description.
//
//****************************************************************************
tCmdLineEntry g_psCmdTable[] =
{
    { "clc",        Cmd_clc,     "     : clear the screen" },
    { "help",       Cmd_help,     "    : display a list of commands" },
    { "rreg",       Cmd_rreg,     "    : reads a single register given by argument. Example: rreg 4"},
    { "wreg",       Cmd_wreg,     "    : writes to a single register given by the arguments. Example: wreg 4 cc"},
    { "rdata",      Cmd_rdata,     "   : Read data"},
    { "regmap",     Cmd_regmap,     "  : read all registers and print out the current register configuration"},
    { "spi",        Cmd_spi,     "     : sends a sequence of SPI commands, provided as arguments"},
    { "set",        Cmd_setpin,  "     : Set GPIO pin state. Example: set start high"},
    { "start1",     Cmd_start1,     "  : Start ADC1 conversions"},
    { "stop1",      Cmd_stop1,     "   : Stop ADC1 conversions"},



    { "noise",      Cmd_noise,     "   : Measure noise"},
    { "sfocal1",    Cmd_sfocal1,     " : Self-offset calibration"},
    { "syocal1",    Cmd_syocal1,     " : Self-offset calibration"},
    { "collect",    Cmd_collect,     " : Collect data"},


    { "testcfg",       Cmd_testconfig,     " : Configure test case"},
    { "runtest",       Cmd_runtest,     " : Run test case"},
    { "runtest2",       Cmd_runtest2,     " : Run test case"},



    //{ "status",     Cmd_status,     " : shows USB bulk connection status"},

    { 0, 0, 0 } // Indicates the end of lookup table (do not delete!!)
};


//****************************************************************************
//
// This function prints the TI logo, application name, versions and (c) to
// the terminal / console.
//
//****************************************************************************
void PrintLogo(void)
{
    PRINT("\n\033[2J\033[31m\r", 11);
    PRINT("----------------------------------\r\n", 35);
    PRINT("     _\r\n", 7);
    PRINT("   _| |_  Texas Instruments\r\n", 28);
    PRINT("   \\.  _} Console Application\r\n", 30);
    PRINT("     \\(   v1.0.0, (C) 2018\r\n\n", 28);
    PRINT("----------------------------------\n", 36);
    PRINT("\033[0m\r", 5);
}

float statistics_stddev(const int32_t data[], uint32_t array_length)
{
    // Consider a different algorithm
    // https://stackoverflow.com/questions/1174984/how-to-efficiently-calculate-a-running-standard-deviation
    // https://provideyourown.com/2012/statistics-on-the-arduino/

    float sum = 0.0;
    float mean = 0.0;
    float variance = 0.0;
    float standardDeviation = 0.0;

    // Get array length
    //uint32_t array_length = (sizeof(data)/sizeof(*data));

    // Calculate mean
    uint_fast8_t i;
    for (i = 0; i < array_length; i++)
    {
        // TODO: Check for overflow...
        sum += data[i];
        //mean += sum / array_length;
    }
    mean = sum / array_length;

    // Calculate variance
    for (i = 0; i < array_length; i++)
    {
        variance += pow(data[i] - mean, 2);
    }
    variance = variance / array_length;

    // Calculate standard deviation
    standardDeviation = sqrt(variance);

    return standardDeviation;
}


#if 0

// Maximum number of characters allowed in command string, used for memory initialization
#define MAX_CONSOLE_STRING_LENGTH    (16)       // NOTE: We run into errors if we try to send more than 16 characters at a time since the UART FIFO runs out of memory

// Command string character array (holds the string since the last occurrence of '\r')
static char IOString[MAX_CONSOLE_STRING_LENGTH] = "";

// Index to keep track of how many characters have been placed into the command string
static int32_t IOindex = 0;


//****************************************************************************
//
// This function reads UART receive FIFO characters one at a time and stores
// them into a character array. Once a '\r' is found, this function calls the
// 'cmdline' module to parse the command table (above) for the corresponding
// function to run. This function also provides command line validation.
//
// This function is similar to UARTgets(), but provides some extra validation.
//
//****************************************************************************
void IOReadCharacters(void)
{

    // TODO: Make this function work for any interface, not just UART


    //
    // Loop while there are characters in the UART receive FIFO.
    //
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
       //
       // Read the next character from the receive FIFO
       //
       int32_t character = MAP_UARTCharGetNonBlocking(UART0_BASE);      // TODO: Try to read more than one character at a time,see UARTgets() in uartstdio.c

       //
       // Ignore CSI sequences (i.e. cursor move, etc.)
       //
       if(character == 27)
       {
           //
           // CSI sequences begin with an escape character (27 in decimal),
           // are followed by '[', then any number of characters between 0x20 and 0x3F,
           // and finally end with a single '@A–Z[\]^_`a–z{|}~' character.
           //
           MAP_UARTCharGetNonBlocking(UART0_BASE);     // '['

           while(   (MAP_UARTCharGetNonBlocking(UART0_BASE) >= 0x20) &&     // 0x20-0x3F
                    (MAP_UARTCharGetNonBlocking(UART0_BASE) <= 0x3F));

           MAP_UARTCharGetNonBlocking(UART0_BASE);     // '@A–Z[\]^_`a–z{|}~'

           // For debugging, print something else out in its place
           //PRINT("\a", 1);

           //
           // Read the next character (if any)
           //
           break;
       }

       //
       // Handle 'backspace' character (0x08)
       //
       if(character == 8)
       {
           //
           // Check if we have any characters to delete
           //
           if (IOindex > 0)
           {
               //
               // Send CSI sequence to terminal to move the cursor left,
               // and clear the character(s) to the right of the cursor
               //
               PRINT("\033[1D\033[0K", 8);

               //
               // Remove the character from the command string
               //
               IOindex--;
               IOString[IOindex] = '\0';
           }

           //
           // Read the next character (if any)
           //
           break;
       }

       //
       // Ignore characters that are not 'A-Z', 'a-z', '0-9', '\r' (enter), or ' ' (space)
       // Reference: http://www.asciitable.com/
       //
       if( ((character < '0') ||
           ((character > '9') && (character < 'A')) ||
           ((character > 'Z') && (character < 'a')) ||
           (character > 'z')) &&
           (character != '\r') &&
           (character != ' '))
       {
           //
           // Read the next character (if any)
           //
           break;
       }

       //
       // Process the received character...
       //
       if(IOindex > MAX_CONSOLE_STRING_LENGTH - 1)
       {
           //
           // We don't have space in the command string for this character,
           // (and we also need to reserve a spot for the '\0' character at the end)
           //
           PRINT("\n\033[31m\rERROR: Command many not exceed %d characters.\033[0m\n", MAX_CONSOLE_STRING_LENGTH - 1);

           //
           // In case the command was entered from an input field or text box,
           // read all remaining characters and ignore them. In case data
           // overflowed the command string more than once, we read the FIFO
           // over and over again until we return only '0xFF' results
           // (i.e. the host TX is no longer active).
           //
           // NOTE: MAP_UARTCharGetNonBlocking() returns a 32-bit number
           // hence we compare the result to '0xFFFFFFFF'.
           //
           while(MAP_UARTCharGetNonBlocking(UART0_BASE) != 0xFFFFFFFF);

           //
           // Clear the command string and reset the string index
           //
           memset(IOString, '\0', MAX_CONSOLE_STRING_LENGTH);
           IOindex = 0;

           //
           // Print command prompt
           //
           PRINT("\n>> ", 4);
       }
       else if(character != '\r')
       {
           //
           // Echo character back to the UART (unless it is a carriage return),
           // add it into the command string, and increment the string counter
           //
           MAP_UARTCharPutNonBlocking(UART0_BASE, character);       //TODO: Make hit blocking?
           IOString[IOindex] = character;
           IOindex++;

       }
       else if (IOindex == 0)
       {
           //
           // String is empty, just print the command prompt
           //
           PRINT("\n>> ", 4);
       }
       else
       {
           //
           // Character is '\r' and command string is not empty;
           // therefore, process the command...
           //

           // TODO: Print the time stamp of when the command was processed

           //
           // Process the command from a the lookup table.
           // NOTE: this function removes any leading or trailing spaces
           //
           int result = CmdLineProcess(IOString);

           //
           // Handle command errors (if any)
           //
           switch(result)
           {
               case CMDLINE_BAD_CMD:
                   PRINT("\n'%s' is not a recognized command. Type 'help' to see a list.\n", IOString);
                   break;

               case CMDLINE_TOO_MANY_ARGS: //TODO: create missing cases
                  break;

               case CMDLINE_TOO_FEW_ARGS:
                  break;

               case CMDLINE_INVALID_ARG:
                  break;

               default:     // 0 = success
                   break;
           }

           //
           // Clear the command string and reset the string index
           //
           memset(IOString, '\0', MAX_CONSOLE_STRING_LENGTH);
           IOindex = 0;

           //
           // Print command prompt
           //
           PRINT("\r\n>> ", 4);
       }
   }
}

#endif


//****************************************************************************
//
// This function implements the "help" command.  It prints a simple list of
// the available commands with a brief description.
//
//****************************************************************************
int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    PrintLogo();


    //
    // Print some header text.
    //
    PRINT("\r\nAvailable commands\r\n");
    PRINT("------------------\r\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        PRINT(pEntry->pcCmd);
        PRINT(pEntry->pcHelp);
        PRINT("\r\n");

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
// This function implements the "clc" command.  It clears the terminal.
//
//****************************************************************************
int Cmd_clc(int argc, char *argv[])
{
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


//****************************************************************************
//
// This function implements the "spi" command.  It sends SPI commands.
//
//****************************************************************************
int Cmd_spi(int argc, char *argv[])
{
    //uint8_t sendData[NUM_OF_SSI_DATA]  = {0x20, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
    //uint32_t getData[NUM_OF_SSI_DATA]  = {0x00};

    PRINT("\nargc = %d\r\n", argc);
    int i;
    for (i = 0; i < argc; i++)
    {
        PRINT("argv[%d] = %s\r\n", i, argv[i]);
    }

    //
    // Fail the command if the "echo" command is not terminated with a space.
    //
    //if(pucStr[4] != ' ')
    //{
    //    return(-1);
    //}



    // Set nCS low
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, 0);

    /* Send the data from the SSI3 Master */
    //for(ii = 0; ii < NUM_OF_SSI_DATA; ii++)
    //{
    //    MAP_SSIDataPut(SSI3_BASE, sendData[ii]);
    //    MAP_SSIDataGet(SSI3_BASE, &getData[ii]);
    //}

    //MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);


    // Print data to console
    //UARTprintf("ID:     0x%02X\n", getData[2]);
    //UARTprintf("POWER:  0x%02X\n", getData[3]);
    //UARTprintf("INTER:  0x%02X\n", getData[4]);
    //UARTprintf("MODE0:  0x%02X\n", getData[5]);
    //UARTprintf("MODE1:  0x%02X\n", getData[6]);
    //UARTprintf("MODE2:  0x%02X\n", getData[7]);
    //UARTprintf("INPMUX: 0x%02X\n\n", getData[8]);

    //
    // Return success.
    //
    return(0);
}


//****************************************************************************
//
// This function implements the "rreg" command.
//
//****************************************************************************
int Cmd_rreg(int argc, char *argv[])
{

    char *end;
    uint8_t addr = strtol(argv[1], &end, 16);

    uint8_t value;
    value = readSingleRegister(addr);

    PRINT("%10s:\t0x%02X\r\n", adcRegisterNames[addr], value);

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
int Cmd_wreg(int argc, char *argv[])
{

    char *end;
    uint8_t addr = strtol(argv[1], &end, 16);
    uint8_t val = strtol(argv[2], &end, 16);

    writeSingleRegister(addr, val);

    PRINT("Wrote 0x%02X to %s\r\n", val, adcRegisterNames[addr]);

    //
    // Return success.
    //
    return(0);
}

int Cmd_rdata(int argc, char *argv[])
{
    // Ignore command arguments

    waitForDRDYinterrupt(100);

    //bool CRC_BYTE_ENABLED = true;
    //bool STATUS_BYTE_ENABLED = true;

    uint8_t statusByte, checksumByte;

    uint32_t data = readData(&statusByte, NULL, &checksumByte);
    PRINT("Status: 0x%02X\r\n", statusByte);
    PRINT("Data: 0x%08X\r\n", data);
    PRINT("Checksum: 0x%02X\r\n", checksumByte);


    //
    // Return success.
    //
    return(0);
}




int Cmd_noise(int argc, char *argv[])
{
    // Ignore command arguments


#define SAMPLES (100)

    uint32_t data[SAMPLES];

    int i;
    for (i = 0; i < SAMPLES; i++)
    {
        waitForDRDYinterrupt(100);
        data[i] = readData(NULL, NULL, NULL);
    }

    float stDev = statistics_stddev(&data[0], SAMPLES);
    PRINT("CodesRMS: %d\r\n", (uint32_t) stDev);


    //
    // Return success.
    //
    return(0);
}



int Cmd_collect(int argc, char *argv[])
{

    char *end;
    uint32_t num_samples = strtol(argv[1], &end, 10);
    uint8_t statusByte, checksumByte;

    int i;
    for (i = 0; i < num_samples; i++)
    {
        waitForDRDYinterrupt(100);
        uint32_t data = readData(&statusByte, NULL, &checksumByte);

        PRINT("Status: 0x%02X\tData: 0x%08X\tChecksum: 0x%02X\r\n", statusByte, data, checksumByte);
    }

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
int Cmd_regmap(int argc, char *argv[])
{
    // Ignore command arguments

    uint8_t registerValues[NUM_REGISTERS] = { 0 };

    /* Send the data from the SSI3 Master */
    int i;
    for(i = 0; i < NUM_REGISTERS; i++)
    {
        registerValues[i] = readSingleRegister(i);
    }

    for(i = 0; i < NUM_REGISTERS; i++)
    {
        PRINT("0x%02X %10s:\t0x%02X\r\n", i, adcRegisterNames[i], registerValues[i]);
    }

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
int Cmd_setpin(int argc, char *argv[])
{
    if (argc < 3) { return CMDLINE_TOO_FEW_ARGS; }
    else if (argc > 3) { return CMDLINE_TOO_MANY_ARGS; }

    // TODO: change strings to upper or lower case to avoid errors

    bool b_arg;

    if ( (strcmp(argv[2], "high") == 0) || (strcmp(argv[2], "1") == 0) ) {
        b_arg = true;
    } else if ( (strcmp(argv[2], "low") == 0) || (strcmp(argv[2], "0") == 0) ) {
        b_arg = false;
    } else {
        return CMDLINE_INVALID_ARG;
    }

    if ( strcmp(argv[1], "start") == 0 ) {
        setSTART(b_arg);
    } else if ( strcmp(argv[1], "pwdn") == 0 ) {
        setPWDN(b_arg);
    }


    //
    // Return success.
    //
    return(0);
}


int Cmd_start1(int argc, char *argv[])
{
    // Ignore command arguments
    sendCommand(OPCODE_START1);

    PRINT("Sent START1 (0x%02X) command\r\n", OPCODE_START1);

    //
    // Return success.
    //
    return(0);
}

int Cmd_stop1(int argc, char *argv[])
{
    // Ignore command arguments
    sendCommand(OPCODE_STOP1);

    PRINT("Sent STOP1 (0x%02X) command\r\n", OPCODE_STOP1);

    //
    // Return success.
    //
    return(0);
}

int Cmd_sfocal1(int argc, char *argv[])
{
    // Ignore command arguments
    sendCommand(OPCODE_SFOCAL1);
    waitForDRDYinterrupt(100);

    PRINT("SFOCAL1 (0x%02X) calibration complete\r\n", OPCODE_SFOCAL1);

    //
    // Return success.
    //
    return(0);
}

int Cmd_syocal1(int argc, char *argv[])
{
    // Ignore command arguments
    sendCommand(OPCODE_SYOCAL1);
    waitForDRDYinterrupt(100);

    PRINT("SYOCAL1 (0x%02X) calibration complete\r\n", OPCODE_SYOCAL1);

    //
    // Return success.
    //
    return(0);
}



int Cmd_testconfig(int argc, char *argv[])
{
    // Ignore command arguments

    writeSingleRegister(0x01, 0x00);            // Turn off internal reference and clear reset flag
    writeSingleRegister(0x03, 0x10);            // Turn on input chop
    writeSingleRegister(0x04, 0x00);            // SINC1 filter
    writeSingleRegister(0x05, 0x59);            // PGA = 32V/V, 1200 SPS
    writeSingleRegister(0x06, 0x12);            // Input MUX: AIN1/AIN2
    writeSingleRegister(0x0F, 0x0A);            // REF MUX: AIN0 / AIN3
    writeSingleRegister(0x10, 0x19);            // MAGP = 0.5V (disabled)
    writeSingleRegister(0x11, 0x19);            // MAGN = 0.5V (disabled)
    writeSingleRegister(0x12, 0xE0);            // GPIO connected to AINCOM/9/8 (and enabled low, by default)
    writeSingleRegister(0x15, 0x5A);            // ADC2: 100SPS, AIN4/5 ext ref, 4V/V
    writeSingleRegister(0x16, 0x67);            // ADC2 Input MUX: AIN6/AIN7

    PRINT("Registers programmed\r\n");

    //
    // Return success.
    //
    return(0);
}

int Cmd_runtest(int argc, char *argv[])
{
#define NUM_CYCLES    (5)


    int i;
    for (i = 0; i < NUM_CYCLES; i++)
    {
        sendCommand(OPCODE_STOP1);
        sendCommand(OPCODE_START1);
        Cmd_noise(1, NULL);
    }

    //
    // Return success.
    //
    return(0);
}


int Cmd_runtest2(int argc, char *argv[])
{
#define NUM_CYCLES    (5)

    //char *string[]; = "collect 512";
    //*string

    int i;
    for (i = 0; i < NUM_CYCLES; i++)
    {
        sendCommand(OPCODE_STOP1);
        sendCommand(OPCODE_START1);
        //Cmd_collect(2, string);
    }

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
//int Cmd_status(int argc, char *argv[])
//{
//    //
//    // Print status
//    //
//    if (g_bUSBConfigured)
//    {
//        PRINT("\nHost:\033[32m Connected\033[0m\n", 26);
//    }
//    else
//    {
//        PRINT("\nHost:\033[31m Disconnected\033[0m\n", 29);
//    }
//
//    //
//    // Return success.
//    //
//    return(0);
//}
