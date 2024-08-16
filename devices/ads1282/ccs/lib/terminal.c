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

#include "terminal.h"
//TODO: Add function documenation
// Macro to direct print output
#define PRINT USBprintf


char g_pcCmdBuf[CMD_BUF_SIZE];

// Internal variables
uint32_t ui32CmdIdx;
char pcJunk;

#ifdef ENABLE_HISTORY_BUFFER
    #define HISTORY_SIZE    (3)
    uint8_t historyIndex = 0;
    char g_CmdHistoryBuf[HISTORY_SIZE][CMD_BUF_SIZE];
    static void updateHistoryBuffer(void);
    static void updateCommandBuffer(void);
#endif


//****************************************************************************
//
// Special characters
//
//****************************************************************************
#define BACKSPACE       ((char) 0x08)
#define DELETE          ((char) 0x7F)
#define RETURN          ((char) 0x0D)   // Carriage return (CR)
#define ESC             ((char) 0x1B)
#define NEWLINE         ((char) 0x0A)   // Line feed (LF)


static const char pcCRLF[2] = {RETURN, NEWLINE};

uint32_t RxBytesAvailable(void)
{
    return USBBufferDataAvailable(CDC_RX_BUFFER);
}

void ReadCdcRxBuffer(uint32_t numBytesToRead)
{
    assert(numBytesToRead < CMD_BUF_SIZE); //TODO: Return an error instead of asserting
    USBBufferRead(CDC_RX_BUFFER, (uint8_t *)&g_pcCmdBuf[0], numBytesToRead);
}

void delReturn(void)
{
    ui32CmdIdx = strlen(g_pcCmdBuf) - 1;
    char lastCharacter = g_pcCmdBuf[ui32CmdIdx];

    if (lastCharacter == RETURN)
    {
        // Replace line feed with null
        g_pcCmdBuf[ui32CmdIdx] = '\0';
        ui32CmdIdx++;

        // Send CR+LF
        USBBufferWrite(CDC_TX_BUFFER, (uint8_t *)&pcCRLF, 2);
    }
}

#ifdef ENABLE_TERMINAL_PROCESSING

// Internal function prototypes
static void controlSequenceHandler(void);

//****************************************************************************
//
// Character sequence sent to the serial terminal to implement a character
// erase when backspace is pressed.
//
//****************************************************************************
static const char pcBackspace[3] = {BACKSPACE, ' ', BACKSPACE};     // Alternate CSI implementation: "\033[1D\033[0K"

void TerminalProcess(void)
{
    // Read 1 byte
    uint32_t numBytesRead = USBBufferRead(CDC_RX_BUFFER, (uint8_t *)&g_pcCmdBuf[ui32CmdIdx], 1);

    if(numBytesRead)
    {
        char newCharacter = g_pcCmdBuf[ui32CmdIdx];

        //
        // Check for BACKSPACE or DELETE character
        // NOTE: PuTTY will send a DEL char when the "Backspace" key is entered.
        //
        if (newCharacter == BACKSPACE || newCharacter == DELETE)
        {
            g_pcCmdBuf[ui32CmdIdx] = '\0';      // Delete the backspace character from buffer

            if (ui32CmdIdx > 0)                 // Check if there is a previous character to delete
            {
                ui32CmdIdx--;                   // Decrement index
                g_pcCmdBuf[ui32CmdIdx] = '\0';  // Delete the previously sent character
                USBBufferWrite(CDC_TX_BUFFER,   // Echo backspace to terminal
                               (uint8_t *)pcBackspace, 3);
            }
        }

        //
        // Check for carriage return character
        //
        else if (newCharacter == RETURN)
        {
            g_pcCmdBuf[ui32CmdIdx] = '\0';      // Replace line feed with null
            ui32CmdIdx++;
            g_ui32Flags |= COMMAND_RECEIVED;    // Indicate that a command has been received.
            USBBufferWrite(CDC_TX_BUFFER,       // Send CR+LF
                           (uint8_t *)&pcCRLF, 2);
        }

        //
        // Check for ANSI escape sequence, TODO: Check if this has a different format on terminals other than PuTTY
        // REFERENCE: https://en.wikipedia.org/wiki/ANSI_escape_code
        //
        else if (newCharacter == ESC)
        {
            g_pcCmdBuf[ui32CmdIdx] = '\0';      // Replace ESC with null

            // Check for CSI escape sequence (NOTE: Other escape sequences are not handled!)
            USBBufferRead(CDC_RX_BUFFER, (uint8_t *)&pcJunk, 1);    // 0x40-0x5F, 1 time (for CSI byte = '[')
            if (pcJunk == '[') { controlSequenceHandler(); }
        }

        //
        // Ignore characters that are not 'A-Z', 'a-z', '0-9', or ' ' (space)
        //
        else if (!isalnum((int)newCharacter) && (newCharacter != ' '))
        {
            g_pcCmdBuf[ui32CmdIdx] = '\0';      // Remove new character from buffer

            // For debugging ONLY!
            //PRINT("Character ignored!");
            //PRINT("\r\n> %s", g_pcCmdBuf);
        }

        //
        // Check if we have space left in the command buffer
        //
        else if (ui32CmdIdx < CMD_BUF_SIZE - 1) // minus 1 to leave space for '\0' character
        {
            // Change all character to lower case
            g_pcCmdBuf[ui32CmdIdx] = (char)tolower((int)newCharacter);

            // Echo character & increment index
            USBBufferWrite(CDC_TX_BUFFER, (uint8_t *)&newCharacter, 1);
            ui32CmdIdx++;
        }

        //
        // Buffer overflow
        //
        else
        {
            clearCommandBuffer(false);
            PRINT("\r\nCommand exceeded maximum length (%d characters)\r\n", CMD_BUF_SIZE);
            PRINT("\r\n> %s", g_pcCmdBuf);
        }
    }
}

static void controlSequenceHandler(void)
{
    //USBBufferRead(CDC_RX_BUFFER, (uint8_t *)&pcJunk, 1);  // 0x30-0x3F, 0 or 1 times (parameter byte)
    //USBBufferRead(CDC_RX_BUFFER, (uint8_t *)&pcJunk, 1);  // 0x20-0x2F, any number of times (intermediate bytes)

    // Read until end of sequence: 0x40-0x7E, 1 time (final byte)
    do
    {
        USBBufferRead(CDC_RX_BUFFER, (uint8_t *)&pcJunk, 1);
    }
    while ((pcJunk < 0x40) || (pcJunk > 0x7E));


#ifdef ENABLE_HISTORY_BUFFER

    // We are currently assuming that no parameter or intermediate bytes were sent...
    switch (pcJunk)
    {
        // UP ARROW
        case 'A':
            if (historyIndex < HISTORY_SIZE)
            {
                historyIndex++;
                updateCommandBuffer();
            }
            break;

        // DOWN ARROW
        case 'B':
            if (historyIndex > 0)
            {
                historyIndex--;
                updateCommandBuffer();
            }
            break;
    }

#endif

    // For debugging ONLY!
    //PRINT("CSI sequence ignored!");
    //PRINT("\r\n> %s", g_pcCmdBuf);
}

#endif


void clearCommandBuffer(bool validCommand)
{

#ifdef ENABLE_HISTORY_BUFFER
    historyIndex = 0;
    if (validCommand) { updateHistoryBuffer(); }

#endif

    memset(&g_pcCmdBuf[0], 0, sizeof(g_pcCmdBuf));
    ui32CmdIdx = 0;
}


#ifdef ENABLE_HISTORY_BUFFER

static void updateHistoryBuffer(void)
{
    // This is a slow implementation but was easy to code...
    uint8_t i;

    for (i = HISTORY_SIZE - 1; i > 0; i--)
    {
        strncpy(g_CmdHistoryBuf[i], g_CmdHistoryBuf[i-1], CMD_BUF_SIZE);
        // TODO: Assign dynamic memory to store command string
        // TODO: Use "g_CmdHistoryBuf[]" to store pointers to memory locations
        // TODO: Create a FIFO module to implement "g_CmdHistoryBuf[]"
    }

    // TODO: Fig bug where command argument are not getting copied
    // TODO: If command is repeated, do not copy to history buffer
    strncpy(g_CmdHistoryBuf[0], g_pcCmdBuf, CMD_BUF_SIZE);

}

static void updateCommandBuffer(void)
{
    strncpy(g_pcCmdBuf, g_CmdHistoryBuf[historyIndex - 1], CMD_BUF_SIZE);
    ui32CmdIdx = strlen(g_pcCmdBuf);
    PRINT("\033[2K\r> %s", g_pcCmdBuf);
}

#endif



#ifdef UNIT_TESTING

#include <stdbool.h>

bool TerminalProcess_test(void)
{
    // Initial test conditions
    bool passed = true;
    memset(&g_pcCmdBuf[0], 0, sizeof(g_pcCmdBuf));
    strcpy(g_pcCmdBuf, "Command string");               // 14 characters (not including '\0' termination)
    ui32CmdIdx = 14;

    // Try adding symbols that are not allowed...
    g_pcCmdBuf[ui32CmdIdx] = '!';
    passed &= (TerminalProcess() == IGNORE_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 14));

    g_pcCmdBuf[ui32CmdIdx] = '{';
    passed &= (TerminalProcess() == IGNORE_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 14));

    g_pcCmdBuf[ui32CmdIdx] = '*';
    passed &= (TerminalProcess() == IGNORE_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 14));

    g_pcCmdBuf[ui32CmdIdx] = '~';
    passed &= (TerminalProcess() == IGNORE_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 14));


    // Try adding allowable characters...
    g_pcCmdBuf[ui32CmdIdx] = ' ';                       // 15
    passed &= (TerminalProcess() == ECHO_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 15));

    g_pcCmdBuf[ui32CmdIdx] = '1';                       // 16
    passed &= (TerminalProcess() == ECHO_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 16));

    g_pcCmdBuf[ui32CmdIdx] = 'A';                       // 17
    passed &= (TerminalProcess() == ECHO_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 17));

    g_pcCmdBuf[ui32CmdIdx] = 'z';                       // 18
    passed &= (TerminalProcess() == ECHO_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 18));


    // Try to overflow the buffer...
    memset(&g_pcCmdBuf[0], 0, sizeof(g_pcCmdBuf));
    strcpy(g_pcCmdBuf, "This string is getting too long");   // 31 characters
    ui32CmdIdx = 31;

    g_pcCmdBuf[ui32CmdIdx] = 'E';                            // 32
    passed &= (TerminalProcess() == BUFFER_OVERFLOW);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (g_pcCmdBuf[31] == '\0') && (ui32CmdIdx == 0));


    // Try to send backspace while buffer is empty...
    g_pcCmdBuf[ui32CmdIdx] = BACKSPACE;
    passed &= (TerminalProcess() == IGNORE_CHARACTER);
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 0));

    // Try to send backspace while buffer is not empty...
    strcpy(g_pcCmdBuf, "I made an");                        // 9 characters
    ui32CmdIdx = 9;
    g_pcCmdBuf[ui32CmdIdx] = BACKSPACE;
    passed &= (TerminalProcess() == BACKSPACE_CHARACTER);   // "I made a"
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 9) );

    // Try to send backspace while buffer is not empty...
    g_pcCmdBuf[ui32CmdIdx] = BACKSPACE;
    passed &= (TerminalProcess() == BACKSPACE_CHARACTER);   // "I made a"
    passed &= ( (g_pcCmdBuf[ui32CmdIdx] == '\0') && (ui32CmdIdx == 9));



/* Need to test...
#define DELETE_CHARACTER        ((int8_t) -2)       // Delete current position and right shift

#define ECHO_STRING             ((int8_t)  2)       // Echo entire string (when recalling a past command)
#define EXECUTE_COMMAND         ((int8_t)  3)       // Call command parsing function
#define LEFT_ARROW              ((int8_t)  4)       // Move cursor left
#define RIGHT_ARROW
*/


    return passed;
}

#endif  /* UNIT_TESTING */

#if 0
// TODO: Is this even necessary to include for PAMB?
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

    // TO DO: Make this function work for any interface, not just UART


    //
    // Loop while there are characters in the UART receive FIFO.
    //
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
       //
       // Read the next character from the receive FIFO
       //
       int32_t character = MAP_UARTCharGetNonBlocking(UART0_BASE);      // TO DO: Try to read more than one character at a time,see UARTgets() in uartstdio.c

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
           MAP_UARTCharPutNonBlocking(UART0_BASE, character);       //Make this blocking?
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

           // TO DO: Print the time stamp of when the command was processed

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

               case CMDLINE_TOO_MANY_ARGS: //To DO: create missing cases
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
