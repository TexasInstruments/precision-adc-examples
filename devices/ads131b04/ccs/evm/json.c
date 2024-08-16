#include "json.h"


// Select function to use for printing to IO
#define PRINT USBprintf

// Constants
const static char* true_string    = "true";
const static char* false_string   = "false";

// Internal function prototypes
static const char* bool2str(const bool logic);



//
// NOTE: Make sure each of the following functions aligns with a defined case
// in pambCustomCodec.js's "decode()" function!
//


// Sent in response to a received command, to acknowledge the command
void json_acknowledge(const char *commandBuffer)
{
    PRINT("{\"acknowledge\":\"%s\"}\r\n", commandBuffer);
}

// Prints a message to the GUI's console
void json_console(const char *pcString, ...)
{
    va_list vaArgP;

    //
    // Start the vargs processing.
    //
    va_start(vaArgP, pcString);

    const char *json_console_start = "{\"console\":\"";
    USBBufferWrite(CDC_TX_BUFFER, (uint8_t*)json_console_start, strlen(json_console_start));

    USBvprintf(pcString, vaArgP);

    const char *json_console_end = "\"}\r\n";
    USBBufferWrite(CDC_TX_BUFFER, (uint8_t*)json_console_end, strlen(json_console_end));

    //
    // We're finished with the vargs now.
    //
    va_end(vaArgP);
}

// Sends an error notification to the GUI
void json_error(const uint32_t error_code)
{
    PRINT("{\"error\":%d}\r\n", error_code);
}

// Updates the GUI of the current firmware state
void json_evmState(const evmState_t state)
{

    PRINT("{\"evm_state\":\"");
    switch(state)
    {
        case IDLE:
            PRINT("idle");
            break;

        case BUSY:
            PRINT("busy");
            break;

        case COLLECTING:
            PRINT("collecting");
            break;
    }
    PRINT("\"}\r\n");
}


void json_deviceModes(const bool standby, const bool locked)
{
    PRINT("{\"device_modes\":{\"standby\":%s,\"locked\":%s}}\r\n", bool2str(standby), bool2str(locked));
}

void json_id(const char *name, const char *date, const char *time)
{
    PRINT("{\"id\":{\"name\":\"%s\",\"date\":\"%s\",\"time\":\"%s\"}}\r\n", name, date, time);
}

//void json_pinLevels(const bool nSYNC_logic)
//{
//    PRINT("{\"pins\":{\"nSYNC\":%s}}\r\n", bool2str(nSYNC_logic));
//}

void json_preamble(void)
{
    PRINT("{\"preamble\":{"

                        // Sending an empty object will tell GUI to generate the preamble from the register map data

                            "}}\r\n");

                        // Here are some examples of how to add data to the preamble (if necessary)...

                                //"\"channel_count\":%d,"
                                //"\"MODE\":%d,"
                                //"\"status_en\":%s,"
                            //"}}\r\n", CHANCNT, getRegisterValue(MODE_ADDRESS), bool2str(true));
}

void json_regmap(const uint8_t regAddr, const  uint16_t regVal)
{
    PRINT("{\"register\":{\"address\":0x%02X,\"value\":0x%04X}}\r\n", regAddr, regVal);
}

void json_command(const char *cmdStr, const char *descriptionStr)
{
    PRINT("{\"command\":{\"string\":\"%s\",\"description\":\"%s\"}}\r\n", cmdStr, descriptionStr);
}



// Helper functions
static const char* bool2str(const bool logic)
{
    return (logic ? true_string : false_string);
}
