#ifndef GC_JSON_H_
#define GC_JSON_H_

#include <stdbool.h>
#include <stdint.h>

#include "lib/terminal.h"
#include "usb/usbstdio.h"

// EVM states
typedef enum {IDLE, BUSY, COLLECTING} evmState_t;

// List of status values
#define ERROR_NONE                      (0)
#define ERROR_DEVICE_UNRESPONSIVE       (1)
#define ERROR_BAD_COMMAND               (2)

void json_acknowledge(const char *commandBuffer);
void json_console(const char *pcString, ...);
void json_error(const uint32_t error_code);
void json_evmState(const evmState_t state);
void json_deviceModes(const bool standby, const bool locked);
void json_id(const char *name, const char *date, const char *time);
//void json_pinLevels(const bool nSYNC_logic);
void json_preamble(void);
void json_regmap(const uint8_t regAddr, const uint16_t regVal);
void json_command(const char *cmdStr, const char *descriptionStr);


#endif /* GC_JSON_H_ */
