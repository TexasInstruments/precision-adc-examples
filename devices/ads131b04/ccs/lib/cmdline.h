//*****************************************************************************
//
// cmdline.h - Prototypes for command line processing functions.
//
// Copyright (c) 2007 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************

#ifndef __CMDLINE_H__
#define __CMDLINE_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup cmdline_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Defines the maximum number of arguments that can be parsed.
//  (Must be in the range of 2 to 255)
//
//*****************************************************************************
#define CMDLINE_MAX_ARGS        (30)

//*****************************************************************************
//
// Defines the character used to separate command line arguments
//
//*****************************************************************************
#define CMDLINE_ARG_SEPARATOR   (' ')   // Space

//*****************************************************************************
//
//! Defines the value that is returned if the command is not found.
//
//*****************************************************************************
#define CMDLINE_BAD_CMD         (-1)

//*****************************************************************************
//
//! Defines the value that is returned if there are too many arguments.
//
//*****************************************************************************
#define CMDLINE_TOO_MANY_ARGS   (-2)

//*****************************************************************************
//
//! Defines the value that is returned if there are too few arguments.
//
//*****************************************************************************
#define CMDLINE_TOO_FEW_ARGS   (-3)

//*****************************************************************************
//
//! Defines the value that is returned if an argument is invalid.
//
//*****************************************************************************
#define CMDLINE_INVALID_ARG   (-4)

//*****************************************************************************
//
//! Defines the value that is returned if the device is locked.
//
//*****************************************************************************
#define CMDLINE_DEVICE_LOCKED   (-5)

//*****************************************************************************
//
// Command line function callback type.
//
//*****************************************************************************
typedef int (*pfnCmdLine)(const int argc, const char *argv[]);

//*****************************************************************************
//
//! Structure for an entry in the command list table.
//
//*****************************************************************************
typedef struct
{
    //
    //! A pointer to a string containing the name of the command.
    //
    const char *pcCmd;

    //
    //! A function pointer to the implementation of the command.
    //
    pfnCmdLine pfnCmd;

    //
    //! A pointer to a string of brief help text for the command.
    //
    const char *pcHelp;
}
tCmdLineEntry;

//*****************************************************************************
//
//! This is the command table that must be provided by the application.  The
//! last element of the array must be a structure whose pcCmd field contains
//! a NULL pointer.
//
//*****************************************************************************
extern tCmdLineEntry g_psCmdTable[];

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern int CmdLineProcess(char *pcCmdLine);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __CMDLINE_H__
