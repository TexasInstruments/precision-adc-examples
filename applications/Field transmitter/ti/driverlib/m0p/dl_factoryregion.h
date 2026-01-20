/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
/*!****************************************************************************
 *  @file       dl_factoryregion.h
 *  @brief      Factory Region Driver Library
 *  @defgroup   FACTORYREGION Factory Region (FACTORYREGION)
 *
 *  @anchor ti_dl_dl_factoryregion_Overview
 *  # Overview
 *
 *  <hr>
 ******************************************************************************
 */
/** @addtogroup FACTORYREGION
 * @{
 */
#ifndef ti_dl_dl_factoryregion__include
#define ti_dl_dl_factoryregion__include

#include <stdbool.h>
#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/dl_common.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __MSPM0GX51X_TRIM_CACHE__

/*!
 * @brief Macro indicating SRAM trim table has been
          initialized
 *
 * This define is used to indicate that the trim
 * table in SRAM has been initialized with values
 * from TRIM.
 */
#define DL_FACTORYREGION_TRIM_FLAG_IS_SET (0xCAFECAFE)

/*! @brief  Data struct for @ref DL_FactoryRegion_initTrimTable. */
typedef struct {
    /*! Trace identifier */
    uint32_t TRACEID;
    /*! Device identifier */
    uint32_t DEVICEID;
    /*! Device variant identifier */
    uint32_t USERID;
    /*! BSL UART pin configuration */
    uint32_t BSLPIN_UART;
    /*! BSL I2C pin configuration */
    uint32_t BSLPIN_I2C;
    /*! BSL pin invocation configuration */
    uint32_t BSLPIN_INVOKE;
    /*! Main, SRAM, and data bank memory size configuration */
    uint32_t SRAMFLASH;
    /*! SYSPLL0 4-8MHZ factory configuration */
    uint32_t PLLSTARTUP0_4_8MHZ;
    /*! SYSPLL1 4-8MHZ factory configuration */
    uint32_t PLLSTARTUP1_4_8MHZ;
    /*! SYSPLL0 8-16MHZ factory configuration */
    uint32_t PLLSTARTUP0_8_16MHZ;
    /*! SYSPLL1 8-16MHZ factory configuration */
    uint32_t PLLSTARTUP1_8_16MHZ;
    /*! SYSPLL0 16-32MHZ factory configuration */
    uint32_t PLLSTARTUP0_16_32MHZ;
    /*! SYSPLL1 16-32MHZ factory configuration */
    uint32_t PLLSTARTUP1_16_32MHZ;
    /*! SYSPLL0 32-48MHZ factory configuration */
    uint32_t PLLSTARTUP0_32_48MHZ;
    /*! SYSPLL1 32-48MHZ factory configuration */
    uint32_t PLLSTARTUP1_32_48MHZ;
    /*! Temperature sensor room temperature calibration code */
    uint32_t TEMP_SENSE0;
    /*! SRAM trim table initialization flag */
    uint32_t DATA_SET;
} DL_FACTORYREGION_TrimValues;

/*!
 * @brief Pointer to trim table values in SRAM
 */
extern DL_FACTORYREGION_TrimValues *FACTORYVALUE;

/**
 *  @brief  Check to see if TRIM content has been copied
 *          to SRAM.
 *
 *  @return      Status of trim table flag.
 *
 *  @retval      Returns '1' if flag has been set.
 *  @retval      Returns '0' if flag has not been set.
 */
__STATIC_INLINE bool DL_FactoryRegion_isTrimTableInSram(void)
{
    return (FACTORYVALUE->DATA_SET == DL_FACTORYREGION_TRIM_FLAG_IS_SET);
}
#endif

/**
 *  @brief   Get the size of the MAIN Flash region
 *
 *  @return  The size of the MAIN Flash region in number of kB.
 */
__STATIC_INLINE uint16_t DL_FactoryRegion_getMAINFlashSize(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint16_t flashSize;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        flashSize = FACTORYVALUE->SRAMFLASH &
                    FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_MASK;
    } else {
        flashSize = FACTORYREGION->SRAMFLASH &
                    FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_MASK;
    }
#else
    uint16_t flashSize =
        FACTORYREGION->SRAMFLASH & FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_MASK;
#endif
    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return flashSize;
}

/**
 *  @brief   Get the size of the SRAM region
 *
 *  @return  The size of the SRAM region in number of kB.
 */
__STATIC_INLINE uint16_t DL_FactoryRegion_getSRAMFlashSize(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint16_t flashSize;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        flashSize =
            (FACTORYVALUE->SRAMFLASH & FACTORYREGION_SRAMFLASH_SRAM_SZ_MASK) >>
            FACTORYREGION_SRAMFLASH_SRAM_SZ_OFS;
    } else {
        flashSize = (FACTORYREGION->SRAMFLASH &
                        FACTORYREGION_SRAMFLASH_SRAM_SZ_MASK) >>
                    FACTORYREGION_SRAMFLASH_SRAM_SZ_OFS;
    }
#else
    uint16_t flashSize =
        (FACTORYREGION->SRAMFLASH & FACTORYREGION_SRAMFLASH_SRAM_SZ_MASK) >>
        FACTORYREGION_SRAMFLASH_SRAM_SZ_OFS;
#endif
    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return flashSize;
}

/**
 *  @brief   Get the size of the DATA Flash region
 *
 *  @return  The size of the DATA Flash region in number of kB.
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getDATAFlashSize(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint16_t flashSize;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        flashSize = (FACTORYVALUE->SRAMFLASH &
                        FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_MASK) >>
                    FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_OFS;
    } else {
        flashSize = (FACTORYREGION->SRAMFLASH &
                        FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_MASK) >>
                    FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_OFS;
    }
#else
    uint16_t flashSize = (FACTORYREGION->SRAMFLASH &
                             FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_MASK) >>
                         FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return flashSize;
}

/**
 *  @brief   Get the number of Flash banks on the device
 *
 *  @return  The number of flash banks. A value between [1, 4].
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getNumBanks(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t numBanks;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        numBanks = ((FACTORYVALUE->SRAMFLASH &
                        FACTORYREGION_SRAMFLASH_MAINNUMBANKS_MASK) >>
                       FACTORYREGION_SRAMFLASH_MAINNUMBANKS_OFS) +
                   (uint8_t) 1;
    } else {
        numBanks = ((FACTORYREGION->SRAMFLASH &
                        FACTORYREGION_SRAMFLASH_MAINNUMBANKS_MASK) >>
                       FACTORYREGION_SRAMFLASH_MAINNUMBANKS_OFS) +
                   (uint8_t) 1;
    }
#else
    uint8_t numBanks = ((FACTORYREGION->SRAMFLASH &
                            FACTORYREGION_SRAMFLASH_MAINNUMBANKS_MASK) >>
                           FACTORYREGION_SRAMFLASH_MAINNUMBANKS_OFS) +
                       (uint8_t) 1;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return numBanks;
}

/**
 *  @brief   Get the trace ID of the device
 *
 *  @return  The trace ID of the device.
 */
__STATIC_INLINE uint32_t DL_FactoryRegion_getTraceID(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint32_t traceID;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        traceID = FACTORYVALUE->TRACEID & FACTORYREGION_TRACEID_DATA_MASK;
    } else {
        traceID = FACTORYREGION->TRACEID & FACTORYREGION_TRACEID_DATA_MASK;
    }
#else
    uint32_t traceID =
        FACTORYREGION->TRACEID & FACTORYREGION_TRACEID_DATA_MASK;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return traceID;
}

/**
 *  @brief   Get JEDEC bank and company code
 *
 *  @return  TI's JEDEC bank and company code
 */
__STATIC_INLINE uint16_t DL_FactoryRegion_getManufacturerCode(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint16_t manufacturerCode;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        manufacturerCode = (FACTORYVALUE->DEVICEID &
                               FACTORYREGION_DEVICEID_MANUFACTURER_MASK) >>
                           FACTORYREGION_DEVICEID_MANUFACTURER_OFS;
    } else {
        manufacturerCode = (FACTORYREGION->DEVICEID &
                               FACTORYREGION_DEVICEID_MANUFACTURER_MASK) >>
                           FACTORYREGION_DEVICEID_MANUFACTURER_OFS;
    }
#else
    uint16_t manufacturerCode =
        (FACTORYREGION->DEVICEID & FACTORYREGION_DEVICEID_MANUFACTURER_MASK) >>
        FACTORYREGION_DEVICEID_MANUFACTURER_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return manufacturerCode;
}

/**
 *  @brief   Get the part number of the device
 *
 *  @return  The part number of the device
 */
__STATIC_INLINE uint16_t DL_FactoryRegion_getPartNumber(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint16_t partNumber;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        partNumber =
            (FACTORYVALUE->DEVICEID & FACTORYREGION_DEVICEID_PARTNUM_MASK) >>
            FACTORYREGION_DEVICEID_PARTNUM_OFS;
    } else {
        partNumber =
            (FACTORYREGION->DEVICEID & FACTORYREGION_DEVICEID_PARTNUM_MASK) >>
            FACTORYREGION_DEVICEID_PARTNUM_OFS;
    }
#else
    uint16_t partNumber =
        (FACTORYREGION->DEVICEID & FACTORYREGION_DEVICEID_PARTNUM_MASK) >>
        FACTORYREGION_DEVICEID_PARTNUM_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return partNumber;
}

/**
 *  @brief   Get the version of the device
 *
 *  @return  The version of the device
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getVersion(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t version;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        version =
            (FACTORYVALUE->DEVICEID & FACTORYREGION_DEVICEID_VERSION_MASK) >>
            FACTORYREGION_DEVICEID_VERSION_OFS;
    } else {
        version =
            (FACTORYREGION->DEVICEID & FACTORYREGION_DEVICEID_VERSION_MASK) >>
            FACTORYREGION_DEVICEID_VERSION_OFS;
    }
#else
    uint8_t version =
        (FACTORYREGION->DEVICEID & FACTORYREGION_DEVICEID_VERSION_MASK) >>
        FACTORYREGION_DEVICEID_VERSION_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return version;
}

/**
 *  @brief   Get the bit pattern identifying the part
 *
 *  @return  The bit pattern identifying the part
 */
__STATIC_INLINE uint16_t DL_FactoryRegion_getUserIDPart(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint16_t userIDPart;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        userIDPart = (FACTORYVALUE->USERID & FACTORYREGION_USERID_PART_MASK) >>
                     FACTORYREGION_USERID_PART_OFS;
    } else {
        userIDPart =
            (FACTORYREGION->USERID & FACTORYREGION_USERID_PART_MASK) >>
            FACTORYREGION_USERID_PART_OFS;
    }
#else
    uint16_t userIDPart =
        (FACTORYREGION->USERID & FACTORYREGION_USERID_PART_MASK) >>
        FACTORYREGION_USERID_PART_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return userIDPart;
}

/**
 *  @brief   Get the bit pattern identifying a variant of a part
 *
 *  @return  The bit pattern identifying a variant of a part
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getUserIDVariant(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t userIDVariant;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        userIDVariant =
            (FACTORYVALUE->USERID & FACTORYREGION_USERID_VARIANT_MASK) >>
            FACTORYREGION_USERID_VARIANT_OFS;
    } else {
        userIDVariant =
            (FACTORYREGION->USERID & FACTORYREGION_USERID_VARIANT_MASK) >>
            FACTORYREGION_USERID_VARIANT_OFS;
    }
#else
    uint8_t userIDVariant =
        (FACTORYREGION->USERID & FACTORYREGION_USERID_VARIANT_MASK) >>
        FACTORYREGION_USERID_VARIANT_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return userIDVariant;
}

/**
 *  @brief   Get the value that preserves compatibility with lesser minorrev values
 *
 *  Monotonic increasing value indicating a new revision that preserves
 *  compatibility with lesser minorrev values. New capability may
 *  be introduced such that lesser minorrev numbers may not be
 *  compatible with greater if the new capability is used
 *
 *  @return  The value that preserves compatibility with lesser minorrev values
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getUserIDMinorRev(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t minorRev;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        minorRev =
            (FACTORYVALUE->USERID & FACTORYREGION_USERID_MINORREV_MASK) >>
            FACTORYREGION_USERID_MINORREV_OFS;
    } else {
        minorRev =
            (FACTORYREGION->USERID & FACTORYREGION_USERID_MINORREV_MASK) >>
            FACTORYREGION_USERID_MINORREV_OFS;
    }
#else
    uint8_t minorRev =
        (FACTORYREGION->USERID & FACTORYREGION_USERID_MINORREV_MASK) >>
        FACTORYREGION_USERID_MINORREV_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return minorRev;
}

/**
 *  @brief   Get the value that preserves compatibility with lesser majorrev values
 *
 *  Monotonic increasing value indicating a new revision significant
 *  enough that users of the device may have to revise PCB or or
 *  software design
 *
 *  @return  The value that tells users they may have to revise PCB or majorrev design
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getUserIDMajorRev(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t majorRev;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        majorRev =
            (FACTORYVALUE->USERID & FACTORYREGION_USERID_MAJORREV_MASK) >>
            FACTORYREGION_USERID_MAJORREV_OFS;
    } else {
        majorRev =
            (FACTORYREGION->USERID & FACTORYREGION_USERID_MAJORREV_MASK) >>
            FACTORYREGION_USERID_MAJORREV_OFS;
    }
#else
    uint8_t majorRev =
        (FACTORYREGION->USERID & FACTORYREGION_USERID_MAJORREV_MASK) >>
        FACTORYREGION_USERID_MAJORREV_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return majorRev;
}

/**
 *  @brief   Get the UART RXD PIN used by BSL
 *
 *  @return  The UART RXD PIN used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinUARTRXDPad(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslUARTRXDPad;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslUARTRXDPad = (FACTORYVALUE->BSLPIN_UART &
                            FACTORYREGION_BSLPIN_UART_UART_RXD_PAD_MASK) >>
                        FACTORYREGION_BSLPIN_UART_UART_RXD_PAD_OFS;
    } else {
        bslUARTRXDPad = (FACTORYREGION->BSLPIN_UART &
                            FACTORYREGION_BSLPIN_UART_UART_RXD_PAD_MASK) >>
                        FACTORYREGION_BSLPIN_UART_UART_RXD_PAD_OFS;
    }
#else
    uint8_t bslUARTRXDPad = (FACTORYREGION->BSLPIN_UART &
                                FACTORYREGION_BSLPIN_UART_UART_RXD_PAD_MASK) >>
                            FACTORYREGION_BSLPIN_UART_UART_RXD_PAD_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslUARTRXDPad;
}

/**
 *  @brief   Get the UART RXD Pin function selection value used by BSL
 *
 *  @return  The UART RXD Pin function selection value used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinUARTRXDFunction(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslUARTRXDFunction;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslUARTRXDFunction = (FACTORYVALUE->BSLPIN_UART &
                                 FACTORYREGION_BSLPIN_UART_UART_RXD_PF_MASK) >>
                             FACTORYREGION_BSLPIN_UART_UART_RXD_PF_OFS;
    } else {
        bslUARTRXDFunction = (FACTORYREGION->BSLPIN_UART &
                                 FACTORYREGION_BSLPIN_UART_UART_RXD_PF_MASK) >>
                             FACTORYREGION_BSLPIN_UART_UART_RXD_PF_OFS;
    }
#else
    uint8_t bslUARTRXDFunction =
        (FACTORYREGION->BSLPIN_UART &
            FACTORYREGION_BSLPIN_UART_UART_RXD_PF_MASK) >>
        FACTORYREGION_BSLPIN_UART_UART_RXD_PF_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslUARTRXDFunction;
}

/**
 *  @brief   Get the UART TXD PIN used by BSL
 *
 *  @return  The UART TXD PIN used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinUARTTXDPad(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslUARTTXDPad;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslUARTTXDPad = (FACTORYVALUE->BSLPIN_UART &
                            FACTORYREGION_BSLPIN_UART_UART_TXD_PAD_MASK) >>
                        FACTORYREGION_BSLPIN_UART_UART_TXD_PAD_OFS;
    } else {
        bslUARTTXDPad = (FACTORYREGION->BSLPIN_UART &
                            FACTORYREGION_BSLPIN_UART_UART_TXD_PAD_MASK) >>
                        FACTORYREGION_BSLPIN_UART_UART_TXD_PAD_OFS;
    }
#else
    uint8_t bslUARTTXDPad = (FACTORYREGION->BSLPIN_UART &
                                FACTORYREGION_BSLPIN_UART_UART_TXD_PAD_MASK) >>
                            FACTORYREGION_BSLPIN_UART_UART_TXD_PAD_OFS;
#endif
    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslUARTTXDPad;
}

/**
 *  @brief   Get the UART TXD Pin function selection value used by BSL
 *
 *  @return  The UART TXD Pin function selection value used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinUARTTXDFunction(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslUARTTXDFunction;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslUARTTXDFunction = (FACTORYVALUE->BSLPIN_UART &
                                 FACTORYREGION_BSLPIN_UART_UART_TXD_PF_MASK) >>
                             FACTORYREGION_BSLPIN_UART_UART_TXD_PF_OFS;
    } else {
        bslUARTTXDFunction = (FACTORYREGION->BSLPIN_UART &
                                 FACTORYREGION_BSLPIN_UART_UART_TXD_PF_MASK) >>
                             FACTORYREGION_BSLPIN_UART_UART_TXD_PF_OFS;
    }
#else
    uint8_t bslUARTTXDFunction =
        (FACTORYREGION->BSLPIN_UART &
            FACTORYREGION_BSLPIN_UART_UART_TXD_PF_MASK) >>
        FACTORYREGION_BSLPIN_UART_UART_TXD_PF_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslUARTTXDFunction;
}

/**
 *  @brief   Get the I2C SDA Pin used by BSL
 *
 *  @return  The I2C SDA Pin used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinI2CSDAPad(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslI2CSDAPad;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslI2CSDAPad = (FACTORYVALUE->BSLPIN_I2C &
                           FACTORYREGION_BSLPIN_I2C_I2C_SDA_PAD_MASK) >>
                       FACTORYREGION_BSLPIN_I2C_I2C_SDA_PAD_OFS;
    } else {
        bslI2CSDAPad = (FACTORYREGION->BSLPIN_I2C &
                           FACTORYREGION_BSLPIN_I2C_I2C_SDA_PAD_MASK) >>
                       FACTORYREGION_BSLPIN_I2C_I2C_SDA_PAD_OFS;
    }
#else
    uint8_t bslI2CSDAPad = (FACTORYREGION->BSLPIN_I2C &
                               FACTORYREGION_BSLPIN_I2C_I2C_SDA_PAD_MASK) >>
                           FACTORYREGION_BSLPIN_I2C_I2C_SDA_PAD_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslI2CSDAPad;
}

/**
 *  @brief   Get the I2C SDA Pin function selection value used by BSL
 *
 *  @return  The I2C SDA Pin function selection value used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinI2CSDAFunction(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslI2CSDAFunction;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslI2CSDAFunction = (FACTORYVALUE->BSLPIN_I2C &
                                FACTORYREGION_BSLPIN_I2C_I2C_SDA_PF_MASK) >>
                            FACTORYREGION_BSLPIN_I2C_I2C_SDA_PF_OFS;
    } else {
        bslI2CSDAFunction = (FACTORYREGION->BSLPIN_I2C &
                                FACTORYREGION_BSLPIN_I2C_I2C_SDA_PF_MASK) >>
                            FACTORYREGION_BSLPIN_I2C_I2C_SDA_PF_OFS;
    }
#else
    uint8_t bslI2CSDAFunction =
        (FACTORYREGION->BSLPIN_I2C &
            FACTORYREGION_BSLPIN_I2C_I2C_SDA_PF_MASK) >>
        FACTORYREGION_BSLPIN_I2C_I2C_SDA_PF_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslI2CSDAFunction;
}

/**
 *  @brief   Get the I2C SCL Pin used by BSL
 *
 *  @return  The I2C SCL Pin used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinI2CSCLPad(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslI2CSCLPad;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslI2CSCLPad = (FACTORYVALUE->BSLPIN_I2C &
                           FACTORYREGION_BSLPIN_I2C_I2C_SCL_PAD_MASK) >>
                       FACTORYREGION_BSLPIN_I2C_I2C_SCL_PAD_OFS;
    } else {
        bslI2CSCLPad = (FACTORYREGION->BSLPIN_I2C &
                           FACTORYREGION_BSLPIN_I2C_I2C_SCL_PAD_MASK) >>
                       FACTORYREGION_BSLPIN_I2C_I2C_SCL_PAD_OFS;
    }
#else
    uint8_t bslI2CSCLPad = (FACTORYREGION->BSLPIN_I2C &
                               FACTORYREGION_BSLPIN_I2C_I2C_SCL_PAD_MASK) >>
                           FACTORYREGION_BSLPIN_I2C_I2C_SCL_PAD_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslI2CSCLPad;
}

/**
 *  @brief   Get the I2C SCL Pin function selection value used by BSL
 *
 *  @return  The I2C SCL Pin function selection value used by BSL
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinI2CSCLFunction(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslI2CSCLFunction;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslI2CSCLFunction = (FACTORYVALUE->BSLPIN_I2C &
                                FACTORYREGION_BSLPIN_I2C_I2C_SCL_PF_MASK) >>
                            FACTORYREGION_BSLPIN_I2C_I2C_SCL_PF_OFS;
    } else {
        bslI2CSCLFunction = (FACTORYREGION->BSLPIN_I2C &
                                FACTORYREGION_BSLPIN_I2C_I2C_SCL_PF_MASK) >>
                            FACTORYREGION_BSLPIN_I2C_I2C_SCL_PF_OFS;
    }
#else
    uint8_t bslI2CSCLFunction =
        (FACTORYREGION->BSLPIN_I2C &
            FACTORYREGION_BSLPIN_I2C_I2C_SCL_PF_MASK) >>
        FACTORYREGION_BSLPIN_I2C_I2C_SCL_PF_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslI2CSCLFunction;
}

/**
 *  @brief   Get the GPIO level used by BSL pin invocation
 *
 *  @return  The GPIO level used by BSL pin invocation
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinInvokeGPIOLevel(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslInvokeGPIOLevel;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslInvokeGPIOLevel =
            (FACTORYVALUE->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_GPIO_LEVEL_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_GPIO_LEVEL_OFS;
    } else {
        bslInvokeGPIOLevel =
            (FACTORYREGION->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_GPIO_LEVEL_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_GPIO_LEVEL_OFS;
    }
#else
    uint8_t bslInvokeGPIOLevel =
        (FACTORYREGION->BSLPIN_INVOKE &
            FACTORYREGION_BSLPIN_INVOKE_GPIO_LEVEL_MASK) >>
        FACTORYREGION_BSLPIN_INVOKE_GPIO_LEVEL_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslInvokeGPIOLevel;
}

/**
 *  @brief   Get the GPIO Pin selection value used by BSL pin invocation
 *
 *  @return  The GPIO Pin selection value used by BSL pin invocation
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinInvokeGPIOPin(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslInvokeGPIOPin;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslInvokeGPIOPin =
            (FACTORYVALUE->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_GPIO_PIN_SEL_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_GPIO_PIN_SEL_OFS;
    } else {
        bslInvokeGPIOPin =
            (FACTORYREGION->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_GPIO_PIN_SEL_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_GPIO_PIN_SEL_OFS;
    }
#else
    uint8_t bslInvokeGPIOPin =
        (FACTORYREGION->BSLPIN_INVOKE &
            FACTORYREGION_BSLPIN_INVOKE_GPIO_PIN_SEL_MASK) >>
        FACTORYREGION_BSLPIN_INVOKE_GPIO_PIN_SEL_OFS;
#endif
    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslInvokeGPIOPin;
}

/**
 *  @brief   Get the GPIO module selection used by BSL pin invocation
 *
 *  @return  The GPIO module selection used by BSL pin invocation
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinInvokeGPIOModule(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslInvokeGPIOModule;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslInvokeGPIOModule =
            (FACTORYVALUE->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_GPIO_REG_SEL_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_GPIO_REG_SEL_OFS;
    } else {
        bslInvokeGPIOModule =
            (FACTORYREGION->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_GPIO_REG_SEL_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_GPIO_REG_SEL_OFS;
    }
#else
    uint8_t bslInvokeGPIOModule =
        (FACTORYREGION->BSLPIN_INVOKE &
            FACTORYREGION_BSLPIN_INVOKE_GPIO_REG_SEL_MASK) >>
        FACTORYREGION_BSLPIN_INVOKE_GPIO_REG_SEL_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslInvokeGPIOModule;
}

/**
 *  @brief   Get the BSL invocation pin number
 *
 *  @return  The BSL invocation pin number
 */
__STATIC_INLINE uint8_t DL_FactoryRegion_getBSLPinInvokeGPIOModulePad(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint8_t bslInvokeGPIOModulePad;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        bslInvokeGPIOModulePad =
            (FACTORYVALUE->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_BSL_PAD_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_BSL_PAD_OFS;
    } else {
        bslInvokeGPIOModulePad =
            (FACTORYREGION->BSLPIN_INVOKE &
                FACTORYREGION_BSLPIN_INVOKE_BSL_PAD_MASK) >>
            FACTORYREGION_BSLPIN_INVOKE_BSL_PAD_OFS;
    }
#else
    uint8_t bslInvokeGPIOModulePad =
        (FACTORYREGION->BSLPIN_INVOKE &
            FACTORYREGION_BSLPIN_INVOKE_BSL_PAD_MASK) >>
        FACTORYREGION_BSLPIN_INVOKE_BSL_PAD_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return bslInvokeGPIOModulePad;
}

/**
 *  @brief   Get the ADC conversion results of temperature sensor output voltage
 *
 * Returns the temperature sensor output voltage at the factory trim temperature
 * in ADC result code format. This ADC result code is based upon 12-bit
 * sampling mode together with the 1.4-V internal voltage reference.
 *
 *  @return  The ADC conversion results of temperature sensor output voltage
 */
__STATIC_INLINE uint32_t DL_FactoryRegion_getTemperatureVoltage(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

#ifdef __MSPM0GX51X_TRIM_CACHE__
    uint32_t tempVoltage;
    if (DL_FactoryRegion_isTrimTableInSram()) {
        tempVoltage = (FACTORYVALUE->TEMP_SENSE0 &
                          FACTORYREGION_TEMP_SENSE0_DATA_MASK) >>
                      FACTORYREGION_TEMP_SENSE0_DATA_OFS;
    } else {
        tempVoltage = (FACTORYREGION->TEMP_SENSE0 &
                          FACTORYREGION_TEMP_SENSE0_DATA_MASK) >>
                      FACTORYREGION_TEMP_SENSE0_DATA_OFS;
    }
#else
    uint32_t tempVoltage =
        (FACTORYREGION->TEMP_SENSE0 & FACTORYREGION_TEMP_SENSE0_DATA_MASK) >>
        FACTORYREGION_TEMP_SENSE0_DATA_OFS;
#endif

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return tempVoltage;
}

/**
 *  @brief   Get the BOOTCRC value
 *
 *  The 32-bit CRC of all locations in OPEN including reserved locations.
 *
 *  @return  The BOOTCRC value
 */
__STATIC_INLINE uint32_t DL_FactoryRegion_getBOOTCRCData(void)
{
    /* Save CPUSS state and then disable the cache before TRIM access */
    uint32_t ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED, DL_CORE_CACHE_DISABLED,
        DL_CORE_LITERAL_CACHE_ENABLED);

    uint32_t BOOTCRCData =
        (FACTORYREGION->BOOTCRC & FACTORYREGION_BOOTCRC_DATA_MASK) >>
        FACTORYREGION_BOOTCRC_DATA_OFS;

    /* Restore CPUSS state */
    CPUSS->CTL = ctlTemp;

    return BOOTCRCData;
}

#ifdef __MSPM0GX51X_TRIM_CACHE__
/**
 *  @brief Initializes the struct with values from TRIM
 *
 *  Workaround for TRIM region becoming unreadable upon
 *  setting flash wait states equal to 2 on MSPM0GX51X
 *  devices.
 *
 */
void DL_FactoryRegion_initTrimTable(void);
#endif

#ifdef __MSPM0C110X_ADC_ERR_06__
/**
 *  @brief Get the ADC offset value
 *
 *  Workaround for errata on MSPM0C110x devices: ADC_ERR_06
 *  0x41C40040 is expected to point to factory region reserved0 bit
 *
 *  @return  ADC offset value
 */
__STATIC_INLINE float DL_FactoryRegion_getADCOffset(void)
{
    return ((float) (*(int16_t *) (0x41C40040)));
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ti_dl_dl_factoryregion__include */
/** @}*/
