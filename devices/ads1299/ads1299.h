/* <INSERT COPYRIGHT & LICENSE TEXT HERE> */

#ifndef ADS1299_H_
#define ADS1299_H_

#include <stdint.h>


//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************



//**********************************************************************************
//
// Device commands
//
//**********************************************************************************



//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (ID) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |                    REV_ID[2:0]                   |    RESERVED0   |           DEV_ID[1:0]           |            NU_CH[1:0]           |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* ID register address */
    #define ID_ADDRESS														((uint8_t) 0x00)

    /* ID default (reset) value */
    #define ID_DEFAULT														((uint8_t) 0x1E)

    /* ID register field masks */
    #define ID_REV_ID_MASK													((uint8_t) 0xE0)
    #define ID_RESERVED0_MASK												((uint8_t) 0x10)
    #define ID_DEV_ID_MASK													((uint8_t) 0x0C)
    #define ID_NU_CH_MASK													((uint8_t) 0x03)

    /* DEV_ID field values */
    #define ID_DEV_ID_ADS1299x												((uint8_t) 0x0C)

    /* NU_CH field values */
    #define ID_NU_CH_4channelADS12994										((uint8_t) 0x00)
    #define ID_NU_CH_6channelADS12996										((uint8_t) 0x01)
    #define ID_NU_CH_8channelADS1299										((uint8_t) 0x02)



/* Register 0x01 (CONFIG1) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |    RESERVED0   |    NDAISY_EN   |     CLK_EN     |          RESERVED1[1:0]         |                      DR[2:0]                     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONFIG1 register address */
    #define CONFIG1_ADDRESS													((uint8_t) 0x01)

    /* CONFIG1 default (reset) value */
    #define CONFIG1_DEFAULT													((uint8_t) 0x96)

    /* CONFIG1 register field masks */
    #define CONFIG1_RESERVED0_MASK											((uint8_t) 0x80)
    #define CONFIG1_NDAISY_EN_MASK											((uint8_t) 0x40)
    #define CONFIG1_CLK_EN_MASK												((uint8_t) 0x20)
    #define CONFIG1_RESERVED1_MASK											((uint8_t) 0x18)
    #define CONFIG1_DR_MASK													((uint8_t) 0x07)

    /* DR field values */
    #define CONFIG1_DR_16kSPS												((uint8_t) 0x00)
    #define CONFIG1_DR_8kSPS												((uint8_t) 0x01)
    #define CONFIG1_DR_4kSPS												((uint8_t) 0x02)
    #define CONFIG1_DR_2kSPS												((uint8_t) 0x03)
    #define CONFIG1_DR_1kSPS												((uint8_t) 0x04)
    #define CONFIG1_DR_500SPS												((uint8_t) 0x05)
    #define CONFIG1_DR_250SPS												((uint8_t) 0x06)



/* Register 0x02 (CONFIG2) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |                  RESERVED0[2:0]                  |     INT_CAL    |    RESERVED1   |    CAL_AMP0    |          CAL_FREQ[1:0]          |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONFIG2 register address */
    #define CONFIG2_ADDRESS													((uint8_t) 0x02)

    /* CONFIG2 default (reset) value */
    #define CONFIG2_DEFAULT													((uint8_t) 0xC0)

    /* CONFIG2 register field masks */
    #define CONFIG2_RESERVED0_MASK											((uint8_t) 0xE0)
    #define CONFIG2_INT_CAL_MASK											((uint8_t) 0x10)
    #define CONFIG2_RESERVED1_MASK											((uint8_t) 0x08)
    #define CONFIG2_CAL_AMP0_MASK											((uint8_t) 0x04)
    #define CONFIG2_CAL_FREQ_MASK											((uint8_t) 0x03)

    /* CAL_FREQ field values */
    #define CONFIG2_CAL_FREQ_PulsedatfCLK221								((uint8_t) 0x00)
    #define CONFIG2_CAL_FREQ_PulsedatfCLK220								((uint8_t) 0x01)
    #define CONFIG2_CAL_FREQ_Atdc											((uint8_t) 0x03)



/* Register 0x03 (CONFIG3) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |   NPD_REFBUF   |          RESERVED0[1:0]         |    BIAS_MEAS   |   BIAS_REFINT  |    NPD_BIAS    | BIAS_LOFF_SENS |    BIAS_STAT   |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONFIG3 register address */
    #define CONFIG3_ADDRESS													((uint8_t) 0x03)

    /* CONFIG3 default (reset) value */
    #define CONFIG3_DEFAULT													((uint8_t) 0x60)

    /* CONFIG3 register field masks */
    #define CONFIG3_NPD_REFBUF_MASK											((uint8_t) 0x80)
    #define CONFIG3_RESERVED0_MASK											((uint8_t) 0x60)
    #define CONFIG3_BIAS_MEAS_MASK											((uint8_t) 0x10)
    #define CONFIG3_BIAS_REFINT_MASK										((uint8_t) 0x08)
    #define CONFIG3_NPD_BIAS_MASK											((uint8_t) 0x04)
    #define CONFIG3_BIAS_LOFF_SENS_MASK										((uint8_t) 0x02)
    #define CONFIG3_BIAS_STAT_MASK											((uint8_t) 0x01)



/* Register 0x04 (LOFF) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |                   COMP_TH[2:0]                   |    RESERVED0   |          ILEAD_OFF[1:0]         |          FLEAD_OFF[1:0]         |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* LOFF register address */
    #define LOFF_ADDRESS													((uint8_t) 0x04)

    /* LOFF default (reset) value */
    #define LOFF_DEFAULT													((uint8_t) 0x00)

    /* LOFF register field masks */
    #define LOFF_COMP_TH_MASK												((uint8_t) 0xE0)
    #define LOFF_RESERVED0_MASK												((uint8_t) 0x10)
    #define LOFF_ILEAD_OFF_MASK												((uint8_t) 0x0C)
    #define LOFF_FLEAD_OFF_MASK												((uint8_t) 0x03)

    /* COMP_TH field values */
    #define LOFF_COMP_TH_95_5_PERCENT										((uint8_t) 0x00)
    #define LOFF_COMP_TH_925_75_PERCENT										((uint8_t) 0x20)
    #define LOFF_COMP_TH_90_10_PERCENT										((uint8_t) 0x40)
    #define LOFF_COMP_TH_875_125_PERCENT									((uint8_t) 0x60)
    #define LOFF_COMP_TH_85_15_PERCENT										((uint8_t) 0x80)
    #define LOFF_COMP_TH_80_20_PERCENT										((uint8_t) 0xA0)
    #define LOFF_COMP_TH_75_25_PERCENT										((uint8_t) 0xC0)
    #define LOFF_COMP_TH_70_30_PERCENT										((uint8_t) 0xE0)

    /* ILEAD_OFF field values */
    #define LOFF_ILEAD_OFF_6nA												((uint8_t) 0x00)
    #define LOFF_ILEAD_OFF_24nA												((uint8_t) 0x04)
    #define LOFF_ILEAD_OFF_6uA												((uint8_t) 0x08)
    #define LOFF_ILEAD_OFF_24uA												((uint8_t) 0x0C)

    /* FLEAD_OFF field values */
    #define LOFF_FLEAD_OFF_DC												((uint8_t) 0x00)
    #define LOFF_FLEAD_OFF_AC_78_HZ											((uint8_t) 0x01)
    #define LOFF_FLEAD_OFF_AC_312_HZ										((uint8_t) 0x02)
    #define LOFF_FLEAD_OFF_AC_FDR4_HZ										((uint8_t) 0x03)



/* Register 0x05 (CH1SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD1      |                    GAIN1[2:0]                    |      SRB2      |                     MUX1[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH1SET register address */
    #define CH1SET_ADDRESS													((uint8_t) 0x05)

    /* CH1SET default (reset) value */
    #define CH1SET_DEFAULT													((uint8_t) 0x61)

    /* CH1SET register field masks */
    #define CH1SET_PD1_MASK													((uint8_t) 0x80)
    #define CH1SET_GAIN1_MASK												((uint8_t) 0x70)
    #define CH1SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH1SET_MUX1_MASK												((uint8_t) 0x07)

    /* GAIN1 field values */
    #define CH1SET_GAIN1_1													((uint8_t) 0x00)
    #define CH1SET_GAIN1_2													((uint8_t) 0x10)
    #define CH1SET_GAIN1_4													((uint8_t) 0x20)
    #define CH1SET_GAIN1_6													((uint8_t) 0x30)
    #define CH1SET_GAIN1_8													((uint8_t) 0x40)
    #define CH1SET_GAIN1_12													((uint8_t) 0x50)
    #define CH1SET_GAIN1_24													((uint8_t) 0x60)

    /* MUX1 field values */
    #define CH1SET_MUX1_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH1SET_MUX1_INPUT_SHORT											((uint8_t) 0x01)
    #define CH1SET_MUX1_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH1SET_MUX1_MVDD												((uint8_t) 0x03)
    #define CH1SET_MUX1_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH1SET_MUX1_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH1SET_MUX1_BIAS_DRP											((uint8_t) 0x06)
    #define CH1SET_MUX1_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x06 (CH2SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD2      |                    GAIN2[2:0]                    |      SRB2      |                     MUX2[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH2SET register address */
    #define CH2SET_ADDRESS													((uint8_t) 0x06)

    /* CH2SET default (reset) value */
    #define CH2SET_DEFAULT													((uint8_t) 0x61)

    /* CH2SET register field masks */
    #define CH2SET_PD2_MASK													((uint8_t) 0x80)
    #define CH2SET_GAIN2_MASK												((uint8_t) 0x70)
    #define CH2SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH2SET_MUX2_MASK												((uint8_t) 0x07)

    /* GAIN2 field values */
    #define CH2SET_GAIN2_1													((uint8_t) 0x00)
    #define CH2SET_GAIN2_2													((uint8_t) 0x10)
    #define CH2SET_GAIN2_4													((uint8_t) 0x20)
    #define CH2SET_GAIN2_6													((uint8_t) 0x30)
    #define CH2SET_GAIN2_8													((uint8_t) 0x40)
    #define CH2SET_GAIN2_12													((uint8_t) 0x50)
    #define CH2SET_GAIN2_24													((uint8_t) 0x60)

    /* MUX2 field values */
    #define CH2SET_MUX2_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH2SET_MUX2_INPUT_SHORT											((uint8_t) 0x01)
    #define CH2SET_MUX2_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH2SET_MUX2_MVDD												((uint8_t) 0x03)
    #define CH2SET_MUX2_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH2SET_MUX2_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH2SET_MUX2_BIAS_DRP											((uint8_t) 0x06)
    #define CH2SET_MUX2_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x07 (CH3SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD3      |                    GAIN3[2:0]                    |      SRB2      |                     MUX3[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH3SET register address */
    #define CH3SET_ADDRESS													((uint8_t) 0x07)

    /* CH3SET default (reset) value */
    #define CH3SET_DEFAULT													((uint8_t) 0x61)

    /* CH3SET register field masks */
    #define CH3SET_PD3_MASK													((uint8_t) 0x80)
    #define CH3SET_GAIN3_MASK												((uint8_t) 0x70)
    #define CH3SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH3SET_MUX3_MASK												((uint8_t) 0x07)

    /* GAIN3 field values */
    #define CH3SET_GAIN3_1													((uint8_t) 0x00)
    #define CH3SET_GAIN3_2													((uint8_t) 0x10)
    #define CH3SET_GAIN3_4													((uint8_t) 0x20)
    #define CH3SET_GAIN3_6													((uint8_t) 0x30)
    #define CH3SET_GAIN3_8													((uint8_t) 0x40)
    #define CH3SET_GAIN3_12													((uint8_t) 0x50)
    #define CH3SET_GAIN3_24													((uint8_t) 0x60)

    /* MUX3 field values */
    #define CH3SET_MUX3_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH3SET_MUX3_INPUT_SHORT											((uint8_t) 0x01)
    #define CH3SET_MUX3_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH3SET_MUX3_MVDD												((uint8_t) 0x03)
    #define CH3SET_MUX3_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH3SET_MUX3_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH3SET_MUX3_BIAS_DRP											((uint8_t) 0x06)
    #define CH3SET_MUX3_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x08 (CH4SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD4      |                    GAIN4[2:0]                    |      SRB2      |                     MUX4[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH4SET register address */
    #define CH4SET_ADDRESS													((uint8_t) 0x08)

    /* CH4SET default (reset) value */
    #define CH4SET_DEFAULT													((uint8_t) 0x61)

    /* CH4SET register field masks */
    #define CH4SET_PD4_MASK													((uint8_t) 0x80)
    #define CH4SET_GAIN4_MASK												((uint8_t) 0x70)
    #define CH4SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH4SET_MUX4_MASK												((uint8_t) 0x07)

    /* GAIN4 field values */
    #define CH4SET_GAIN4_1													((uint8_t) 0x00)
    #define CH4SET_GAIN4_2													((uint8_t) 0x10)
    #define CH4SET_GAIN4_4													((uint8_t) 0x20)
    #define CH4SET_GAIN4_6													((uint8_t) 0x30)
    #define CH4SET_GAIN4_8													((uint8_t) 0x40)
    #define CH4SET_GAIN4_12													((uint8_t) 0x50)
    #define CH4SET_GAIN4_24													((uint8_t) 0x60)

    /* MUX4 field values */
    #define CH4SET_MUX4_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH4SET_MUX4_INPUT_SHORT											((uint8_t) 0x01)
    #define CH4SET_MUX4_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH4SET_MUX4_MVDD												((uint8_t) 0x03)
    #define CH4SET_MUX4_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH4SET_MUX4_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH4SET_MUX4_BIAS_DRP											((uint8_t) 0x06)
    #define CH4SET_MUX4_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x09 (CH5SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD5      |                    GAIN5[2:0]                    |      SRB2      |                     MUX5[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH5SET register address */
    #define CH5SET_ADDRESS													((uint8_t) 0x09)

    /* CH5SET default (reset) value */
    #define CH5SET_DEFAULT													((uint8_t) 0x61)

    /* CH5SET register field masks */
    #define CH5SET_PD5_MASK													((uint8_t) 0x80)
    #define CH5SET_GAIN5_MASK												((uint8_t) 0x70)
    #define CH5SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH5SET_MUX5_MASK												((uint8_t) 0x07)

    /* GAIN5 field values */
    #define CH5SET_GAIN5_1													((uint8_t) 0x00)
    #define CH5SET_GAIN5_2													((uint8_t) 0x10)
    #define CH5SET_GAIN5_4													((uint8_t) 0x20)
    #define CH5SET_GAIN5_6													((uint8_t) 0x30)
    #define CH5SET_GAIN5_8													((uint8_t) 0x40)
    #define CH5SET_GAIN5_12													((uint8_t) 0x50)
    #define CH5SET_GAIN5_24													((uint8_t) 0x60)

    /* MUX5 field values */
    #define CH5SET_MUX5_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH5SET_MUX5_INPUT_SHORT											((uint8_t) 0x01)
    #define CH5SET_MUX5_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH5SET_MUX5_MVDD												((uint8_t) 0x03)
    #define CH5SET_MUX5_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH5SET_MUX5_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH5SET_MUX5_BIAS_DRP											((uint8_t) 0x06)
    #define CH5SET_MUX5_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x0A (CH6SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD6      |                    GAIN6[2:0]                    |      SRB2      |                     MUX6[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH6SET register address */
    #define CH6SET_ADDRESS													((uint8_t) 0x0A)

    /* CH6SET default (reset) value */
    #define CH6SET_DEFAULT													((uint8_t) 0x61)

    /* CH6SET register field masks */
    #define CH6SET_PD6_MASK													((uint8_t) 0x80)
    #define CH6SET_GAIN6_MASK												((uint8_t) 0x70)
    #define CH6SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH6SET_MUX6_MASK												((uint8_t) 0x07)

    /* GAIN6 field values */
    #define CH6SET_GAIN6_1													((uint8_t) 0x00)
    #define CH6SET_GAIN6_2													((uint8_t) 0x10)
    #define CH6SET_GAIN6_4													((uint8_t) 0x20)
    #define CH6SET_GAIN6_6													((uint8_t) 0x30)
    #define CH6SET_GAIN6_8													((uint8_t) 0x40)
    #define CH6SET_GAIN6_12													((uint8_t) 0x50)
    #define CH6SET_GAIN6_24													((uint8_t) 0x60)

    /* MUX6 field values */
    #define CH6SET_MUX6_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH6SET_MUX6_INPUT_SHORT											((uint8_t) 0x01)
    #define CH6SET_MUX6_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH6SET_MUX6_MVDD												((uint8_t) 0x03)
    #define CH6SET_MUX6_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH6SET_MUX6_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH6SET_MUX6_BIAS_DRP											((uint8_t) 0x06)
    #define CH6SET_MUX6_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x0B (CH7SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD7      |                    GAIN7[2:0]                    |      SRB2      |                     MUX7[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH7SET register address */
    #define CH7SET_ADDRESS													((uint8_t) 0x0B)

    /* CH7SET default (reset) value */
    #define CH7SET_DEFAULT													((uint8_t) 0x61)

    /* CH7SET register field masks */
    #define CH7SET_PD7_MASK													((uint8_t) 0x80)
    #define CH7SET_GAIN7_MASK												((uint8_t) 0x70)
    #define CH7SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH7SET_MUX7_MASK												((uint8_t) 0x07)

    /* GAIN7 field values */
    #define CH7SET_GAIN7_1													((uint8_t) 0x00)
    #define CH7SET_GAIN7_2													((uint8_t) 0x10)
    #define CH7SET_GAIN7_4													((uint8_t) 0x20)
    #define CH7SET_GAIN7_6													((uint8_t) 0x30)
    #define CH7SET_GAIN7_8													((uint8_t) 0x40)
    #define CH7SET_GAIN7_12													((uint8_t) 0x50)
    #define CH7SET_GAIN7_24													((uint8_t) 0x60)

    /* MUX7 field values */
    #define CH7SET_MUX7_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH7SET_MUX7_INPUT_SHORT											((uint8_t) 0x01)
    #define CH7SET_MUX7_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH7SET_MUX7_MVDD												((uint8_t) 0x03)
    #define CH7SET_MUX7_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH7SET_MUX7_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH7SET_MUX7_BIAS_DRP											((uint8_t) 0x06)
    #define CH7SET_MUX7_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x0C (CH8SET) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |       PD8      |                    GAIN8[2:0]                    |      SRB2      |                     MUX8[2:0]                    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH8SET register address */
    #define CH8SET_ADDRESS													((uint8_t) 0x0C)

    /* CH8SET default (reset) value */
    #define CH8SET_DEFAULT													((uint8_t) 0x61)

    /* CH8SET register field masks */
    #define CH8SET_PD8_MASK													((uint8_t) 0x80)
    #define CH8SET_GAIN8_MASK												((uint8_t) 0x70)
    #define CH8SET_SRB2_MASK												((uint8_t) 0x08)
    #define CH8SET_MUX8_MASK												((uint8_t) 0x07)

    /* GAIN8 field values */
    #define CH8SET_GAIN8_1													((uint8_t) 0x00)
    #define CH8SET_GAIN8_2													((uint8_t) 0x10)
    #define CH8SET_GAIN8_4													((uint8_t) 0x20)
    #define CH8SET_GAIN8_6													((uint8_t) 0x30)
    #define CH8SET_GAIN8_8													((uint8_t) 0x40)
    #define CH8SET_GAIN8_12													((uint8_t) 0x50)
    #define CH8SET_GAIN8_24													((uint8_t) 0x60)

    /* MUX8 field values */
    #define CH8SET_MUX8_NORMAL_ELEC											((uint8_t) 0x00)
    #define CH8SET_MUX8_INPUT_SHORT											((uint8_t) 0x01)
    #define CH8SET_MUX8_BIAS_MEAS_EN										((uint8_t) 0x02)
    #define CH8SET_MUX8_MVDD												((uint8_t) 0x03)
    #define CH8SET_MUX8_TEMP_SENSOR											((uint8_t) 0x04)
    #define CH8SET_MUX8_TEST_SIGNAL											((uint8_t) 0x05)
    #define CH8SET_MUX8_BIAS_DRP											((uint8_t) 0x06)
    #define CH8SET_MUX8_BIAS_DRN											((uint8_t) 0x07)



/* Register 0x0D (BIAS_SENSP) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |     BIASP8     |     BIASP7     |     BIASP6     |     BIASP5     |     BIASP4     |     BIASP3     |     BIASP2     |     BIASP1     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* BIAS_SENSP register address */
    #define BIAS_SENSP_ADDRESS												((uint8_t) 0x0D)

    /* BIAS_SENSP default (reset) value */
    #define BIAS_SENSP_DEFAULT												((uint8_t) 0x00)

    /* BIAS_SENSP register field masks */
    #define BIAS_SENSP_BIASP8_MASK											((uint8_t) 0x80)
    #define BIAS_SENSP_BIASP7_MASK											((uint8_t) 0x40)
    #define BIAS_SENSP_BIASP6_MASK											((uint8_t) 0x20)
    #define BIAS_SENSP_BIASP5_MASK											((uint8_t) 0x10)
    #define BIAS_SENSP_BIASP4_MASK											((uint8_t) 0x08)
    #define BIAS_SENSP_BIASP3_MASK											((uint8_t) 0x04)
    #define BIAS_SENSP_BIASP2_MASK											((uint8_t) 0x02)
    #define BIAS_SENSP_BIASP1_MASK											((uint8_t) 0x01)



/* Register 0x0E (BIAS_SENSN) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |     BIASN8     |     BIASN7     |     BIASN6     |     BIASN5     |     BIASN4     |     BIASN3     |     BIASN2     |     BIASN1     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* BIAS_SENSN register address */
    #define BIAS_SENSN_ADDRESS												((uint8_t) 0x0E)

    /* BIAS_SENSN default (reset) value */
    #define BIAS_SENSN_DEFAULT												((uint8_t) 0x00)

    /* BIAS_SENSN register field masks */
    #define BIAS_SENSN_BIASN8_MASK											((uint8_t) 0x80)
    #define BIAS_SENSN_BIASN7_MASK											((uint8_t) 0x40)
    #define BIAS_SENSN_BIASN6_MASK											((uint8_t) 0x20)
    #define BIAS_SENSN_BIASN5_MASK											((uint8_t) 0x10)
    #define BIAS_SENSN_BIASN4_MASK											((uint8_t) 0x08)
    #define BIAS_SENSN_BIASN3_MASK											((uint8_t) 0x04)
    #define BIAS_SENSN_BIASN2_MASK											((uint8_t) 0x02)
    #define BIAS_SENSN_BIASN1_MASK											((uint8_t) 0x01)



/* Register 0x0F (LOFF_SENSP) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |     LOFFP8     |     LOFFP7     |     LOFFP6     |     LOFFP5     |     LOFFP4     |     LOFFP3     |     LOFFP2     |     LOFFP1     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* LOFF_SENSP register address */
    #define LOFF_SENSP_ADDRESS												((uint8_t) 0x0F)

    /* LOFF_SENSP default (reset) value */
    #define LOFF_SENSP_DEFAULT												((uint8_t) 0x00)

    /* LOFF_SENSP register field masks */
    #define LOFF_SENSP_LOFFP8_MASK											((uint8_t) 0x80)
    #define LOFF_SENSP_LOFFP7_MASK											((uint8_t) 0x40)
    #define LOFF_SENSP_LOFFP6_MASK											((uint8_t) 0x20)
    #define LOFF_SENSP_LOFFP5_MASK											((uint8_t) 0x10)
    #define LOFF_SENSP_LOFFP4_MASK											((uint8_t) 0x08)
    #define LOFF_SENSP_LOFFP3_MASK											((uint8_t) 0x04)
    #define LOFF_SENSP_LOFFP2_MASK											((uint8_t) 0x02)
    #define LOFF_SENSP_LOFFP1_MASK											((uint8_t) 0x01)



/* Register 0x10 (LOFF_SENSN) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |     LOFFM8     |     LOFFM7     |     LOFFM6     |     LOFFM5     |     LOFFM4     |     LOFFM3     |     LOFFM2     |     LOFFM1     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* LOFF_SENSN register address */
    #define LOFF_SENSN_ADDRESS												((uint8_t) 0x10)

    /* LOFF_SENSN default (reset) value */
    #define LOFF_SENSN_DEFAULT												((uint8_t) 0x00)

    /* LOFF_SENSN register field masks */
    #define LOFF_SENSN_LOFFM8_MASK											((uint8_t) 0x80)
    #define LOFF_SENSN_LOFFM7_MASK											((uint8_t) 0x40)
    #define LOFF_SENSN_LOFFM6_MASK											((uint8_t) 0x20)
    #define LOFF_SENSN_LOFFM5_MASK											((uint8_t) 0x10)
    #define LOFF_SENSN_LOFFM4_MASK											((uint8_t) 0x08)
    #define LOFF_SENSN_LOFFM3_MASK											((uint8_t) 0x04)
    #define LOFF_SENSN_LOFFM2_MASK											((uint8_t) 0x02)
    #define LOFF_SENSN_LOFFM1_MASK											((uint8_t) 0x01)



/* Register 0x11 (LOFF_FLIP) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |   LOFF_FLIP8   |   LOFF_FLIP7   |   LOFF_FLIP6   |   LOFF_FLIP5   |   LOFF_FLIP4   |   LOFF_FLIP3   |   LOFF_FLIP2   |   LOFF_FLIP1   |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* LOFF_FLIP register address */
    #define LOFF_FLIP_ADDRESS												((uint8_t) 0x11)

    /* LOFF_FLIP default (reset) value */
    #define LOFF_FLIP_DEFAULT												((uint8_t) 0x00)

    /* LOFF_FLIP register field masks */
    #define LOFF_FLIP_LOFF_FLIP8_MASK										((uint8_t) 0x80)
    #define LOFF_FLIP_LOFF_FLIP7_MASK										((uint8_t) 0x40)
    #define LOFF_FLIP_LOFF_FLIP6_MASK										((uint8_t) 0x20)
    #define LOFF_FLIP_LOFF_FLIP5_MASK										((uint8_t) 0x10)
    #define LOFF_FLIP_LOFF_FLIP4_MASK										((uint8_t) 0x08)
    #define LOFF_FLIP_LOFF_FLIP3_MASK										((uint8_t) 0x04)
    #define LOFF_FLIP_LOFF_FLIP2_MASK										((uint8_t) 0x02)
    #define LOFF_FLIP_LOFF_FLIP1_MASK										((uint8_t) 0x01)



/* Register 0x12 (LOFF_STATP) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |    IN8P_OFF    |    IN7P_OFF    |    IN6P_OFF    |    IN5P_OFF    |    IN4P_OFF    |    IN3P_OFF    |    IN2P_OFF    |    IN1P_OFF    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* LOFF_STATP register address */
    #define LOFF_STATP_ADDRESS												((uint8_t) 0x12)

    /* LOFF_STATP default (reset) value */
    #define LOFF_STATP_DEFAULT												((uint8_t) 0x00)

    /* LOFF_STATP register field masks */
    #define LOFF_STATP_IN8P_OFF_MASK										((uint8_t) 0x80)
    #define LOFF_STATP_IN7P_OFF_MASK										((uint8_t) 0x40)
    #define LOFF_STATP_IN6P_OFF_MASK										((uint8_t) 0x20)
    #define LOFF_STATP_IN5P_OFF_MASK										((uint8_t) 0x10)
    #define LOFF_STATP_IN4P_OFF_MASK										((uint8_t) 0x08)
    #define LOFF_STATP_IN3P_OFF_MASK										((uint8_t) 0x04)
    #define LOFF_STATP_IN2P_OFF_MASK										((uint8_t) 0x02)
    #define LOFF_STATP_IN1P_OFF_MASK										((uint8_t) 0x01)



/* Register 0x13 (LOFF_STATN) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |    IN8N_OFF    |    IN7N_OFF    |    IN6N_OFF    |    IN5N_OFF    |    IN4N_OFF    |    IN3N_OFF    |    IN2N_OFF    |    IN1N_OFF    |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* LOFF_STATN register address */
    #define LOFF_STATN_ADDRESS												((uint8_t) 0x13)

    /* LOFF_STATN default (reset) value */
    #define LOFF_STATN_DEFAULT												((uint8_t) 0x00)

    /* LOFF_STATN register field masks */
    #define LOFF_STATN_IN8N_OFF_MASK										((uint8_t) 0x80)
    #define LOFF_STATN_IN7N_OFF_MASK										((uint8_t) 0x40)
    #define LOFF_STATN_IN6N_OFF_MASK										((uint8_t) 0x20)
    #define LOFF_STATN_IN5N_OFF_MASK										((uint8_t) 0x10)
    #define LOFF_STATN_IN4N_OFF_MASK										((uint8_t) 0x08)
    #define LOFF_STATN_IN3N_OFF_MASK										((uint8_t) 0x04)
    #define LOFF_STATN_IN2N_OFF_MASK										((uint8_t) 0x02)
    #define LOFF_STATN_IN1N_OFF_MASK										((uint8_t) 0x01)



/* Register 0x14 (GPIO) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |                             GPIOD[3:0]                            |                             GPIOC[3:0]                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* GPIO register address */
    #define GPIO_ADDRESS													((uint8_t) 0x14)

    /* GPIO default (reset) value */
    #define GPIO_DEFAULT													((uint8_t) 0x00)

    /* GPIO register field masks */
    #define GPIO_GPIOD_MASK													((uint8_t) 0xF0)
    #define GPIO_GPIOC_MASK													((uint8_t) 0x0F)

    /* GPIOC field values */
    #define GPIO_GPIOC_Output												((uint8_t) 0x00)
    #define GPIO_GPIOC_Input												((uint8_t) 0x01)



/* Register 0x15 (MISC1) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |          RESERVED0[1:0]         |      SRB1      |                                   RESERVED1[4:0]                                   |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* MISC1 register address */
    #define MISC1_ADDRESS													((uint8_t) 0x15)

    /* MISC1 default (reset) value */
    #define MISC1_DEFAULT													((uint8_t) 0x00)

    /* MISC1 register field masks */
    #define MISC1_RESERVED0_MASK											((uint8_t) 0xC0)
    #define MISC1_SRB1_MASK													((uint8_t) 0x20)
    #define MISC1_RESERVED1_MASK											((uint8_t) 0x1F)



/* Register 0x16 (MISC2) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |                                                             RESERVED0[7:0]                                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* MISC2 register address */
    #define MISC2_ADDRESS													((uint8_t) 0x16)

    /* MISC2 default (reset) value */
    #define MISC2_DEFAULT													((uint8_t) 0x00)

    /* MISC2 register field masks */
    #define MISC2_RESERVED0_MASK											((uint8_t) 0xFF)



/* Register 0x17 (CONFIG4) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 7     |      Bit 6     |      Bit 5     |      Bit 4     |      Bit 3     |      Bit 2     |      Bit 1     |      Bit 0     |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 * |                           RESERVED0[3:0]                          |   SINGLE_SHOT  |    RESERVED1   |  NPD_LOFF_COMP |    RESERVED2   |
 * -----------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONFIG4 register address */
    #define CONFIG4_ADDRESS													((uint8_t) 0x17)

    /* CONFIG4 default (reset) value */
    #define CONFIG4_DEFAULT													((uint8_t) 0x00)

    /* CONFIG4 register field masks */
    #define CONFIG4_RESERVED0_MASK											((uint8_t) 0xF0)
    #define CONFIG4_SINGLE_SHOT_MASK										((uint8_t) 0x08)
    #define CONFIG4_RESERVED1_MASK											((uint8_t) 0x04)
    #define CONFIG4_NPD_LOFF_COMP_MASK										((uint8_t) 0x02)
    #define CONFIG4_RESERVED2_MASK											((uint8_t) 0x01)



#endif /* ADS1299_H_ */
