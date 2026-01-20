/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 * PURPOSE ARE DISCLAIMED. IN NO SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY TfHEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!****************************************************************************
 *  @file       dl_rtc_b.h
 *  @brief      Real Time Clock A (RTC_a) Peripheral Interface
 *  @defgroup   RTC_B Real Time Clock A (RTC_B)
 *
 *  @anchor ti_devices_msp_dl_rtc_b_Overview
 *  # Overview
 *  The RTC_B Driver Library allows full configuration of the MSPM0 RTC_B module.
 *  The real-time clock (RTC_B) module provides clock counters with calendar mode,
 *  a flexible programmable alarm, offset calibration, and a provision for
 *  temperature compensation. The module also provides a separate time stamp calendar
 *  capable of capturing a time stamp event upon loss of VDD or detection of a tamper event
 *  on a tamper I/O pin.
 *
 *  <hr>
 ******************************************************************************
 */
/** @addtogroup RTC_B
 * @{
 */
#ifndef ti_dl_dl_rtc_b__include
#define ti_dl_dl_rtc_b__include

#include <ti/driverlib/dl_rtc_common.h>

#ifdef __MSPM0_HAS_RTC_B__

#ifdef __cplusplus
extern "C" {
#endif

/* clang-format off */

/** @addtogroup DL_RTC_B_INTERRUPT
 *  @{
 */

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERRUPT_CALENDAR_ALARM1
 */
#define DL_RTC_B_INTERRUPT_CALENDAR_ALARM1          (DL_RTC_COMMON_INTERRUPT_CALENDAR_ALARM1)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERRUPT_CALENDAR_ALARM2
 */
#define DL_RTC_B_INTERRUPT_CALENDAR_ALARM2          (DL_RTC_COMMON_INTERRUPT_CALENDAR_ALARM2)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERRUPT_PRESCALER0
 */
#define DL_RTC_B_INTERRUPT_PRESCALER0               (DL_RTC_COMMON_INTERRUPT_PRESCALER0)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERRUPT_PRESCALER1
 */
#define DL_RTC_B_INTERRUPT_PRESCALER1               (DL_RTC_COMMON_INTERRUPT_PRESCALER1)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERRUPT_INTERVAL_ALARM
 */
#define DL_RTC_B_INTERRUPT_INTERVAL_ALARM           (DL_RTC_COMMON_INTERRUPT_INTERVAL_ALARM)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERRUPT_READY
 */
#define DL_RTC_B_INTERRUPT_READY                    (DL_RTC_COMMON_INTERRUPT_READY)

/** @}*/

/** @addtogroup DL_RTC_B_EVENT
 *  @{
 */

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_EVENT_CALENDAR_ALARM1
 */
#define DL_RTC_B_EVENT_CALENDAR_ALARM1          (DL_RTC_COMMON_EVENT_CALENDAR_ALARM1)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_EVENT_CALENDAR_ALARM2
 */
#define DL_RTC_B_EVENT_CALENDAR_ALARM2          (DL_RTC_COMMON_EVENT_CALENDAR_ALARM2)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_EVENT_PRESCALER0
 */
#define DL_RTC_B_EVENT_PRESCALER0               (DL_RTC_COMMON_EVENT_PRESCALER0)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_EVENT_PRESCALER1
 */
#define DL_RTC_B_EVENT_PRESCALER1               (DL_RTC_COMMON_EVENT_PRESCALER1)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_EVENT_INTERVAL_ALARM
 */
#define DL_RTC_B_EVENT_INTERVAL_ALARM           (DL_RTC_COMMON_EVENT_INTERVAL_ALARM)

/**
 * @brief Redirects to common @ref DL_RTC_COMMON_EVENT_READY
 */
#define DL_RTC_B_EVENT_READY                    (DL_RTC_COMMON_EVENT_READY)

/** @}*/

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_FORMAT_BINARY
 */
#define DL_RTC_B_FORMAT_BINARY                      DL_RTC_COMMON_FORMAT_BINARY

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_FORMAT_BCD
 */
#define DL_RTC_B_FORMAT_BCD                         DL_RTC_COMMON_FORMAT_BCD

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_STATUS_READY
 */
#define DL_RTC_B_STATUS_READY                       DL_RTC_COMMON_STATUS_READY

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_STATUS_NOT_READY
 */
#define DL_RTC_B_STATUS_NOT_READY                   DL_RTC_COMMON_STATUS_NOT_READY

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_COMPENSATION_STATUS_READY
 */
#define DL_RTC_B_COMPENSATION_STATUS_READY          DL_RTC_COMMON_COMPENSATION_STATUS_READY

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_COMPENSATION_STATUS_NOT_READY
 */
#define DL_RTC_B_COMPENSATION_STATUS_NOT_READY      DL_RTC_COMMON_COMPENSATION_STATUS_NOT_READY

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_COMPENSATION_WRITE_RESULT_OK
 */
#define DL_RTC_B_COMPENSATION_WRITE_RESULT_OK       DL_RTC_COMMON_COMPENSATION_WRITE_RESULT_OK

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_COMPENSATION_WRITE_RESULT_NOT_OK
 */
#define DL_RTC_B_COMPENSATION_WRITE_RESULT_NOT_OK   DL_RTC_COMMON_COMPENSATION_WRITE_RESULT_NOT_OK

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_OFFSET_CALIBRATION_SIGN_DOWN
 */
#define DL_RTC_B_OFFSET_CALIBRATION_SIGN_DOWN       DL_RTC_COMMON_OFFSET_CALIBRATION_SIGN_DOWN

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_OFFSET_CALIBRATION_SIGN_UP
 */
#define DL_RTC_B_OFFSET_CALIBRATION_SIGN_UP         DL_RTC_COMMON_OFFSET_CALIBRATION_SIGN_UP

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_OFFSET_CALIBRATION_FREQUENCY_512
 */
#define DL_RTC_B_OFFSET_CALIBRATION_FREQUENCY_512              DL_RTC_COMMON_OFFSET_CALIBRATION_FREQUENCY_512

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_OFFSET_CALIBRATION_FREQUENCY_256
 */
#define DL_RTC_B_OFFSET_CALIBRATION_FREQUENCY_256              DL_RTC_COMMON_OFFSET_CALIBRATION_FREQUENCY_256

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_OFFSET_CALIBRATION_FREQUENCY_1
 */
#define DL_RTC_B_OFFSET_CALIBRATION_FREQUENCY_1              DL_RTC_COMMON_OFFSET_CALIBRATION_FREQUENCY_1

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_TEMP_CALIBRATION_DOWN1PPM
 */
#define DL_RTC_B_TEMP_CALIBRATION_DOWN1PPM                  DL_RTC_COMMON_TEMP_CALIBRATION_DOWN1PPM

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_TEMP_CALIBRATION_UP1PPM
 */
#define DL_RTC_B_TEMP_CALIBRATION_UP1PPM                    DL_RTC_COMMON_TEMP_CALIBRATION_UP1PPM

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERVAL_ALARM_MINUTECHANGE
 */
#define DL_RTC_B_INTERVAL_ALARM_MINUTECHANGE                DL_RTC_COMMON_INTERVAL_ALARM_MINUTECHANGE

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERVAL_ALARM_HOURCHANGE
 */
#define DL_RTC_B_INTERVAL_ALARM_HOURCHANGE                  DL_RTC_COMMON_INTERVAL_ALARM_HOURCHANGE

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERVAL_ALARM_NOON
 */
#define DL_RTC_B_INTERVAL_ALARM_NOON                        DL_RTC_COMMON_INTERVAL_ALARM_NOON

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_INTERVAL_ALARM_MIDNIGHT
 */
#define DL_RTC_B_INTERVAL_ALARM_MIDNIGHT                    DL_RTC_COMMON_INTERVAL_ALARM_MIDNIGHT

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER0_DIVIDE_8
 */
#define DL_RTC_B_PRESCALER0_DIVIDE_8                        DL_RTC_COMMON_PRESCALER0_DIVIDE_8

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER0_DIVIDE_16
 */
#define DL_RTC_B_PRESCALER0_DIVIDE_16                       DL_RTC_COMMON_PRESCaLER0_DIVIDE_16

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER0_DIVIDE_32
 */
#define DL_RTC_B_PRESCALER0_DIVIDE_32                       DL_RTC_COMMON_PRESCALER0_DIVIDE_32

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER0_DIVIDE_64
 */
#define DL_RTC_B_PRESCALER0_DIVIDE_64                       DL_RTC_COMMON_PRESCALER0_DIVIDE_64

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER0_DIVIDE_128
 */
#define DL_RTC_B_PRESCALER0_DIVIDE_128                      DL_RTC_COMMON_PRESCALER0_DIVIDE_128

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER0_DIVIDE_256
 */
#define DL_RTC_B_PRESCALER0_DIVIDE_256                      DL_RTC_COMMON_PRESCALER0_DIVIDE_256

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_2
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_2                        DL_RTC_COMMON_PRESCALER1_DIVIDE_2

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_4
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_4                        DL_RTC_COMMON_PRESCALER1_DIVIDE_4

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_8
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_8                        DL_RTC_COMMON_PRESCALER1_DIVIDE_8

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_16
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_16                       DL_RTC_COMMON_PRESCALER1_DIVIDE_16

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_32
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_32                       DL_RTC_COMMON_PRESCALER1_DIVIDE_32

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_64
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_64                       DL_RTC_COMMON_PRESCALER1_DIVIDE_64

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_128
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_128                      DL_RTC_COMMON_PRESCALER1_DIVIDE_128

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_PRESCALER1_DIVIDE_256
 */
#define DL_RTC_B_PRESCALER1_DIVIDE_256                      DL_RTC_COMMON_PRESCALER1_DIVIDE_256

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_IIDX_ALARM1
 */
#define DL_RTC_B_IIDX_ALARM1                                DL_RTC_COMMON_IIDX_ALARM1

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_IIDX_ALARM2
 */
#define DL_RTC_B_IIDX_ALARM2                                DL_RTC_COMMON_IIDX_ALARM2

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_IIDX_PRESCALER0
 */
#define DL_RTC_B_IIDX_PRESCALER0                            DL_RTC_COMMON_IIDX_PRESCALER0

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_IIDX_PRESCALER1
 */
#define DL_RTC_B_IIDX_PRESCALER1                            DL_RTC_COMMON_IIDX_PRESCALER1

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_IIDX_INTERVAL_TIMER
 */
#define DL_RTC_B_IIDX_INTERVAL_TIMER                        DL_RTC_COMMON_IIDX_INTERVAL_TIMER

/*!
 * @brief Redirects to common @ref DL_RTC_COMMON_IIDX_READY
 */
#define DL_RTC_B_IIDX_READY                                 DL_RTC_COMMON_IIDX_READY

/**
 * @brief Redirects to common @ref DL_RTC_Common_Calendar
 *
 */
typedef DL_RTC_Common_Calendar                      DL_RTC_B_Calendar;

/**
 * @brief Redirects to common @ref DL_RTC_Common_CalendarAlarm
 *
 */
typedef DL_RTC_Common_CalendarAlarm                 DL_RTC_B_CalendarAlarm;


/*!
 * @brief Redirects to common @ref DL_RTC_Common_isClockSourceLFCLK
 */
#define DL_RTC_B_isClockSourceLFCLK                 DL_RTC_Common_isClockSourceLFCLK

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableClockControl
 */
#define DL_RTC_B_enableClockControl                 DL_RTC_Common_enableClockControl

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableClockControl
 */
#define DL_RTC_B_disableClockControl                DL_RTC_Common_disableClockControl

/*!
 * @brief Redirects to common @ref DL_RTC_Common_reset
 */
#define DL_RTC_B_reset                              DL_RTC_Common_reset

/*!
 * @brief Redirects to common @ref DL_RTC_Common_isReset
 */
#define DL_RTC_B_isReset                            DL_RTC_Common_isReset

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableDebugInterrupts
 */
#define DL_RTC_B_enableDebugInterrupts              DL_RTC_Common_enableDebugInterrupts

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableDebugInterrupts
 */
#define DL_RTC_B_disableDebugInterrupts             DL_RTC_Common_disableDebugInterrupts

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableDebugMode
 */
#define DL_RTC_B_enableDebugMode                    DL_RTC_Common_enableDebugMode

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableDebugMode
 */
#define DL_RTC_B_disableDebugMode                   DL_RTC_Common_disableDebugMode

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setClockFormat
 */
#define DL_RTC_B_setClockFormat                     DL_RTC_Common_setClockFormat

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getClockFormat
 */
#define DL_RTC_B_getClockFormat                     DL_RTC_Common_getClockFormat

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setIntervalAlarm
 */
#define DL_RTC_B_setIntervalAlarm                   DL_RTC_Common_setIntervalAlarm

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getIntervalAlarm
 */
#define DL_RTC_B_getIntervalAlarm                   DL_RTC_Common_getIntervalAlarm

/*!
 * @brief Redirects to common @ref DL_RTC_Common_isSafeToRead
 */
#define DL_RTC_B_isSafeToRead                       DL_RTC_Common_isSafeToRead

/*!
 * @brief Redirects to common @ref DL_RTC_Common_isReadyToCalibrate
 */
#define DL_RTC_B_isReadyToCalibrate                 DL_RTC_Common_isReadyToCalibrate

/*!
 * @brief Redirects to common @ref DL_RTC_Common_isCalibrationWriteResultOK
 */
#define DL_RTC_B_isCalibrationWriteResultOK         DL_RTC_Common_isCalibrationWriteResultOK

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setOffsetCalibrationFrequency
 */
#define DL_RTC_B_setOffsetCalibrationFrequency      DL_RTC_Common_setOffsetCalibrationFrequency

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setOffsetCalibrationAdjValue
 */
#define DL_RTC_B_setOffsetCalibrationAdjValue       DL_RTC_Common_setOffsetCalibrationAdjValue

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setOffsetCalibration
 */
#define DL_RTC_B_setOffsetCalibration               DL_RTC_Common_setOffsetCalibration

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getOffsetCalibrationFrequency
 */
#define DL_RTC_B_getOffsetCalibrationFrequency      DL_RTC_Common_getOffsetCalibrationFrequency

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getOffsetCalibrationSign
 */
#define DL_RTC_B_getOffsetCalibrationSign           DL_RTC_Common_getOffsetCalibrationSign

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setTemperatureCompensation
 */
#define DL_RTC_B_setTemperatureCompensation         DL_RTC_Common_setTemperatureCompensation

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTemperatureCompensation
 */
#define DL_RTC_B_getTemperatureCompensation         DL_RTC_Common_getTemperatureCompensation

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarSecondsBCD
 */
#define DL_RTC_B_setCalendarSecondsBCD              DL_RTC_Common_setCalendarSecondsBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarSecondsBCD
 */
#define DL_RTC_B_getCalendarSecondsBCD              DL_RTC_Common_getCalendarSecondsBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarMinutesBCD
 */
#define DL_RTC_B_setCalendarMinutesBCD              DL_RTC_Common_setCalendarMinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarMinutesBCD
 */
#define DL_RTC_B_getCalendarMinutesBCD              DL_RTC_Common_getCalendarMinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarHoursBCD
 */
#define DL_RTC_B_setCalendarHoursBCD                DL_RTC_Common_setCalendarHoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarHoursBCD
 */
#define DL_RTC_B_getCalendarHoursBCD                DL_RTC_Common_getCalendarHoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarDayOfWeekBCD
 */
#define DL_RTC_B_setCalendarDayOfWeekBCD            DL_RTC_Common_setCalendarDayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarDayOfWeekBCD
 */
#define DL_RTC_B_getCalendarDayOfWeekBCD            DL_RTC_Common_getCalendarDayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarDayOfMonthBCD
 */
#define DL_RTC_B_setCalendarDayOfMonthBCD           DL_RTC_Common_setCalendarDayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarDayOfMonthBCD
 */
#define DL_RTC_B_getCalendarDayOfMonthBCD           DL_RTC_Common_getCalendarDayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarMonthBCD
 */
#define DL_RTC_B_setCalendarMonthBCD                DL_RTC_Common_setCalendarMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarMonthBCD
 */
#define DL_RTC_B_getCalendarMonthBCD                DL_RTC_Common_getCalendarMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarYearBCD
 */
#define DL_RTC_B_setCalendarYearBCD                 DL_RTC_Common_setCalendarYearBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarYearBCD
 */
#define DL_RTC_B_getCalendarYearBCD                 DL_RTC_Common_getCalendarYearBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1MinutesBCD
 */
#define DL_RTC_B_enableAlarm1MinutesBCD             DL_RTC_Common_enableAlarm1MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1MinutesBCD
 */
#define DL_RTC_B_disableAlarm1MinutesBCD            DL_RTC_Common_disableAlarm1MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1MinutesBCD
 */
#define DL_RTC_B_setAlarm1MinutesBCD                DL_RTC_Common_setAlarm1MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1MinutesBCD
 */
#define DL_RTC_B_getAlarm1MinutesBCD                DL_RTC_Common_getAlarm1MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1HoursBCD
 */
#define DL_RTC_B_enableAlarm1HoursBCD               DL_RTC_Common_enableAlarm1HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1HoursBCD
 */
#define DL_RTC_B_disableAlarm1HoursBCD              DL_RTC_Common_disableAlarm1HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1HoursBCD
 */
#define DL_RTC_B_setAlarm1HoursBCD                  DL_RTC_Common_setAlarm1HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1HoursBCD
 */
#define DL_RTC_B_getAlarm1HoursBCD                  DL_RTC_Common_getAlarm1HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1DayOfWeekBCD
 */
#define DL_RTC_B_enableAlarm1DayOfWeekBCD           DL_RTC_Common_enableAlarm1DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1DayOfWeekBCD
 */
#define DL_RTC_B_disableAlarm1DayOfWeekBCD          DL_RTC_Common_disableAlarm1DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1DayOfWeekBCD
 */
#define DL_RTC_B_setAlarm1DayOfWeekBCD              DL_RTC_Common_setAlarm1DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1DayOfWeekBCD
 */
#define DL_RTC_B_getAlarm1DayOfWeekBCD              DL_RTC_Common_getAlarm1DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1DayOfMonthBCD
 */
#define DL_RTC_B_enableAlarm1DayOfMonthBCD          DL_RTC_Common_enableAlarm1DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1DayOfMonthBCD
 */
#define DL_RTC_B_disableAlarm1DayOfMonthBCD         DL_RTC_Common_disableAlarm1DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1DayOfMonthBCD
 */
#define DL_RTC_B_setAlarm1DayOfMonthBCD             DL_RTC_Common_setAlarm1DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1DayOfMonthBCD
 */
#define DL_RTC_B_getAlarm1DayOfMonthBCD             DL_RTC_Common_getAlarm1DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2MinutesBCD
 */
#define DL_RTC_B_enableAlarm2MinutesBCD             DL_RTC_Common_enableAlarm2MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2MinutesBCD
 */
#define DL_RTC_B_disableAlarm2MinutesBCD            DL_RTC_Common_disableAlarm2MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2MinutesBCD
 */
#define DL_RTC_B_setAlarm2MinutesBCD                DL_RTC_Common_setAlarm2MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2MinutesBCD
 */
#define DL_RTC_B_getAlarm2MinutesBCD                DL_RTC_Common_getAlarm2MinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2HoursBCD
 */
#define DL_RTC_B_enableAlarm2HoursBCD               DL_RTC_Common_enableAlarm2HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2HoursBCD
 */
#define DL_RTC_B_disableAlarm2HoursBCD              DL_RTC_Common_disableAlarm2HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2HoursBCD
 */
#define DL_RTC_B_setAlarm2HoursBCD                  DL_RTC_Common_setAlarm2HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2HoursBCD
 */
#define DL_RTC_B_getAlarm2HoursBCD                  DL_RTC_Common_getAlarm2HoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2DayOfWeekBCD
 */
#define DL_RTC_B_enableAlarm2DayOfWeekBCD           DL_RTC_Common_enableAlarm2DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2DayOfWeekBCD
 */
#define DL_RTC_B_disableAlarm2DayOfWeekBCD          DL_RTC_Common_disableAlarm2DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2DayOfWeekBCD
 */
#define DL_RTC_B_setAlarm2DayOfWeekBCD              DL_RTC_Common_setAlarm2DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2DayOfWeekBCD
 */
#define DL_RTC_B_getAlarm2DayOfWeekBCD              DL_RTC_Common_getAlarm2DayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2DayOfMonthBCD
 */
#define DL_RTC_B_enableAlarm2DayOfMonthBCD          DL_RTC_Common_enableAlarm2DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2DayOfMonthBCD
 */
#define DL_RTC_B_disableAlarm2DayOfMonthBCD         DL_RTC_Common_disableAlarm2DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2DayOfMonthBCD
 */
#define DL_RTC_B_setAlarm2DayOfMonthBCD             DL_RTC_Common_setAlarm2DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2DayOfMonthBCD
 */
#define DL_RTC_B_getAlarm2DayOfMonthBCD             DL_RTC_Common_getAlarm2DayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarSecondsBinary
 */
#define DL_RTC_B_setCalendarSecondsBinary           DL_RTC_Common_setCalendarSecondsBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarSecondsBinary
 */
#define DL_RTC_B_getCalendarSecondsBinary           DL_RTC_Common_getCalendarSecondsBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarMinutesBinary
 */
#define DL_RTC_B_setCalendarMinutesBinary           DL_RTC_Common_setCalendarMinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarMinutesBinary
 */
#define DL_RTC_B_getCalendarMinutesBinary           DL_RTC_Common_getCalendarMinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarHoursBinary
 */
#define DL_RTC_B_setCalendarHoursBinary             DL_RTC_Common_setCalendarHoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarHoursBinary
 */
#define DL_RTC_B_getCalendarHoursBinary             DL_RTC_Common_getCalendarHoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarDayOfWeekBinary
 */
#define DL_RTC_B_setCalendarDayOfWeekBinary         DL_RTC_Common_setCalendarDayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarDayOfWeekBinary
 */
#define DL_RTC_B_getCalendarDayOfWeekBinary         DL_RTC_Common_getCalendarDayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarDayOfMonthBinary
 */
#define DL_RTC_B_setCalendarDayOfMonthBinary        DL_RTC_Common_setCalendarDayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarDayOfMonthBinary
 */
#define DL_RTC_B_getCalendarDayOfMonthBinary        DL_RTC_Common_getCalendarDayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarMonthBinary
 */
#define DL_RTC_B_setCalendarMonthBinary             DL_RTC_Common_setCalendarMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarMonthBinary
 */
#define DL_RTC_B_getCalendarMonthBinary             DL_RTC_Common_getCalendarMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarYearBinary
 */
#define DL_RTC_B_setCalendarYearBinary              DL_RTC_Common_setCalendarYearBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarYearBinary
 */
#define DL_RTC_B_getCalendarYearBinary              DL_RTC_Common_getCalendarYearBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1MinutesBinary
 */
#define DL_RTC_B_enableAlarm1MinutesBinary          DL_RTC_Common_enableAlarm1MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1MinutesBinary
 */
#define DL_RTC_B_disableAlarm1MinutesBinary         DL_RTC_Common_disableAlarm1MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1MinutesBinary
 */
#define DL_RTC_B_setAlarm1MinutesBinary             DL_RTC_Common_setAlarm1MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1MinutesBinary
 */
#define DL_RTC_B_getAlarm1MinutesBinary             DL_RTC_Common_getAlarm1MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1HoursBinary
 */
#define DL_RTC_B_enableAlarm1HoursBinary            DL_RTC_Common_enableAlarm1HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1HoursBinary
 */
#define DL_RTC_B_disableAlarm1HoursBinary           DL_RTC_Common_disableAlarm1HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1HoursBinary
 */
#define DL_RTC_B_setAlarm1HoursBinary               DL_RTC_Common_setAlarm1HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1HoursBinary
 */
#define DL_RTC_B_getAlarm1HoursBinary               DL_RTC_Common_getAlarm1HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1DayOfWeekBinary
 */
#define DL_RTC_B_enableAlarm1DayOfWeekBinary        DL_RTC_Common_enableAlarm1DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1DayOfWeekBinary
 */
#define DL_RTC_B_disableAlarm1DayOfWeekBinary       DL_RTC_Common_disableAlarm1DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1DayOfWeekBinary
 */
#define DL_RTC_B_setAlarm1DayOfWeekBinary           DL_RTC_Common_setAlarm1DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1DayOfWeekBinary
 */
#define DL_RTC_B_getAlarm1DayOfWeekBinary           DL_RTC_Common_getAlarm1DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm1DayOfMonthBinary
 */
#define DL_RTC_B_enableAlarm1DayOfMonthBinary       DL_RTC_Common_enableAlarm1DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm1DayOfMonthBinary
 */
#define DL_RTC_B_disableAlarm1DayOfMonthBinary      DL_RTC_Common_disableAlarm1DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm1DayOfMonthBinary
 */
#define DL_RTC_B_setAlarm1DayOfMonthBinary          DL_RTC_Common_setAlarm1DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm1DayOfMonthBinary
 */
#define DL_RTC_B_getAlarm1DayOfMonthBinary          DL_RTC_Common_getAlarm1DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2MinutesBinary
 */
#define DL_RTC_B_enableAlarm2MinutesBinary          DL_RTC_Common_enableAlarm2MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2MinutesBinary
 */
#define DL_RTC_B_disableAlarm2MinutesBinary         DL_RTC_Common_disableAlarm2MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2MinutesBinary
 */
#define DL_RTC_B_setAlarm2MinutesBinary             DL_RTC_Common_setAlarm2MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2MinutesBinary
 */
#define DL_RTC_B_getAlarm2MinutesBinary             DL_RTC_Common_getAlarm2MinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2HoursBinary
 */
#define DL_RTC_B_enableAlarm2HoursBinary            DL_RTC_Common_enableAlarm2HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2HoursBinary
 */
#define DL_RTC_B_disableAlarm2HoursBinary           DL_RTC_Common_disableAlarm2HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2HoursBinary
 */
#define DL_RTC_B_setAlarm2HoursBinary               DL_RTC_Common_setAlarm2HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2HoursBinary
 */
#define DL_RTC_B_getAlarm2HoursBinary               DL_RTC_Common_getAlarm2HoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2DayOfWeekBinary
 */
#define DL_RTC_B_enableAlarm2DayOfWeekBinary        DL_RTC_Common_enableAlarm2DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2DayOfWeekBinary
 */
#define DL_RTC_B_disableAlarm2DayOfWeekBinary       DL_RTC_Common_disableAlarm2DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2DayOfWeekBinary
 */
#define DL_RTC_B_setAlarm2DayOfWeekBinary           DL_RTC_Common_setAlarm2DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2DayOfWeekBinary
 */
#define DL_RTC_B_getAlarm2DayOfWeekBinary           DL_RTC_Common_getAlarm2DayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableAlarm2DayOfMonthBinary
 */
#define DL_RTC_B_enableAlarm2DayOfMonthBinary       DL_RTC_Common_enableAlarm2DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableAlarm2DayOfMonthBinary
 */
#define DL_RTC_B_disableAlarm2DayOfMonthBinary      DL_RTC_Common_disableAlarm2DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setAlarm2DayOfMonthBinary
 */
#define DL_RTC_B_setAlarm2DayOfMonthBinary          DL_RTC_Common_setAlarm2DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getAlarm2DayOfMonthBinary
 */
#define DL_RTC_B_getAlarm2DayOfMonthBinary          DL_RTC_Common_getAlarm2DayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setPrescalerEvents
 */
#define DL_RTC_B_setPrescalerEvents                 DL_RTC_Common_setPrescalerEvents

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setPeriodicAlarm0
 */
#define DL_RTC_B_setPeriodicAlarm0                  DL_RTC_Common_setPeriodicAlarm0

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setPeriodicAlarm1
 */
#define DL_RTC_B_setPeriodicAlarm1                  DL_RTC_Common_setPeriodicAlarm1

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setPeriodicAlarm2
 */
#define DL_RTC_B_setPeriodicAlarm2                  DL_RTC_Common_setPeriodicAlarm2

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getPrescaler0
 */
#define DL_RTC_B_getPrescaler0                      DL_RTC_Common_getPrescaler0

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getPrescaler1
 */
#define DL_RTC_B_getPrescaler1                      DL_RTC_Common_getPrescaler1

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampSecondsBCD
 */
#define DL_RTC_B_getTimeStampSecondsBCD             DL_RTC_Common_getTimeStampSecondsBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampMinutesBCD
 */
#define DL_RTC_B_getTimeStampMinutesBCD             DL_RTC_Common_getTimeStampMinutesBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampHoursBCD
 */
#define DL_RTC_B_getTimeStampHoursBCD               DL_RTC_Common_getTimeStampHoursBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampDayOfWeekBCD
 */
#define DL_RTC_B_getTimeStampDayOfWeekBCD           DL_RTC_Common_getTimeStampDayOfWeekBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampDayOfMonthBCD
 */
#define DL_RTC_B_getTimeStampDayOfMonthBCD          DL_RTC_Common_getTimeStampDayOfMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampMonthBCD
 */
#define DL_RTC_B_getTimeStampMonthBCD               DL_RTC_Common_getTimeStampMonthBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampYearBCD
 */
#define DL_RTC_B_getTimeStampYearBCD                DL_RTC_Common_getTimeStampYearBCD

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampSecondsBinary
 */
#define DL_RTC_B_getTimeStampSecondsBinary          DL_RTC_Common_getTimeStampSecondsBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampMinutesBinary
 */
#define DL_RTC_B_getTimeStampMinutesBinary          DL_RTC_Common_getTimeStampMinutesBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampHoursBinary
 */
#define DL_RTC_B_getTimeStampHoursBinary            DL_RTC_Common_getTimeStampHoursBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampDayOfWeekBinary
 */
#define DL_RTC_B_getTimeStampDayOfWeekBinary        DL_RTC_Common_getTimeStampDayOfWeekBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampDayOfMonthBinary
 */
#define DL_RTC_B_getTimeStampDayOfMonthBinary       DL_RTC_Common_getTimeStampDayOfMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampMonthBinary
 */
#define DL_RTC_B_getTimeStampMonthBinary            DL_RTC_Common_getTimeStampMonthBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getTimeStampYearBinary
 */
#define DL_RTC_B_getTimeStampYearBinary             DL_RTC_Common_getTimeStampYearBinary

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableInterrupt
 */
#define DL_RTC_B_enableInterrupt                    DL_RTC_Common_enableInterrupt

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableInterrupt
 */
#define DL_RTC_B_disableInterrupt                   DL_RTC_Common_disableInterrupt

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getEnabledInterrupts
 */
#define DL_RTC_B_getEnabledInterrupts               DL_RTC_Common_getEnabledInterrupts

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getEnabledInterruptStatus
 */
#define DL_RTC_B_getEnabledInterruptStatus          DL_RTC_Common_getEnabledInterruptStatus

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getRawInterruptStatus
 */
#define DL_RTC_B_getRawInterruptStatus              DL_RTC_Common_getRawInterruptStatus

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getPendingInterrupt
 */
#define DL_RTC_B_getPendingInterrupt                DL_RTC_Common_getPendingInterrupt

/*!
 * @brief Redirects to common @ref DL_RTC_Common_clearInterruptStatus
 */
#define DL_RTC_B_clearInterruptStatus               DL_RTC_Common_clearInterruptStatus

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableEvent
 */
#define DL_RTC_B_enableEvent                        DL_RTC_Common_enableEvent

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableEvent
 */
#define DL_RTC_B_disableEvent                       DL_RTC_Common_disableEvent

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getEnabledEvents
 */
#define DL_RTC_B_getEnabledEvents                   DL_RTC_Common_getEnabledEvents

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getEnabledEventStatus
 */
#define DL_RTC_B_getEnabledEventStatus              DL_RTC_Common_getEnabledEventStatus

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getRawEventsStatus
 */
#define DL_RTC_B_getRawEventsStatus                 DL_RTC_Common_getRawEventsStatus

/*!
 * @brief Redirects to common @ref DL_RTC_Common_clearEventsStatus
 */
#define DL_RTC_B_clearEventsStatus                  DL_RTC_Common_clearEventsStatus

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setPublisherChanID
 */
#define DL_RTC_B_setPublisherChanID                 DL_RTC_Common_setPublisherChanID

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getPublisherChanID
 */
#define DL_RTC_B_getPublisherChanID                 DL_RTC_Common_getPublisherChanID

/*!
 * @brief Redirects to common @ref DL_RTC_Common_initCalendar
 */
#define DL_RTC_B_initCalendar                       DL_RTC_Common_initCalendar

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarTime
 */
#define DL_RTC_B_getCalendarTime                    DL_RTC_Common_getCalendarTime

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarAlarm1
 */
#define DL_RTC_B_setCalendarAlarm1                  DL_RTC_Common_setCalendarAlarm1

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarAlarm1
 */
#define DL_RTC_B_getCalendarAlarm1                  DL_RTC_Common_getCalendarAlarm1

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableCalendarAlarm1
 */
#define DL_RTC_B_enableCalendarAlarm1               DL_RTC_Common_enableCalendarAlarm1

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableCalendarAlarm1
 */
#define DL_RTC_B_disableCalendarAlarm1              DL_RTC_Common_disableCalendarAlarm1

/*!
 * @brief Redirects to common @ref DL_RTC_Common_setCalendarAlarm2
 */
#define DL_RTC_B_setCalendarAlarm2                  DL_RTC_Common_setCalendarAlarm2

/*!
 * @brief Redirects to common @ref DL_RTC_Common_getCalendarAlarm2
 */
#define DL_RTC_B_getCalendarAlarm2                  DL_RTC_Common_getCalendarAlarm2

/*!
 * @brief Redirects to common @ref DL_RTC_Common_enableCalendarAlarm2
 */
#define DL_RTC_B_enableCalendarAlarm2               DL_RTC_Common_enableCalendarAlarm2

/*!
 * @brief Redirects to common @ref DL_RTC_Common_disableCalendarAlarm2
 */
#define DL_RTC_B_disableCalendarAlarm2              DL_RTC_Common_disableCalendarAlarm2

#ifdef __cplusplus
}
#endif

#endif /* __MSPM0_HAS_RTC_B__ */

#endif /* ti_dl_dl_rtc_b__include */
/** @}*/
