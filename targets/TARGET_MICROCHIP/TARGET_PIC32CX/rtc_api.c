/***************************************************************************//**
* @file rtc_api.c
*******************************************************************************
* @section License
* <b>Copyright (c) 2021 Microchip Technology Inc. and its subsidiaries.</b>
*******************************************************************************
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************/

#if DEVICE_RTC
#include <time.h>
#include "pic32cx.h"
#include "rtc_api.h"
#include "pinmap.h"
//#include "rtc.h"
#include "PeripheralPins.h"
#include "pmc.h"

/* RTC Write Protect Key "RTC" in ASCII */
#define RTC_WP_KEY     (0x525443)

/* The BCD code shift value */
#define BCD_SHIFT      4

/* The BCD code mask value */
#define BCD_MASK       0xfu

/* The BCD mul/div factor value */
#define BCD_FACTOR     10

static bool rtc_time_set = false;

void rtc_init(void)
{
    /* Check external clock */
    while (!pmc_osc_is_ready_32kxtal()) {
        pmc_switch_sclk_to_32kxtal(0);
    }

    /* Default RTC configuration, 24-hour mode */
	RTC->RTC_MR &= (~RTC_MR_HRMOD);
}

void rtc_free(void)
{
    /* None to do */
}

/*
 * Little check routine to see if the RTC has been enabled
 * 0 = Disabled, 1 = Enabled
 */
int rtc_isenabled(void)
{
    bool rtc_free_run;

    rtc_free_run = ((RTC->RTC_SR & RTC_SR_ACKUPD) == RTC_SR_ACKUPD_FREERUN);

    return (rtc_free_run & rtc_time_set);
}

time_t rtc_read(void)
{
    struct tm t;
    time_t t_of_day;
	uint32_t ul_reg;
	uint32_t ul_cent;
	uint32_t ul_temp;

	/* Get the current RTC time (multiple reads are necessary to insure a stable value). */
	ul_reg = RTC->RTC_TIMR;
	while (ul_reg != RTC->RTC_TIMR) {
		ul_reg = RTC->RTC_TIMR;
	}

	/* Hour */
	ul_temp = (ul_reg & RTC_TIMR_HOUR_Msk) >> RTC_TIMR_HOUR_Pos;
	t.tm_hour = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);
	if ((ul_reg & RTC_TIMR_AMPM) == RTC_TIMR_AMPM) {
		t.tm_hour += 12;
	}

	/* Minute */
	ul_temp = (ul_reg & RTC_TIMR_MIN_Msk) >> RTC_TIMR_MIN_Pos;
	t.tm_min = (ul_temp >> BCD_SHIFT) * BCD_FACTOR +  (ul_temp & BCD_MASK);

	/* Second */
	ul_temp = (ul_reg & RTC_TIMR_SEC_Msk) >> RTC_TIMR_SEC_Pos;
	t.tm_sec = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	/* Get the current date (multiple reads are necessary to insure a stable value). */
	ul_reg = RTC->RTC_CALR;
	while (ul_reg != RTC->RTC_CALR) {
		ul_reg = RTC->RTC_CALR;
	}

	/* Retrieve year */
	ul_temp = (ul_reg & RTC_CALR_CENT_Msk) >> RTC_CALR_CENT_Pos;
	ul_cent = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);
	ul_temp = (ul_reg & RTC_CALR_YEAR_Msk) >> RTC_CALR_YEAR_Pos;
	t.tm_year = (ul_cent * BCD_FACTOR * BCD_FACTOR) +
			(ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);
	t.tm_year -= 1900; /* Year - 1900 */

	/* Retrieve month */
	ul_temp = (ul_reg & RTC_CALR_MONTH_Msk) >> RTC_CALR_MONTH_Pos;
	t.tm_mon = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);
	t.tm_mon--;  /* Month 0 = Jan */

	/* Retrieve day */
	ul_temp = (ul_reg & RTC_CALR_DATE_Msk) >> RTC_CALR_DATE_Pos;
	t.tm_mday = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	/* Retrieve week */
	t.tm_wday = ((ul_reg & RTC_CALR_DAY_Msk) >> RTC_CALR_DAY_Pos);

	// Is DST on? 1 = yes, 0 = no, -1 = unknown
    t.tm_isdst = -1;

    /* Get Epoch Time */
    t_of_day = mktime(&t);

    return t_of_day;
}

void rtc_write(time_t t)
{
    struct tm ts;
	uint32_t ul_reg;
	uint32_t ul_temp;

    /* Get Human-readable date */
    ts = *localtime(&t);

	/* Stops the RTC time/date counting */
	RTC->RTC_CR |= (RTC_CR_UPDTIM | RTC_CR_UPDCAL);

	/* Wait ACK Update flag */
	ul_temp = 0xFFFF;
	while ((RTC->RTC_SR & RTC_SR_ACKUPD) != RTC_SR_ACKUPD) {
		if ((ul_temp--) == 0) {
			break;
		}
	}

	/* Acknowledge Clear */
	RTC->RTC_SCCR = RTC_SCCR_ACKCLR;

	/* Hour */
	ul_reg = ((ts.tm_hour / BCD_FACTOR) << (RTC_TIMR_HOUR_Pos + BCD_SHIFT)) |
			((ts.tm_hour % BCD_FACTOR) << RTC_TIMR_HOUR_Pos);

	/* If 12-hour mode, set AMPM bit */
	if ((RTC->RTC_MR & RTC_MR_HRMOD) == RTC_MR_HRMOD) {
		if (ts.tm_hour > 12) {
			ts.tm_hour -= 12;
			ul_reg |= RTC_TIMR_AMPM;
		}
	}

	/* Minute */
	ul_reg |= ((ts.tm_min / BCD_FACTOR) << (RTC_TIMR_MIN_Pos + BCD_SHIFT)) |
			((ts.tm_min % BCD_FACTOR) << RTC_TIMR_MIN_Pos);

	/* Second */
	ul_reg |= ((ts.tm_sec / BCD_FACTOR) << (RTC_TIMR_SEC_Pos + BCD_SHIFT)) |
			((ts.tm_sec % BCD_FACTOR) << RTC_TIMR_SEC_Pos);

	/* Set Time */
	RTC->RTC_TIMR = ul_reg;

	/* Cent */
	ul_temp = ts.tm_year + 1900;
	ul_reg = ((ul_temp / BCD_FACTOR / BCD_FACTOR / BCD_FACTOR) <<
			(RTC_CALR_CENT_Pos + BCD_SHIFT) |
			((ul_temp / BCD_FACTOR / BCD_FACTOR) % BCD_FACTOR) <<  RTC_CALR_CENT_Pos);

	/* Year */
	ul_reg |= (((ul_temp / BCD_FACTOR) % BCD_FACTOR) <<
			(RTC_CALR_YEAR_Pos + BCD_SHIFT)) |
			((ul_temp % BCD_FACTOR) << RTC_CALR_YEAR_Pos);

	/* Month */
	ul_temp = ts.tm_mon + 1;
	ul_reg |= ((ul_temp / BCD_FACTOR) << (RTC_CALR_MONTH_Pos + BCD_SHIFT)) |
			((ul_temp % BCD_FACTOR) << RTC_CALR_MONTH_Pos);

	/* Week */
	ul_reg |= (ts.tm_wday << RTC_CALR_DAY_Pos);

	/* Day */
	ul_reg |= ((ts.tm_mday / BCD_FACTOR) << (RTC_CALR_DATE_Pos + BCD_SHIFT)) |
			((ts.tm_mday % BCD_FACTOR) << RTC_CALR_DATE_Pos);

	/* Set Date */
	RTC->RTC_CALR = ul_reg;

	/* Stops the update procedure */
	RTC->RTC_CR &= ~(RTC_CR_UPDTIM | RTC_CR_UPDCAL);

    /* Set time flag */
    rtc_time_set = true;
}

#endif
