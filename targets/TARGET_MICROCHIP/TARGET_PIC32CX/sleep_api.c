/***************************************************************************//**
* @file sleep_api.c
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
#include "device.h"

#if DEVICE_SLEEP

#include "sleep_api.h"
#include "pic32cx.h"
#include "sleep.h"
#include "pmc.h"
#include "supc.h"

/** Send the microcontroller to sleep
*
* The processor is setup ready for sleep, and sent to sleep. In this mode, the
* system clock to the core is stopped until a reset or an interrupt occurs. This eliminates
* dynamic power used by the processor, memory systems and buses. The processor, peripheral and
* memory state are maintained, and the peripherals continue to work and can generate interrupts.
*
* The processor can be woken up by any internal peripheral interrupt or external pin interrupt.
*
* The wake-up time shall be less than 10 us.
*
*/
void hal_sleep(void)
{
    pmc_sleep(PIC32CX_PM_SMODE_SLEEP_WFI);
}

/** Send the microcontroller to deep sleep
*
* This processor is setup ready for deep sleep, and sent to sleep using __WFI(). This mode
* has the same sleep features as sleep plus it powers down peripherals and high frequency clocks.
* All state is still maintained.
*
* The processor can only be woken up by low power ticker, RTC, an external interrupt on a pin or a watchdog timer.
*
* The wake-up time shall be less than 10 ms.
*/
void hal_deepsleep(void)
{
    /* Set wakeup sources */
    pmc_set_fast_startup_input(PMC_FSMR_RTTAL | PMC_FSMR_RTCAL);
    supc_set_backup_mode(SUPC, SUPC_BMR_RTTWKEN | SUPC_BMR_RTCWKEN | SUPC_BMR_FWUPEN);

    /* Enter wait mode */
    pmc_sleep(PIC32CX_PM_SMODE_WAIT);
}

#endif
