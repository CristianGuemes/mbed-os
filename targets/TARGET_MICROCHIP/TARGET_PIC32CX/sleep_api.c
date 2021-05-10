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
#include "pmc.h"
#include "supc.h"
#include "interrupt_sam_nvic.h"

/** (SCR) Sleep deep bit */
#define SCR_SLEEPDEEP   (0x1 <<  2)

//typedef struct {
//	uint32_t ticks_before;
//	uint32_t ticks_after;
//	uint32_t ticks_diff;
//} ticks_sleep_t;
//ticks_sleep_t ticks_sleep[32] = {0};
//uint32_t idx = 0;

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
	irqflags_t cpu_irq_flags;

	cpu_irq_flags = cpu_irq_save();

	cpu_irq_disable();

	/* Enter in sleep mode */
  	PMC->PMC_FSMR &= (uint32_t) ~PMC_FSMR_LPM;
	SCB->SCR &= (uint32_t) ~SCR_SLEEPDEEP;

	cpu_irq_enable();

	__DSB();

//	ticks_sleep[idx].ticks_before = TC0->TC_CHANNEL[0].TC_CV;
//	PIOB->PIO_SODR = 1 << 20;

	__WFI();

//	ticks_sleep[idx].ticks_after = TC0->TC_CHANNEL[0].TC_CV;
//	PIOB->PIO_CODR = 1 << 20;
//	ticks_sleep[idx].ticks_diff = ticks_sleep[idx].ticks_after - ticks_sleep[idx].ticks_before;
//	idx++;

	cpu_irq_disable();

	cpu_irq_restore(cpu_irq_flags);
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
	irqflags_t cpu_irq_flags;

	cpu_irq_flags = cpu_irq_save();

	cpu_irq_disable();

	/* Select the 12 MHz fast RC Oscillator as Main Clock for the entire system */
	pmc_switch_mainck_to_fastrc();
	pmc_switch_mck_to_mainck(0);

	/* Stop Subsystem 1 */
	pmc_disable_cpck();
	pmc_disable_cpbmck();

	/* Disable PLLs */
	pmc_disable_pll(0);
	pmc_disable_pll(1);
	pmc_disable_pll(2);

	/* Disable the Main Crystal Oscillator */
	pmc_osc_disable_main_xtal();

	/* Enter in sleep mode */
  	PMC->PMC_FSMR &= (uint32_t) ~PMC_FSMR_LPM;
	SCB->SCR &= (uint32_t) ~SCR_SLEEPDEEP;

	cpu_irq_enable();

	__DSB();

//	ticks_sleep[idx].ticks_before = TC0->TC_CHANNEL[0].TC_CV;
	TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_TCCLKS_Msk;
//	PIOB->PIO_SODR = 1 << 20;

	__WFI();


	TC0->TC_CHANNEL[0].TC_CMR &= ~TC_CMR_TCCLKS_Msk;
//	ticks_sleep[idx].ticks_after = TC0->TC_CHANNEL[0].TC_CV;
//	PIOB->PIO_CODR = 1 << 20;
//	ticks_sleep[idx].ticks_diff = ticks_sleep[idx].ticks_after - ticks_sleep[idx].ticks_before;
//	idx++;

	cpu_irq_disable();

	/* Recall SysInit function to restore clock system by default */
	sysclk_init();

	cpu_irq_restore(cpu_irq_flags);
}

#endif
