/***************************************************************************//**
 * @file analogin_api.c
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

#if DEVICE_ANALOGIN

#include "mbed_assert.h"
#include "analogin_api.h"

#include "pinmap.h"
#include "PeripheralPins.h"

#include "adc.h"

/* Tracking Time*/
#define TRACKING_TIME         1
/* Transfer Period */
#define TRANSFER_PERIOD       1

void analogin_init(analogin_t *obj, PinName pin)
{
    static uint8_t adc_initialized = 0;

    /* Init structure */
    obj->adc = (Adc *) pinmap_peripheral(pin, PinMap_ADC);
    MBED_ASSERT((unsigned int) obj->adc != NC);

    obj->channel = (enum adc_channel_num_t)pinmap_function(pin, PinMap_ADC);
    MBED_ASSERT((unsigned int) obj->channel != NC);

    /* Only initialize the ADC once */
    if (!adc_initialized) {
        pmc_enable_periph_clk(ID_ADC);

        /* Initialize ADC. */
        /* Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
         * For example, MCK = 200MHZ, PRESCAL = 4, then:
         * ADCClock = 200 / ((4+1) * 2) = 20MHz;
         */
        /* Formula:
         *     Startup  Time = startup value / ADCClock
         *     Startup time = 64 / 20MHz = 3.2 us
         */
        adc_init(obj->adc, sysclk_get_cpu_hz(), 6400000, ADC_STARTUP_TIME_4);

        /* Set Internal Voltage Reference */
        adc_set_int_ref_voltage(obj->adc);

        /* Set ADC timing. */
        adc_configure_timing(obj->adc, TRACKING_TIME, TRANSFER_PERIOD);

        /* Configure trigger mode and start convention. */
        adc_set_trigger(obj->adc, ADC_TRGR_TRGMOD_NO_TRIGGER, 0);

        /* Clear Averaging Trigger */
        adc_set_averaging_trigger(obj->adc, false);

        /* Set Resolution bits (OSR) */
        adc_set_resolution(obj->adc, ADC_16_BITS);

        /* Enable TAG */
        adc_enable_tag(obj->adc);

        adc_initialized = 1;
    }
}

uint16_t analogin_read_u16(analogin_t *obj)
{
    uint32_t temp = 0;
    Adc *adc;
    enum adc_channel_num_t uc_ch_num;

    

    /* Enable channel. */
    adc_enable_channel(adc, obj->channel);

    /* Enable Data ready interrupt. */
    adc_enable_interrupt(adc, ADC_IER_DRDY);

    /* Start conversion */
    adc_start(adc);

    /* Wait while conversion is not complete */
    while ((adc_get_status(adc) & ADC_ISR_DRDY) != ADC_ISR_DRDY);

    /* Get ADC result */
    temp = adc_get_latest_value(adc);

    /* Get TAG */
    uc_ch_num = adc_get_tag(adc);

    /* Disable channel. */
    adc_disable_channel(adc, obj->channel);

    if (uc_ch_num != obj->channel) {
        temp = 0;
    }

    return (uint16_t)(temp & ADC_LCDR_LDATA_Msk);
}

float analogin_read(analogin_t *obj)
{
    /* Convert from a uint16 to a float between 0 and 1 by division by 0xFF */
    return analogin_read_u16(obj) / (float) 0xFFFF;
}

const PinMap *analogin_pinmap()
{
    return PinMap_ADC;
}

#endif
