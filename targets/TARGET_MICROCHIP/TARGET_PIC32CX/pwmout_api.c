/***************************************************************************//**
* @file pwmout_api.c
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
#include "clocking.h"

#if DEVICE_PWMOUT

#include "mbed_assert.h"
#include "mbed_power_mgmt.h"
#include "pwmout_api.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "device_peripherals.h"

#include "pic32cx.h"
#include "pwm.h"

/* PWM frequency in Hz */
#define PWM_FREQUENCY           100
/* Default period value of PWM output waveform = 20ms */
#define DEFAULT_PERIOD_VALUE    20000
/** Initial duty cycle value (pulse width = 0) */
#define INIT_DUTY_VALUE         0

pwm_channel_t g_pwm_channel[3];

/** Initialize the pwm out peripheral and configure the pin
*
* @param obj The pwmout object to initialize
* @param pin The pwmout pin to initialize
*/
void pwmout_init(pwmout_t *obj, PinName pin)
{
    obj->channel = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
    obj->pin = pin;
    MBED_ASSERT(obj->channel != (PWMName) NC);

    /* Set Pin Mode */
    ioport_mode_t mode = (ioport_mode_t)pinmap_function(pin, PinMap_PWM);
    ioport_set_pin_mode(pin, mode);

    /* Enable PWM peripheral clock */
    pmc_enable_periph_clk(ID_PWM);

    /* Disable PWM channel */
    pwm_channel_disable(PWM, obj->channel);

    /* Set PWM clock A as PWM_FREQUENCY*DEFAULT_PERIOD_VALUE (clock B is not used) */
    pwm_clock_t clock_setting = {
        .ul_clka = PWM_FREQUENCY * DEFAULT_PERIOD_VALUE,
        .ul_clkb = 0,
        .ul_mck = sysclk_get_peripheral_bus_hz(PWM)
    };

    /* Init PWM clock */
    pwm_init(PWM, &clock_setting);

    /* Initialize PWM channel */
    /* Period is left-aligned */
    g_pwm_channel[obj->channel].alignment = PWM_ALIGN_LEFT;
    /* Output waveform starts at a high level */
    g_pwm_channel[obj->channel].polarity = PWM_HIGH;
    /* Use PWM clock A as source clock */
    g_pwm_channel[obj->channel].ul_prescaler = PWM_CMR_CPRE_CLKA;
    /* Period value of output waveform */
    g_pwm_channel[obj->channel].ul_period = DEFAULT_PERIOD_VALUE;
    /* Duty cycle value of output waveform */
    g_pwm_channel[obj->channel].ul_duty = INIT_DUTY_VALUE;
    g_pwm_channel[obj->channel].channel = obj->channel;
    pwm_channel_init(PWM, &g_pwm_channel[obj->channel]);

    /* Enable PWM channel */
    pwm_channel_enable(PWM, obj->channel);
}

/** Deinitialize the pwmout object
*
* @param obj The pwmout object
*/
void pwmout_free(pwmout_t *obj)
{
    /* Disable PWM channel */
    pwm_channel_disable(PWM, obj->channel);
}

/** Set the output duty-cycle in range <0.0f, 1.0f>
*
* Value 0.0f represents 0 percentage, 1.0f represents 100 percent.
* @param obj     The pwmout object
* @param percent The floating-point percentage number
*/
void pwmout_write(pwmout_t *obj, float value)
{
    uint32_t ul_duty = 0;
    if (value < 0.0f) {
        ul_duty = 0;
    } else if (value >= 1.0f) {
        ul_duty = g_pwm_channel[obj->channel].ul_period;
    } else {
        ul_duty = (uint32_t)((float)g_pwm_channel[obj->channel].ul_period * value);
    }

    /* Duty cycle value of output waveform */
    pwm_channel_update_duty(PWM, &g_pwm_channel[obj->channel], ul_duty);
}

/** Read the current float-point output duty-cycle
*
* @param obj The pwmout object
* @return A floating-point output duty-cycle
*/
float pwmout_read(pwmout_t *obj)
{
    if (g_pwm_channel[obj->channel].ul_duty >= g_pwm_channel[obj->channel].ul_period) {
        return 1.0f;
    } else if (g_pwm_channel[obj->channel].ul_duty == 0) {
        return 0.0f;
    } else {
        return ((float) g_pwm_channel[obj->channel].ul_duty / (float) g_pwm_channel[obj->channel].ul_period);
    }
}

/** Set the PWM period specified in seconds, keeping the duty cycle the same
*
* Periods smaller than microseconds (the lowest resolution) are set to zero.
* @param obj     The pwmout object
* @param seconds The floating-point seconds period
*/
void pwmout_period(pwmout_t *obj, float seconds)
{
    pwmout_period_ms(obj, (int)(seconds * 1000));
}

/** Set the PWM period specified in miliseconds, keeping the duty cycle the same
*
* @param obj The pwmout object
* @param ms  The milisecond period
*/
void pwmout_period_ms(pwmout_t *obj, int ms)
{
    pwmout_period_us(obj, (int)(ms * 1000));
}

/** Set the PWM period specified in microseconds, keeping the duty cycle the same
*
* @param obj The pwmout object
* @param us  The microsecond period
*/
void pwmout_period_us(pwmout_t *obj, int us)
{
    pwm_channel_update_period(PWM, &g_pwm_channel[obj->channel], us);
}

/** Read the PWM period specified in microseconds
*
* @param obj The pwmout object
* @return A int output period
*/
int pwmout_read_period_us(pwmout_t *obj)
{
    return (g_pwm_channel[obj->channel].ul_period * 1000);
}

/** Set the PWM pulsewidth specified in seconds, keeping the period the same.
*
* @param obj     The pwmout object
* @param seconds The floating-point pulsewidth in seconds
*/
void pwmout_pulsewidth(pwmout_t *obj, float seconds)
{
    pwmout_pulsewidth_ms(obj, seconds * 1000);
}

/** Set the PWM pulsewidth specified in miliseconds, keeping the period the same.
*
* @param obj The pwmout object
* @param ms  The floating-point pulsewidth in miliseconds
*/
void pwmout_pulsewidth_ms(pwmout_t *obj, int ms)
{
    pwmout_pulsewidth_us(obj, ms * 1000);
}

/** Set the PWM pulsewidth specified in microseconds, keeping the period the same.
*
* @param obj The pwmout object
* @param us  The floating-point pulsewidth in microseconds
*/
void pwmout_pulsewidth_us(pwmout_t *obj, int us)
{
    pwm_channel_update_duty(PWM, &g_pwm_channel[obj->channel], us);
}

/** Read the PWM pulsewidth specified in microseconds
*
* @param obj The pwmout object
* @return A int output pulsewitdth
*/
int pwmout_read_pulsewidth_us(pwmout_t *obj)
{
    return (g_pwm_channel[obj->channel].ul_duty);
}

/** Get the pins that support PWM
*
* Return a PinMap array of pins that support PWM.
* The array is terminated with {NC, NC, 0}.
*
* @return PinMap array
*/
const PinMap *pwmout_pinmap()
{
    return PinMap_PWM;
}

#endif
