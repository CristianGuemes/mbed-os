/***************************************************************************//**
 * @file objects.h
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

#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include "PinNames.h"
#include "CommonPinNames.h"
#include "PeripheralNames.h"
#include "PortNames.h"
#include "pic32cx.h"
#include "uart_serial.h"
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PinName pin;
    PinMode mode;
    PinDirection dir;
} gpio_t;

#if DEVICE_ANALOGIN
struct analogin_s {
    Adc *adc;
    enum adc_channel_num_t channel;
};
#endif

#if DEVICE_I2C
struct i2c_s {
    Twi *i2c;
    uint32_t location;
};
#endif

#if DEVICE_PORTOUT
struct port_s {
    PortName port;
    uint32_t mask;
    PinDirection dir;
};
#endif

#if DEVICE_PWMOUT
struct pwmout_s {
    uint32_t channel;
    PinName pin;
};
#endif

#if DEVICE_INTERRUPTIN
struct gpio_irq_s {
    PinName pin;
    uint32_t id_pio;
    uint8_t risingEdge;
    uint8_t fallingEdge;
};
#endif

#if DEVICE_SERIAL
struct serial_s {
	Usart *p_usart;
	usart_serial_options_t serial_options;
	bool is_usart;
};
#endif

#if DEVICE_SPI
struct spi_s {
    Spi *spi;
    uint8_t cs;
    bool is_slave;
};
#endif

#if DEVICE_FLASH
struct flash_s {
    Sefc *sfec;
};
#endif

#if DEVICE_TRNG
struct trng_s {
    uint8_t dummy;
};
#endif

#if DEVICE_QSPI
struct qspi_s {
    Qspi0 *qspi;
    PinName io0;
    PinName io1;
    PinName io2;
    PinName io3;
    PinName sclk;
    PinName ssel;
};
#endif

#ifdef __cplusplus
}
#endif

#endif
