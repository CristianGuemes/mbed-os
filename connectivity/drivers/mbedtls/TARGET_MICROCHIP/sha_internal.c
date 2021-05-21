/*
*  Hardware-accelerated SHA support functions for Microchip devices
*  containing a SHA peripheral.
*
*  <b>Copyright (c) 2021 Microchip Technology Inc. and its subsidiaries.</b>
*  SPDX-License-Identifier: Apache-2.0
*
*  Licensed under the Apache License, Version 2.0 (the "License"); you may
*  not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
*  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*/

#if defined(MBEDTLS_SHA1_ALT) || defined(MBEDTLS_SHA256_ALT) || defined(MBEDTLS_SHA512_ALT)

#include "sha_internal.h"

static uint32_t sul_current_id = 0;
static uint32_t sul_next_id = 0;

/**
 * \brief Get SHA new identifier.
 *
 * \return Identifier value.
 */
uint32_t sha_internal_get_new_id(void)
{
    return ++sul_next_id;
}

/**
 * \brief Get SHA current identifier.
 *
 * \return Current identifier value.
 */
uint32_t sha_internal_get_current_id(void)
{
    return sul_current_id;
}

/**
 * \brief Set SHA current identifier.
 *
 * \param current_id    Current identifier value
 */
void sha_internal_set_current_id(uint32_t current_id)
{
    sul_current_id = current_id;
}

#endif /* #if defined(MBEDTLS_SHA1_ALT) || defined(MBEDTLS_SHA256_ALT) || defined(MBEDTLS_SHA512_ALT) */

