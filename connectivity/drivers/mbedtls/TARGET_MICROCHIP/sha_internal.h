/**
 * \file sha_internal.h
 *
 * \brief SHA internal functions
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

#ifndef MBEDTLS_SHA_INTERNAL_H
#define MBEDTLS_SHA_INTERNAL_H

#if defined(MBEDTLS_SHA1_ALT) || defined(MBEDTLS_SHA256_ALT) || defined(MBEDTLS_SHA512_ALT)

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t sha_internal_get_new_id(void);
uint32_t sha_internal_get_current_id(void);
void sha_internal_set_current_id(uint32_t current_id);

#ifdef __cplusplus
}
#endif

#endif /* #if defined(MBEDTLS_SHA1_ALT)  || defined(MBEDTLS_SHA256_ALT) || defined(MBEDTLS_SHA512_ALT) */

#endif /* #ifndef MBEDTLS_SHA_INTERNAL_H */
