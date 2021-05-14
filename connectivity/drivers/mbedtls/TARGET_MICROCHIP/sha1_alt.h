/**
 * \file sha1_alt.h
 *
 * \brief SHA-1 cryptographic hash function
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

#ifndef MBEDTLS_SHA1_ALT_H
#define MBEDTLS_SHA1_ALT_H

#if defined(MBEDTLS_SHA1_ALT)

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          SHA-1 context structure
 */
typedef struct
{
    uint32_t state[5];          /*!< intermediate digest state  */
    uint32_t total[2];          /*!< number of bytes processed  */
    unsigned char buffer[64];   /*!< data block being processed */
	int isfirst;                /*!< First block */
}
mbedtls_sha1_context;

#ifdef __cplusplus
}
#endif

#endif /* #if defined(MBEDTLS_SHA1_ALT) */

#endif /* #ifndef MBEDTLS_SHA1_ALT_H */
