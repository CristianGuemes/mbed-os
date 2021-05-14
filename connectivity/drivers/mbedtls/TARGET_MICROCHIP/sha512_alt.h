/**
 * \file sha512_alt.h
 *
 * \brief SHA-512 cryptographic hash function
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

#ifndef MBEDTLS_SHA512_ALT_H
#define MBEDTLS_SHA512_ALT_H

#if defined(MBEDTLS_SHA512_ALT)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          SHA-512 context structure
 */
typedef struct mbedtls_sha512_context_s {
	uint32_t state[16];         /*!< intermediate digest state  */
    uint64_t total[2];          /*!< number of bytes processed  */
    unsigned char buffer[128];  /*!< data block being processed */
    int is384;                  /*!< Determines which function to use:
                                     0: Use SHA-512, or 1: Use SHA-384. */
	int isfirst;                /*!< First block */
}
mbedtls_sha512_context;

#ifdef __cplusplus
}
#endif

#endif /* #if defined MBEDTLS_SHA512_ALT */

#endif /* #ifndef MBEDTLS_SHA512_ALT_H */
