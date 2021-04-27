/**
 * \file sha256_alt.h
 *
 * \brief SHA-256 cryptographic hash function
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

#ifndef MBEDTLS_SHA256_ALT_H
#define MBEDTLS_SHA256_ALT_H

#if defined(MBEDTLS_SHA256_ALT)

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          SHA-256 context structure
 */
typedef struct mbedtls_sha256_context_s {
    uint32_t state[8];          /*!< intermediate digest state  */
    uint32_t total[2];          /*!< number of bytes processed  */
    unsigned char buffer[64];   /*!< data block being processed */
    int is224;                  /*!< Determines which function to use:
                                     0: Use SHA-256, or 1: Use SHA-224 */
}
mbedtls_sha256_context;


/* Internal use */
int mbedtls_internal_sha256_process(mbedtls_sha256_context *ctx, const unsigned char data[64]);

#ifdef __cplusplus
}
#endif

#endif /* #if defined MBEDTLS_SHA256_ALT */

#endif /* #ifndef MBEDTLS_SHA256_ALT_H */
