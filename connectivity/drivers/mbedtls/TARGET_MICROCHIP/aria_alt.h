/*
*  ARIA block cipher
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

#ifndef MBEDTLS_ARIA_ALT_H
#define MBEDTLS_ARIA_ALT_H

#if defined(MBEDTLS_ARIA_ALT)

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
* \brief The ARIA context-type definition.
*/
typedef struct mbedtls_aria_context
{
    uint8_t keySize;        /* Key size: AES_KEY_SIZE_128/192/256 */
    uint8_t opMode;         /* ECB/CBC/CFB/CTR */
    uint32_t keys[8];       /* Cipher key */
    bool encDec;            /* MBEDTLS_ARIA_DECRYPT / MBEDTLS_ARIA_ENCRYPT */
}
mbedtls_aria_context;

#ifdef __cplusplus
}
#endif

#endif /* MBEDTLS_ARIA_ALT */

#endif /* MBEDTLS_ARIA_ALT_H */
