/*
 *  AES block cipher
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

#ifndef MBEDTLS_AES_ALT_H
#define MBEDTLS_AES_ALT_H

#if defined(MBEDTLS_AES_ALT)

#include "aes_pic32cx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          AES context structure
 */
typedef struct
{
    uint32_t keySize;       /* Key size: AES_KEY_SIZE_128/192/256 */
    uint32_t opMode;        /* ECB/CBC/CFB/OFB/CTR */
    uint32_t keys[8];       /* Cipher key */
}
mbedtls_aes_context;

#ifdef __cplusplus
}
#endif

#endif /* MBEDTLS_AES_ALT */

#endif /* MBEDTLS_AES_ALT_H */
