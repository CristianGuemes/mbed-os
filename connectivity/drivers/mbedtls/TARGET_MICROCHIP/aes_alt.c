/*
*  Hardware-accelerated AES implementation for Microchip devices
*  containing an AES peripheral.
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

#include "mbedtls/aes.h"
#include "pic32cx.h"

#if defined(MBEDTLS_AES_C)
#if defined(MBEDTLS_AES_ALT)

#include "aes_pic32cx.h"
#include <string.h>
#include "mbedtls/platform.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

/* Parameter validation macros based on platform_util.h */
#define AES_VALIDATE_RET(cond)    \
    MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_AES_BAD_INPUT_DATA)

#define AES_VALIDATE(cond)        MBEDTLS_INTERNAL_VALIDATE(cond)

/*
 * 32-bit integer manipulation macros
 */
#define GET_UINT32_BE_local(n,b,i)                      \
{                                                       \
    (n) = ( (uint32_t) (b)[(i)    ]       )             \
        | ( (uint32_t) (b)[(i) + 1] <<  8 )             \
        | ( (uint32_t) (b)[(i) + 2] << 16 )             \
        | ( (uint32_t) (b)[(i) + 3] << 24 );            \
}

/* AES configuration */
struct aes_config g_aes_cfg;

/* AES auto padding configuration */
struct aes_ap_config g_aes_ap_cfg;

/* Output data */
static uint32_t *spul_output_data;

/* State indicate */
volatile bool b_aes_state;

#if defined(MBEDTLS_THREADING_C)
#include "mbedtls/threading.h"
/* Mutex for protecting access to the AES instance */
static mbedtls_threading_mutex_t aes_mutex;
static volatile bool aes_mutex_inited = false;
#endif

/**
* \brief The AES interrupt call back function.
*/
static void aes_int_callback(void)
{
    /* Read the output. */
    aes_read_output_data(AES, spul_output_data);
    b_aes_state = true;
}

static void _aes_lock(void)
{
#if defined(MBEDTLS_THREADING_C)
    if (!aes_mutex_inited) {
        /* Turn off interrupts that can cause preemption */
        Disable_global_interrupt();

        if (!aes_mutex_inited) {
            mbedtls_mutex_init(&aes_mutex);
            aes_mutex_inited = true;
        }

        Enable_global_interrupt();
    }

    mbedtls_mutex_lock(&aes_mutex);
#endif
}

static void _aes_unlock(void)
{
#if defined(MBEDTLS_THREADING_C)
    if (aes_mutex_inited) {
        mbedtls_mutex_unlock(&aes_mutex);
    }
#endif
}

/**
* \brief Request AES encryption/decryption
*/
static void _aes_crypt(mbedtls_aes_context *ctx, bool b_crypt_mode, uint8_t *puc_in_text, uint8_t *puc_out_text)
{
    /* Protect context access                                  */
    /* (it may occur at a same time in a threaded environment) */
    _aes_lock();

    b_aes_state = false;

    /* Configure the AES. */
    g_aes_cfg.encrypt_mode = b_crypt_mode ? AES_ENCRYPTION : AES_DECRYPTION;
    g_aes_cfg.key_size = (aes_key_size_t)ctx->keySize;
    g_aes_cfg.start_mode = AES_AUTO_START;
    g_aes_cfg.opmode = AES_ECB_MODE;
    g_aes_cfg.cfb_size = AES_CFB_SIZE_128;
    g_aes_cfg.lod = false;
    g_aes_cfg.algo = AES_ALGO_AES;
    aes_set_config(AES, &g_aes_cfg, &g_aes_ap_cfg);

    /* Set the cryptographic key. */
    aes_write_key(AES, (const uint32_t*)ctx->keys);

    /* Set the pointer to the output data */
    spul_output_data = (uint32_t *)puc_out_text;

    /* Write the data to be ciphered to the input data registers. */
    aes_write_input_data(AES, (uint32_t const *)puc_in_text);

    /* Wait for the end of the encryption process. */
    while (false == b_aes_state) {
        ;
    }

    /* Free context access */
    _aes_unlock();

    /* Return result - the output data is already in the buffer */
}

/*
* Initialize AES context
*/
void mbedtls_aes_init(mbedtls_aes_context *ctx)
{
    AES_VALIDATE_RET(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_aes_context));

    /* Enable the AES module. */
    aes_get_config_defaults(&g_aes_cfg, &g_aes_ap_cfg);
    aes_init(AES, &g_aes_cfg, &g_aes_ap_cfg);
    aes_enable();

    /* Enable AES interrupt. */
    aes_set_callback(AES, AES_INTERRUPT_DATA_READY, aes_int_callback, 1);
}

/*
* Clear AES context
*/
void mbedtls_aes_free(mbedtls_aes_context *ctx)
{
    AES_VALIDATE_RET(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_aes_context));
}

/*
* AES key schedule (encryption)
*/
int mbedtls_aes_setkey_enc( mbedtls_aes_context *ctx,
                           const unsigned char *key,
                           unsigned int keybits )
{
    unsigned int i;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(key != NULL);

    /* Set the key size */
    switch (keybits) {
    case 128:
        ctx->keySize = AES_KEY_SIZE_128;
        break;
    case 192:
        ctx->keySize = AES_KEY_SIZE_192;
        break;
    case 256:
        ctx->keySize = AES_KEY_SIZE_256;
        break;
    default:
        return MBEDTLS_ERR_AES_INVALID_KEY_LENGTH;
    }

    /* Fetch key byte data */
    for (i = 0; i < (keybits >> 5); i++) {
        GET_UINT32_BE_local(ctx->keys[i], key, i * 4);
    }

    return 0;
}

/*
* AES key schedule (decryption)
*/
int mbedtls_aes_setkey_dec( mbedtls_aes_context *ctx,
                           const unsigned char *key,
                           unsigned int keybits )
{
    return mbedtls_aes_setkey_enc(ctx, key, keybits);
}

/*
* AES-ECB block encryption
*/
int mbedtls_internal_aes_encrypt( mbedtls_aes_context *ctx,
                                 const unsigned char input[16],
                                 unsigned char output[16] )
{
    return mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, input, output);
}

/*
* AES-ECB block decryption
*/
int mbedtls_internal_aes_decrypt( mbedtls_aes_context *ctx,
                                 const unsigned char input[16],
                                 unsigned char output[16] )
{
    return mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_DECRYPT, input, output);
}

/*
* AES-ECB block encryption/decryption
*/
int mbedtls_aes_crypt_ecb( mbedtls_aes_context *ctx,
                          int mode,
                          const unsigned char input[16],
                          unsigned char output[16] )
{
    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    /* Call AES module */
    _aes_crypt(ctx, mode, (uint8_t *)input, (uint8_t *)output);

    return 0;
}

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
* AES-CBC buffer encryption/decryption
*/
int mbedtls_aes_crypt_cbc( mbedtls_aes_context *ctx,
                          int mode,
                          size_t length,
                          unsigned char iv[16],
                          const unsigned char *input,
                          unsigned char *output )
{
    int i;
    unsigned char temp[16];

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    if (length % 16) {
        return MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH;
    }

    if (mode == MBEDTLS_AES_DECRYPT) {
        while (length > 0) {
            memcpy(temp, input, 16);
            mbedtls_aes_crypt_ecb(ctx, mode, input, output);

            for (i = 0; i < 16; i++) {
                output[i] = (unsigned char)(output[i] ^ iv[i]);
            }

            memcpy(iv, temp, 16);

            input  += 16;
            output += 16;
            length -= 16;
        }
    }
    else {
        while (length > 0) {
            for( i = 0; i < 16; i++) {
                output[i] = (unsigned char)(input[i] ^ iv[i]);
            }

            mbedtls_aes_crypt_ecb(ctx, mode, output, output);

            memcpy(iv, output, 16);

            input  += 16;
            output += 16;
            length -= 16;
        }
    }

    return 0;
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

#if defined(MBEDTLS_CIPHER_MODE_CFB)
/*
* AES-CFB128 buffer encryption/decryption
*/
int mbedtls_aes_crypt_cfb128( mbedtls_aes_context *ctx,
                             int mode,
                             size_t length,
                             size_t *iv_off,
                             unsigned char iv[16],
                             const unsigned char *input,
                             unsigned char *output )
{
    int c;
    size_t n;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(iv_off != NULL);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    n = *iv_off;

    if (n > 15) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    if (mode == MBEDTLS_AES_DECRYPT) {
        while (length--) {
            if (n == 0) {
                mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
            }

            c = *input++;
            *output++ = (unsigned char)(c ^ iv[n]);
            iv[n] = (unsigned char)c;

            n = (n + 1) & 0x0F;
        }
    }
    else {
        while (length--) {
            if (n == 0) {
                mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
            }

            iv[n] = *output++ = (unsigned char)(iv[n] ^ *input++);

            n = (n + 1) & 0x0F;
        }
    }

    *iv_off = n;

    return 0;
}

/*
* AES-CFB8 buffer encryption/decryption
*/
int mbedtls_aes_crypt_cfb8( mbedtls_aes_context *ctx,
                           int mode,
                           size_t length,
                           unsigned char iv[16],
                           const unsigned char *input,
                           unsigned char *output )
{
    unsigned char c;
    unsigned char ov[17];

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    while (length--) {
        memcpy(ov, iv, 16);
        mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);

        if (mode == MBEDTLS_AES_DECRYPT) {
            ov[16] = *input;
        }

        c = *output++ = (unsigned char)(iv[0] ^ *input++);

        if (mode == MBEDTLS_AES_ENCRYPT) {
            ov[16] = c;
        }

        memcpy(iv, ov + 1, 16);
    }

    return 0;
}
#endif /*MBEDTLS_CIPHER_MODE_CFB */

#if defined(MBEDTLS_CIPHER_MODE_OFB)
/*
* AES-OFB buffer encryption/decryption
*/
int mbedtls_aes_crypt_ofb( mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *iv_off,
                          unsigned char iv[16],
                          const unsigned char *input,
                          unsigned char *output )
{
    size_t n;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(iv_off != NULL);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    n = *iv_off;

    if (n > 15) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    while (length--) {
        if (n == 0) {
            mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
        }

        *output++ = *input++ ^ iv[n];

        n = (n + 1) & 0x0F;
    }

    *iv_off = n;

    return 0;
}
#endif /* MBEDTLS_CIPHER_MODE_OFB */

#if defined(MBEDTLS_CIPHER_MODE_CTR)
/*
* AES-CTR buffer encryption/decryption
*/
int mbedtls_aes_crypt_ctr( mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *nc_off,
                          unsigned char nonce_counter[16],
                          unsigned char stream_block[16],
                          const unsigned char *input,
                          unsigned char *output )
{
    int c, i;
    size_t n;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(nc_off != NULL);
    AES_VALIDATE_RET(nonce_counter != NULL);
    AES_VALIDATE_RET(stream_block != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    n = *nc_off;

    if (n > 0x0F) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    while (length--) {
        if (n == 0) {
            mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, nonce_counter, stream_block);

            for (i = 16; i > 0; i--) {
                if (++nonce_counter[i - 1] != 0) {
                    break;
                }
            }
        }

        c = *input++;
        *output++ = (unsigned char)(c ^ stream_block[n]);

        n = (n + 1) & 0x0F;
    }

    *nc_off = n;

    return 0;
}
#endif /* MBEDTLS_CIPHER_MODE_CTR */

#endif /* MBEDTLS_AES_ALT */
#endif /* MBEDTLS_AES_C */
