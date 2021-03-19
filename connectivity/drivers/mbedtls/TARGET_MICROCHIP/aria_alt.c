/*
 *  Hardware-accelerated ARIA implementation for Microchip devices
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

#if defined(MBEDTLS_ARIA_ALT)

#include "aes_pic32cx.h"
#include <string.h>
#include "mbedtls/platform.h"

/* ARIA configuration */
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
 * \brief Request ARIA encryption/decryption
 */
static void _aria_crypt(mbedtls_aria_context *ctx, uint8_t *puc_init_vector, uint8_t *puc_in_text, uint8_t *puc_out_text)
{
	/* Protect context access                                  */
    /* (it may occur at a same time in a threaded environment) */
	_aes_lock();

	b_aes_state = false;

	/* Configure the AES. */
	g_aes_cfg.encrypt_mode = ctx->encDec;
	g_aes_cfg.key_size = ctx->keySize;
	g_aes_cfg.start_mode = AES_AUTO_START;
	g_aes_cfg.opmode = ctx->opMode;
	g_aes_cfg.cfb_size = AES_CFB_SIZE_128;
	g_aes_cfg.lod = false;
	g_aes_cfg.algo = AES_ALGO_ARIA;
	aes_set_config(AES, &g_aes_cfg, &g_aes_ap_cfg);

	/* Set the cryptographic key. */
	aes_write_key(AES, (const *)ctx->keys);

	/* Set the initialization vector. */
	if (ctx->opMode != AES_ECB_MODE) {
		aes_write_initvector(AES, (uint32_t const *)puc_init_vector);
	}
		  
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

/**
 * \brief Store the key
 */
int _aria_setkey( mbedtls_aria_context *ctx,
                  const unsigned char *key,
                  unsigned int keybits )
{
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
        return MBEDTLS_ERR_ARIA_INVALID_KEY_LENGTH;
	}

    memcpy(ctx->keys, key, keybits / 32);

    return 0;
}

/*
 * Initialize ARIA context
 */
void mbedtls_aria_init(mbedtls_aria_context *ctx)
{
	ARIA_VALIDATE_RET(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_aria_context));

	/* Enable the AES module. */
	aes_get_config_defaults(&g_aes_cfg, &g_aes_ap_cfg);
	aes_init(AES, &g_aes_cfg, &g_aes_ap_cfg);
	aes_enable();

	/* Enable AES interrupt. */
	aes_set_callback(AES, AES_INTERRUPT_DATA_READY, aes_int_callback, 1);
}

/*
 * Clear ARIA context
 */
void mbedtls_aria_free(mbedtls_aria_context *ctx)
{
	ARIA_VALIDATE_RET(ctx != NULL);

	memset(ctx, 0, sizeof(mbedtls_aria_context));

	/* Disable the AES module. */
	aes_disable();
}

/*
 * ARIA key schedule (encryption)
 */
int mbedtls_aria_setkey_enc( mbedtls_aria_context *ctx,
                            const unsigned char *key,
                            unsigned int keybits )
{
	ARIA_VALIDATE_RET(ctx != NULL);
	ARIA_VALIDATE_RET(key != NULL);

	ctx->encDec = MBEDTLS_ARIA_ENCRYPT;
	return _aria_setkey(ctx, key, keybits);
}

/*
 * ARIA key schedule (decryption)
 */
int mbedtls_aria_setkey_dec( mbedtls_aria_context *ctx,
                            const unsigned char *key,
                            unsigned int keybits )
{
    ctx->encDec = MBEDTLS_ARIA_DECRYPT;
	return _aria_setkey(ctx, key, keybits);
}

/*
 * ARIA-ECB block encryption/decryption
 */
int mbedtls_aria_crypt_ecb( mbedtls_aria_context *ctx,
                            const unsigned char input[MBEDTLS_ARIA_BLOCKSIZE],
                            unsigned char output[MBEDTLS_ARIA_BLOCKSIZE] )
{
	ARIA_VALIDATE_RET(ctx != NULL);
    ARIA_VALIDATE_RET(input != NULL);
    ARIA_VALIDATE_RET(output != NULL);

	/* Set mode */
	ctx->opMode = AES_ECB_MODE;

	/* Call AES module */
	_aria_crypt(ctx, NULL, input, output);

    return 0;
}

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * ARIA-CBC buffer encryption/decryption
 */
int mbedtls_aria_crypt_cbc( mbedtls_aria_context *ctx,
                            int mode,
                            size_t length,
                            unsigned char iv[MBEDTLS_ARIA_BLOCKSIZE],
                            const unsigned char *input,
                            unsigned char *output )				   
{
	
    int i;

	ARIA_VALIDATE_RET(ctx != NULL);
    ARIA_VALIDATE_RET(mode == MBEDTLS_ARIA_ENCRYPT ||
                     mode == MBEDTLS_ARIA_DECRYPT);
    ARIA_VALIDATE_RET(iv != NULL);
    ARIA_VALIDATE_RET(input != NULL);
    ARIA_VALIDATE_RET(output != NULL);

    if (length % MBEDTLS_ARIA_BLOCKSIZE) {
        return MBEDTLS_ERR_ARIA_INVALID_INPUT_LENGTH;
	}
		
	/* Set mode */
	ctx->opMode = AES_CBC_MODE;
	
	/* Check encrypt/decrypt */
    if(mode == MBEDTLS_ARIA_DECRYPT) {
        while (length > 0) {
			/* Call AES module for one block */
			_aria_crypt(ctx, iv, input, output);

			/* Set new init vector */
            memcpy(iv, input, MBEDTLS_ARIA_BLOCKSIZE);

            input  += MBEDTLS_ARIA_BLOCKSIZE;
            output += MBEDTLS_ARIA_BLOCKSIZE;
            length -= MBEDTLS_ARIA_BLOCKSIZE;
        }
    }
    else { /* MBEDTLS_ARIA_ENCRYPT */
        while (length > 0) {
			/* Call AES module for one block */
			_aria_crypt(ctx, iv, input, output);

			/* Set new init vector */
            memcpy(iv, output, MBEDTLS_ARIA_BLOCKSIZE);

            input  += MBEDTLS_ARIA_BLOCKSIZE;
            output += MBEDTLS_ARIA_BLOCKSIZE;
            length -= MBEDTLS_ARIA_BLOCKSIZE;
        }
    }

    return 0;
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

#if defined(MBEDTLS_CIPHER_MODE_CFB)
/*
 * ARIA-CFB128 buffer encryption/decryption
 */
int mbedtls_aria_crypt_cfb128( mbedtls_aria_context *ctx,
                               int mode,
                               size_t length,
                               size_t *iv_off,
                               unsigned char iv[MBEDTLS_ARIA_BLOCKSIZE],
                               const unsigned char *input,
                               unsigned char *output )
{
    size_t n = (iv_off != NULL) ? *iv_off : 0;

    ARIA_VALIDATE_RET(ctx != NULL);
    ARIA_VALIDATE_RET(mode == MBEDTLS_ARIA_ENCRYPT ||
                     mode == MBEDTLS_ARIA_DECRYPT);
    ARIA_VALIDATE_RET(iv_off != NULL);
    ARIA_VALIDATE_RET(iv != NULL);
    ARIA_VALIDATE_RET(input != NULL);
    ARIA_VALIDATE_RET(output != NULL);

	if (n >= MBEDTLS_ARIA_BLOCKSIZE) {
        return MBEDTLS_ERR_ARIA_BAD_INPUT_DATA;
	}
		
    if ((n > 0) || (length & 0xf)) {
        /* IV offset or length not aligned to block size */
        int c;

        if (mode == MBEDTLS_ARIA_DECRYPT) {
            while (length--) {
                if (n == 0) {
                    mbedtls_aria_crypt_ecb(ctx, iv, iv);
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
                    mbedtls_aria_crypt_ecb(ctx, iv, iv);
				}

                iv[n] = *output++ = (unsigned char)(iv[n] ^ *input++);

                n = (n + 1) & 0x0F;
            }
        }

        if (iv_off) {
            *iv_off = n;
		}
    }
    else {
		/* Set mode */
		ctx->opMode = AES_CFB_MODE;

		/* Call AES module for one block */
		_aria_crypt(ctx, iv, input, output);
	}

    return 0;
}

#if defined(MBEDTLS_CIPHER_MODE_CTR)
/*
 * ARIA-CTR buffer encryption/decryption
 */
int mbedtls_aria_crypt_ctr( mbedtls_aria_context *ctx,
                            size_t length,
                            size_t *nc_off,
                            unsigned char nonce_counter[MBEDTLS_ARIA_BLOCKSIZE],
                            unsigned char stream_block[MBEDTLS_ARIA_BLOCKSIZE],
                            const unsigned char *input,
                            unsigned char *output )
{
    int c, i;
    size_t n = *nc_off;
	
	ARIA_VALIDATE_RET( ctx != NULL );
    ARIA_VALIDATE_RET( length == 0 || input  != NULL );
    ARIA_VALIDATE_RET( length == 0 || output != NULL );
    ARIA_VALIDATE_RET( nonce_counter != NULL );
    ARIA_VALIDATE_RET( stream_block  != NULL );
    ARIA_VALIDATE_RET( nc_off != NULL );
	
	if (n >= MBEDTLS_ARIA_BLOCKSIZE)
        return MBEDTLS_ERR_ARIA_BAD_INPUT_DATA;
    }

	while (length--) {
		if (n == 0) {
			mbedtls_aria_crypt_ecb(ctx, MBEDTLS_ARIA_ENCRYPT, nonce_counter, stream_block);

			for (i = MBEDTLS_ARIA_BLOCKSIZE; i > 0; i--) {
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

#endif /* MBEDTLS_ARIA_ALT */
