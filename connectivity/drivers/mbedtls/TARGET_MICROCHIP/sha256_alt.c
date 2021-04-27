/*
 *  Hardware-accelerated SHA implementation for Microchip devices
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

#include "mbedtls/sha256.h"
#include "pic32cx.h"

#if defined(MBEDTLS_SHA256_C)
#if defined(MBEDTLS_SHA256_ALT)

#include "sha_pic32cx.h"
#include <string.h>
#include "mbedtls/platform.h"

#define SHA256_VALIDATE_RET(cond)                           \
    MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_SHA256_BAD_INPUT_DATA)
	
#define SHA256_VALIDATE(cond)  MBEDTLS_INTERNAL_VALIDATE(cond)

#ifndef PUT_UINT32_BE
#define PUT_UINT32_BE(n,b,i)                            \
{                                                       \
    (b)[(i)    ] = (unsigned char) ( (n) >> 24 );       \
    (b)[(i) + 1] = (unsigned char) ( (n) >> 16 );       \
    (b)[(i) + 2] = (unsigned char) ( (n) >>  8 );       \
    (b)[(i) + 3] = (unsigned char) ( (n)       );       \
}
#endif

/* Output data array */
static uint32_t output_data[SHA_HASH_SIZE_SHA512];

/* SHA configuration */
struct sha_config g_sha_cfg;

/* State indicate */
volatile bool b_sha1_state = false;

/**
 * \brief The SHA256 interrupt call back function.
 */
static void sha256_callback(uint8_t uc_data)
{
	(void)uc_data;

	/* Read the output */
	sha_read_output_data(SHA, output_data);
	b_sha1_state = true;
}

/*
 * Initialize SHA256 context
 */
void mbedtls_sha256_init(mbedtls_sha256_context *ctx)
{
	SHA256_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha256_context));

	/* Enable the SHA module */
	sha_get_config_defaults(&g_sha_cfg);
	sha_init(SHA, &g_sha_cfg);
	sha_enable();

	/* Enable SHA interrupt */
	sha_set_callback(SHA, SHA_INTERRUPT_DATA_READY, sha_callback256, 1);
}

/*
 * Free SHA256 context
 */
void mbedtls_sha256_free(mbedtls_sha256_context *ctx)
{
	SHA256_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha256_context));
	
	/* Disable the SHA module */
	sha_disable();
}

/*
 * Clone SHA256 context
 */
void mbedtls_sha256_clone(mbedtls_sha256_context *dst, const mbedtls_sha256_context *src)
{
	SHA256_VALIDATE(dst != NULL);
    SHA256_VALIDATE(src != NULL);

    /* Corner case: Destination/source contexts are the same */
    if (dst == src) {
        return;
    }

    *dst = *src;
}

/*
 * SHA-256 context setup
 */
int mbedtls_sha256_starts_ret(mbedtls_sha256_context *ctx, int is224)
{
	SHA256_VALIDATE(ctx != NULL);
	SHA256_VALIDATE_RET(is224 == 0 || is224 == 1);

	ctx->is224 = is224;
	
	/* Configure the SHA */
	g_sha_cfg.start_mode = SHA_MANUAL_START;
	g_sha_cfg.aoe = false;
	g_sha_cfg.procdly = false;
	g_sha_cfg.uihv = false;
	g_sha_cfg.uiehv = false;
	g_sha_cfg.bpe = false;
	if (is224 == 0) {
		g_sha_cfg.algo = SHA_ALGO_SHA256;	
	} else {
		g_sha_cfg.algo = SHA_ALGO_SHA224;
	}
	g_sha_cfg.tmpclk = false;
	g_sha_cfg.dualbuff = false;
	g_sha_cfg.check = SHA_NO_CHECK;
	g_sha_cfg.chkcnt = 0;
	sha_set_config(SHA, &g_sha_cfg);

	/* No automatic padding */
	sha_set_msg_size(SHA, 0); 
	sha_set_byte_count(SHA, 0);  

	/* Set first block */
	sha_set_first_block(SHA);

    return 0;
}

/*
 * SHA-256 process buffer
 */
int mbedtls_sha256_update_ret(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen)
{
    int err = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t fill;
    uint32_t left;

    SHA256_VALIDATE_RET(ctx != NULL);
    SHA256_VALIDATE_RET(ilen == 0 || input != NULL);

	if (ilen == 0) {
        return 0;
	}

    left = ctx->total[0] & 0x3F;
    fill = 64 - left;

    ctx->total[0] += (uint32_t)ilen;
    ctx->total[0] &= 0xFFFFFFFF;

    if (ctx->total[0] < (uint32_t)ilen) {
        ctx->total[1]++;
	}

    if (left && ilen >= fill) {
        memcpy((void *)(ctx->buffer + left), input, fill);

        if ((err = mbedtls_internal_sha256_process(ctx, ctx->buffer)) != 0) {
			return err;
		}

        input += fill;
        ilen -= fill;
        left = 0;
    }

    while (ilen >= 64) {
        if ((err = mbedtls_internal_sha256_process(ctx, ctx->buffer)) != 0) {
			return err;
		}

        input += 64;
        ilen -= 64;
    }

    if (ilen > 0) {
        memcpy((void *)(ctx->buffer + left), input, ilen);
	}

    return 0;
}

/*
 * SHA-256 final digest
 */
int mbedtls_sha256_finish_ret(mbedtls_sha256_context *ctx, unsigned char output[32])
{
    int err = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    uint32_t used;
    uint32_t high, low;

    SHA1_VALIDATE_RET(ctx != NULL);
    SHA1_VALIDATE_RET((unsigned char *)output != NULL);

    /*
     * Add padding: 0x80 then 0x00 until 8 bytes remain for the length
     */
    used = ctx->total[0] & 0x3F;

    ctx->buffer[used++] = 0x80;

    if (used <= 56) {
        /* Enough room for padding + length in current block */
        memset(ctx->buffer + used, 0, 56 - used);
    } else {
        /* We'll need an extra block */
        memset(ctx->buffer + used, 0, 64 - used);

        if ((err = mbedtls_internal_sha256_process(ctx, ctx->buffer)) != 0) {
            return err;
		}

        memset(ctx->buffer, 0, 56);
    }

    /*
     * Add message length
     */
    high = (ctx->total[0] >> 29) | (ctx->total[1] << 3);
    low = (ctx->total[0] <<  3);

    PUT_UINT32_BE(high, ctx->buffer, 56);
    PUT_UINT32_BE(low, ctx->buffer, 60);

    if ((err = mbedtls_internal_sha1_process(ctx, ctx->buffer)) != 0) {
        return err;
	}

    /*
     * Output final state
     */
    PUT_UINT32_BE(ctx->state[0], output, 0);
    PUT_UINT32_BE(ctx->state[1], output, 4);
    PUT_UINT32_BE(ctx->state[2], output, 8);
    PUT_UINT32_BE(ctx->state[3], output, 12);
    PUT_UINT32_BE(ctx->state[4], output, 16);
    PUT_UINT32_BE(ctx->state[5], output, 20);
    PUT_UINT32_BE(ctx->state[6], output, 24);

    if (ctx->is224 == 0) {
        PUT_UINT32_BE(ctx->state[7], output, 28);
	}

    return 0;
}

/*
 * Internal SHA256 process
 */
int mbedtls_internal_sha256_process(mbedtls_sha256_context *ctx, const unsigned char data[64])
{
	SHA256_VALIDATE_RET(ctx != NULL);
    SHA256_VALIDATE_RET((const unsigned char *)data != NULL);
	
	/* Write the data to be hashed to the input data registers */
	sha_write_input_data(SHA, (uint32_t *)data, SHA_BLOCK_SIZE_16);

	/* Start the SHA */
	state = false;
	sha_start(SHA);

	/* Wait for the end of the SHA process */
	while (false == state) {
	}

	/* Copy intermediate result */
	if (ctx->is224 == 0) {
		memcpy(ctx->state, output_data, SHA_HASH_SIZE_SHA256);
	} else {
		memcpy(ctx->state, output_data, SHA_HASH_SIZE_SHA224);
	}

    return 0;
}

#endif /* MBEDTLS_SHA256_ALT */
#endif /* MBEDTLS_SHA256_C */
