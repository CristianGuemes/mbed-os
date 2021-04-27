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


#include "mbedtls/sha512.h"
#include "pic32cx.h"

#if defined(MBEDTLS_SHA512_C)
#if defined(MBEDTLS_SHA512_ALT)

#include "sha_pic32cx.h"
#include <string.h>
#include "mbedtls/platform.h"

#define SHA512_VALIDATE_RET(cond)                           \
    MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_SHA512_BAD_INPUT_DATA)
#define SHA512_VALIDATE(cond)  MBEDTLS_INTERNAL_VALIDATE(cond)

#ifndef PUT_UINT64_BE
#define PUT_UINT64_BE(n,b,i)                            \
{                                                       \
    (b)[(i)    ] = (unsigned char) ( (n) >> 56 );       \
    (b)[(i) + 1] = (unsigned char) ( (n) >> 48 );       \
    (b)[(i) + 2] = (unsigned char) ( (n) >> 40 );       \
    (b)[(i) + 3] = (unsigned char) ( (n) >> 32 );       \
    (b)[(i) + 4] = (unsigned char) ( (n) >> 24 );       \
    (b)[(i) + 5] = (unsigned char) ( (n) >> 16 );       \
    (b)[(i) + 6] = (unsigned char) ( (n) >>  8 );       \
    (b)[(i) + 7] = (unsigned char) ( (n)       );       \
}
#endif

/* Output data array */
static uint32_t output_data[SHA_HASH_SIZE_SHA512];

/* SHA configuration */
struct sha_config g_sha_cfg;

/* State indicate */
volatile bool b_sha1_state = false;

/**
 * \brief The SHA512 interrupt call back function.
 */
static void sha512_callback(uint8_t uc_data)
{
	(void)uc_data;

	/* Read the output */
	sha_read_output_data(SHA, output_data);
	b_sha1_state = true;
}

/*
 * Initialize SHA512 context
 */
void mbedtls_sha512_init(mbedtls_sha512_context *ctx)
{
    SHA512_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha512_context));

	/* Enable the SHA module */
	sha_get_config_defaults(&g_sha_cfg);
	sha_init(SHA, &g_sha_cfg);
	sha_enable();

	/* Enable SHA interrupt */
	sha_set_callback(SHA, SHA_INTERRUPT_DATA_READY, sha_callback512, 1);
}

/*
 * Free SHA512 context
 */
void mbedtls_sha512_free(mbedtls_sha512_context *ctx)
{
	SHA512_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha512_context));
	
	/* Disable the SHA module */
	sha_disable();
}

/*
 * Clone SHA512 context
 */
void mbedtls_sha512_clone(mbedtls_sha512_context *dst, const mbedtls_sha512_context *src)
{
	SHA512_VALIDATE(dst != NULL);
    SHA512_VALIDATE(src != NULL);

    /* Corner case: Destination/source contexts are the same */
    if (dst == src) {
        return;
    }

    *dst = *src;
}

/*
 * SHA-512 context setup
 */
int mbedtls_sha512_starts_ret(mbedtls_sha512_context *ctx, int is384)
{
	SHA512_VALIDATE(ctx != NULL);
    SHA512_VALIDATE_RET(is384 == 0 || is384 == 1);

    ctx->is384 = is384;
	
	/* Configure the SHA */
	g_sha_cfg.start_mode = SHA_MANUAL_START;
	g_sha_cfg.aoe = false;
	g_sha_cfg.procdly = false;
	g_sha_cfg.uihv = false;
	g_sha_cfg.uiehv = false;
	g_sha_cfg.bpe = false;
	if (is384 == 0) {
		g_sha_cfg.algo = SHA_ALGO_SHA512;	
	} else {
		g_sha_cfg.algo = SHA_ALGO_SHA384;
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
 * SHA-512 process buffer
 */
int mbedtls_sha512_update_ret(mbedtls_sha512_context *ctx, const unsigned char *input, size_t ilen)
{
    int err = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t fill;
    unsigned int left;

    SHA512_VALIDATE_RET(ctx != NULL);
    SHA512_VALIDATE_RET(ilen == 0 || input != NULL);

    if (ilen == 0) {
        return 0;
	}

    left = (unsigned int) (ctx->total[0] & 0x7F);
    fill = 128 - left;

    ctx->total[0] += (uint64_t)ilen;

    if (ctx->total[0] < (uint64_t)ilen) {
        ctx->total[1]++;
	}

    if (left && ilen >= fill) {
        memcpy((void *)(ctx->buffer + left), input, fill);

        if ((err = mbedtls_internal_sha512_process(ctx, ctx->buffer)) != 0) {
            return err;
		}

        input += fill;
        ilen -= fill;
        left = 0;
    }

    while (ilen >= 128) {
        if ((err = mbedtls_internal_sha512_process(ctx, input)) != 0) {
            return err;
		}

        input += 128;
        ilen -= 128;
    }

    if (ilen > 0) {
        memcpy((void *)(ctx->buffer + left), input, ilen);
	}
		
    return 0;
}

/*
 * SHA-256 final digest
 */
int mbedtls_sha512_finish_ret(mbedtls_sha512_context *ctx, unsigned char output[64])
{
    int err = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned used;
    uint64_t high, low;

    SHA512_VALIDATE_RET(ctx != NULL);
    SHA512_VALIDATE_RET((unsigned char *)output != NULL);

    /*
     * Add padding: 0x80 then 0x00 until 16 bytes remain for the length
     */
    used = ctx->total[0] & 0x7F;

    ctx->buffer[used++] = 0x80;

    if (used <= 112) {
        /* Enough room for padding + length in current block */
        memset(ctx->buffer + used, 0, 112 - used);
    } else {
        /* We'll need an extra block */
        memset(ctx->buffer + used, 0, 128 - used);

        if ((err = mbedtls_internal_sha512_process( ctx, ctx->buffer)) != 0) {
            return err;
		}

        memset(ctx->buffer, 0, 112);
    }

    /*
     * Add message length
     */
    high = (ctx->total[0] >> 61) | (ctx->total[1] <<  3);
    low = (ctx->total[0] <<  3);

    sha512_put_uint64_be(high, ctx->buffer, 112);
    sha512_put_uint64_be(low, ctx->buffer, 120);

    if ((err = mbedtls_internal_sha512_process(ctx, ctx->buffer)) != 0) {
        return err;
	}

    /*
     * Output final state
     */
    sha512_put_uint64_be(ctx->state[0], output, 0);
    sha512_put_uint64_be(ctx->state[1], output, 8);
    sha512_put_uint64_be(ctx->state[2], output, 16);
    sha512_put_uint64_be(ctx->state[3], output, 24);
    sha512_put_uint64_be(ctx->state[4], output, 32);
    sha512_put_uint64_be(ctx->state[5], output, 40);

    if (ctx->is384 == 0) {
        sha512_put_uint64_be(ctx->state[6], output, 48);
        sha512_put_uint64_be(ctx->state[7], output, 56);
    }

    return 0;
}

int mbedtls_internal_sha512_process(mbedtls_sha512_context *ctx, const unsigned char data[128])
{
	SHA512_VALIDATE_RET(ctx != NULL);
    SHA512_VALIDATE_RET((const unsigned char *)data != NULL);
	
	/* Write the data to be hashed to the input data registers */
	sha_write_input_data(SHA, (uint32_t *)data, SHA_BLOCK_SIZE_32);

	/* Start the SHA */
	state = false;
	sha_start(SHA);

	/* Wait for the end of the SHA process */
	while (false == state) {
	}

	/* Copy intermediate result */
	if (ctx->is384 == 0) {
		memcpy(ctx->state, output_data, SHA_HASH_SIZE_SHA512 / 2);
	} else {
		memcpy(ctx->state, output_data, SHA_HASH_SIZE_SHA384 / 2);
	}

    return 0;
}

#endif /* MBEDTLS_SHA512_ALT */
#endif /* MBEDTLS_SHA512_C */
