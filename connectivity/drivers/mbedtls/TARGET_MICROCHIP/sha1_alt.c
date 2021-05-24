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

#include "mbedtls/sha1.h"
#include "pic32cx.h"

#if defined(MBEDTLS_SHA1_C)
#if defined(MBEDTLS_SHA1_ALT)

#include "sha.h"
#include "sha_internal.h"
#include <string.h>
#include "mbedtls/platform.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#define SHA1_VALIDATE_RET(cond)                             \
    MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_SHA1_BAD_INPUT_DATA)

#define SHA1_VALIDATE(cond)  MBEDTLS_INTERNAL_VALIDATE(cond)

#ifndef PUT_UINT32_BE
#define PUT_UINT32_BE(n,b,i)                         \
{                                                    \
    (b)[(i)    ] = (unsigned char)((n) >> 24);       \
    (b)[(i) + 1] = (unsigned char)((n) >> 16);       \
    (b)[(i) + 2] = (unsigned char)((n) >> 8);        \
    (b)[(i) + 3] = (unsigned char)((n));             \
}
#endif

#define PUT_UINT32_BE_local(n,b,i)                   \
{                                                    \
    (b)[(i) + 3] = (unsigned char)((n) >> 24);       \
    (b)[(i) + 2] = (unsigned char)((n) >> 16);       \
    (b)[(i) + 1] = (unsigned char)((n) >> 8);        \
    (b)[(i)    ] = (unsigned char)((n));             \
}

/* Output data array */
static uint32_t sha1_output_data[SHA_HASH_SIZE_SHA512];

/* SHA configuration */
struct sha_config g_sha1_cfg;

/* State indicate */
volatile bool b_sha1_state = false;

#if defined(MBEDTLS_THREADING_C)
#include "mbedtls/threading.h"
/* Mutex for protecting access to the SHA instance */
static mbedtls_threading_mutex_t sha1_mutex;
static volatile bool sha1_mutex_inited = false;
#endif

/**
* \brief The SHA1 interrupt call back function.
*/
static void sha1_callback(uint8_t uc_data)
{
    (void)uc_data;

    /* Read the output */
    sha_read_output_data(SHA, sha1_output_data);
    b_sha1_state = true;
}

static void _sha1_lock(void)
{
#if defined(MBEDTLS_THREADING_C)
    if (!sha1_mutex_inited) {
        /* Turn off interrupts that can cause preemption */
        Disable_global_interrupt();

        if (!sha1_mutex_inited) {
            mbedtls_mutex_init(&sha1_mutex);
            sha1_mutex_inited = true;
        }

        Enable_global_interrupt();
    }

    mbedtls_mutex_lock(&sha1_mutex);
#endif
}

static void _sha1_unlock(void)
{
#if defined(MBEDTLS_THREADING_C)
    if (sha1_mutex_inited) {
        mbedtls_mutex_unlock(&sha1_mutex);
    }
#endif
}

/*
* Initialize SHA1 context
*/
void mbedtls_sha1_init(mbedtls_sha1_context *ctx)
{
    SHA1_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha1_context));

    /* Enable the SHA module */
    sha_get_config_defaults(&g_sha1_cfg);
    sha_init(SHA, &g_sha1_cfg);
    sha_enable();

    /* Enable SHA interrupt */
    sha_set_callback(SHA, SHA_INTERRUPT_DATA_READY, sha1_callback, 1);
}

/*
* Free SHA1 context
*/
void mbedtls_sha1_free( mbedtls_sha1_context *ctx )
{
    SHA1_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha1_context));
}

/*
* Clone SHA1 context
*/
void mbedtls_sha1_clone(mbedtls_sha1_context *dst, const mbedtls_sha1_context *src)
{
    SHA1_VALIDATE(dst != NULL);
    SHA1_VALIDATE(src != NULL);

    /* Corner case: Destination/source contexts are the same */
    if (dst == src) {
        return;
    }

    memcpy(dst, src, sizeof(mbedtls_sha1_context));
    dst->id = sha_internal_get_new_id();
}

/*
* SHA-1 context setup
*/
int mbedtls_sha1_starts_ret(mbedtls_sha1_context *ctx)
{
    SHA1_VALIDATE(ctx != NULL);

    ctx->total[0] = 0;
    ctx->total[1] = 0;
    ctx->isfirst = true;
    ctx->id = sha_internal_get_new_id();

    return 0;
}

/*
* SHA-1 process buffer
*/
int mbedtls_sha1_update_ret(mbedtls_sha1_context *ctx, const unsigned char *input, size_t ilen)
{
    int err = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t fill;
    uint32_t left;

    SHA1_VALIDATE_RET(ctx != NULL);
    SHA1_VALIDATE_RET(ilen == 0 || input != NULL);

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

        if ((err = mbedtls_internal_sha1_process(ctx, ctx->buffer)) != 0) {
            return err;
        }

        input += fill;
        ilen -= fill;
        left = 0;
    }

    while (ilen >= 64) {
        if ((err = mbedtls_internal_sha1_process(ctx, input)) != 0) {
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
* SHA-1 final digest
*/
int mbedtls_sha1_finish_ret(mbedtls_sha1_context *ctx, unsigned char output[20])
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

        if ((err = mbedtls_internal_sha1_process(ctx, ctx->buffer)) != 0) {
            return err;
        }

        memset(ctx->buffer, 0, 56);
    }

    /*
    * Add message length (multiply by 8 to get number of bits)
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
    PUT_UINT32_BE_local(ctx->state[0], output, 0);
    PUT_UINT32_BE_local(ctx->state[1], output, 4);
    PUT_UINT32_BE_local(ctx->state[2], output, 8);
    PUT_UINT32_BE_local(ctx->state[3], output, 12);
    PUT_UINT32_BE_local(ctx->state[4], output, 16);

    return 0;
}

/*
* Internal SHA1 process
*/
int mbedtls_internal_sha1_process(mbedtls_sha1_context *ctx, const unsigned char data[64])
{
    unsigned int i;
    SHA1_VALIDATE_RET(ctx != NULL);
    SHA1_VALIDATE_RET((const unsigned char *)data != NULL);

	/* Protect context access                                  */
    /* (it may occur at a same time in a threaded environment) */
    _sha1_lock();

    /* Set first block or write intermediate hash */
    if (ctx->isfirst) {
        /* Configure the SHA */
		g_sha1_cfg.algo = SHA_ALGO_SHA1;
		g_sha1_cfg.uihv = false;
        sha_set_config(SHA, &g_sha1_cfg);

        /* No automatic padding */
        sha_set_msg_size(SHA, 0);
        sha_set_byte_count(SHA, 0);

        /* Set first block */
        sha_set_first_block(SHA);
        ctx->isfirst = false;
        sha_internal_set_current_id(ctx->id);
    } else {
        /* Write the intermediate hash value to the input data registers */
        sha_set_write_initial_val(SHA);
        sha_write_input_data(SHA, ctx->state, SHA_BLOCK_SIZE_16);
        sha_clear_write_initial_val(SHA);

        /* Reconfigure the SHA */
		g_sha1_cfg.algo = SHA_ALGO_SHA1;
        g_sha1_cfg.uihv = true;
		sha_set_config(SHA, &g_sha1_cfg);

        /* Set FIRST command again when contexts have changed */
        if (ctx->id != sha_internal_get_current_id()) {
            /* Set first block */
            sha_set_first_block(SHA);
            sha_internal_set_current_id(ctx->id);
        }
    }

    /* Write the data to be hashed to the input data registers */
    sha_write_input_data(SHA, (uint32_t *)data, SHA_BLOCK_SIZE_16);

    /* Start the SHA */
    b_sha1_state = false;
    sha_start(SHA);

    /* Wait for the end of the SHA process */
    while (false == b_sha1_state) {
    }

    /* Copy intermediate result */
    for (i = 0; i < SHA_HASH_SIZE_SHA1; i++) {
        ctx->state[i] = sha1_output_data[i];
    }

    /* Free context access */
    _sha1_unlock();

    return 0;
}

#endif /* MBEDTLS_SHA1_ALT */
#endif /* MBEDTLS_SHA1_C */
