/**
 * \file
 *
 * \brief Secure Hash Algorithm (SHA) driver.
 *
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef SHA_H_INCLUDED
#define SHA_H_INCLUDED

/**
 * \defgroup asfdoc_sam_drivers_sha_group PIC32X Secure Hash Algorithm Driver
 *
 * This driver for SMART ARM&reg;-based microcontrollers 
 * provides an interface for the configuration and management of the 
 * device's Secure Hash Algorithm functionality.
 *
 * Devices from the following series can use this module:
 * - Microchip | SMART PIC32CX
 *
 */

#include <compiler.h>
#include "pic32cx.h"

/// @cond
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/** SHA start mode. */
enum sha_start_mode_t {
	SHA_MANUAL_START = 0,   /**< Manual start mode. */
	SHA_AUTO_START,         /**< Auto start mode. */
	SHA_IDATAR0_START,      /**< SHA_IDATAR0 access only mode. */
};

/** SHA algorithm. */
enum sha_algo_t {
	SHA_ALGO_SHA1            = 0,   /**< SHA1 algorithm processed. */
	SHA_ALGO_SHA256          = 1,   /**< SHA256 algorithm processed. */
	SHA_ALGO_SHA384          = 2,   /**< SHA384 algorithm processed. */
	SHA_ALGO_SHA512          = 3,   /**< SHA512 algorithm processed. */
	SHA_ALGO_SHA224          = 4,   /**< SHA224 algorithm processed. */
	SHA_ALGO_SHA512_224      = 5,   /**< SHA512/224 algorithm processed. */
	SHA_ALGO_SHA512_256      = 6,   /**< SHA512/256 algorithm processed. */
	SHA_ALGO_HMAC_SHA1       = 8,   /**< HMAC algorithm with SHA1 Hash processed. */
	SHA_ALGO_HMAC_SHA256     = 9,   /**< HMAC algorithm with SHA256 Hash processed. */
	SHA_ALGO_HMAC_SHA384     = 10,  /**< HMAC algorithm with SHA384 Hash processed. */
	SHA_ALGO_HMAC_SHA512     = 11,  /**< HMAC algorithm with SHA512 Hash processed. */
	SHA_ALGO_HMAC_SHA224     = 12,  /**< HMAC algorithm with SHA224 Hash processed. */
	SHA_ALGO_HMAC_SHA512_224 = 13,  /**< HMAC algorithm with SHA512/224 Hash processed. */
	SHA_ALGO_HMAC_SHA512_256 = 14,  /**< HMAC algorithm with SHA512/246 Hash processed. */
};

/** SHA hash check. */
enum sha_check_t {
	SHA_NO_CHECK = 0,      /**< No check. */
	SHA_CHECK_EHV,         /**< Check with expected hash in value registers. */
	SHA_CHECK_MESSAGE,     /**< Check with expected hash after message. */
};

/** Block size in words. */
enum sha_block_size_t {
	SHA_BLOCK_SIZE_16  = 16,   /**< Bock size for SHA1, SHA224 and SHA256. */
	SHA_BLOCK_SIZE_32  = 32,   /**< Block size for SHA384, SHA512, SHA512/224 and SHA512/256. */
};

/** Hash result size in words. */
enum sha_hash_size_t {
	SHA_HASH_SIZE_SHA1     = 5,   /**< SHA1 hash size. */
	SHA_HASH_SIZE_SHA224   = 7,   /**< SHA224 and SHA512/224 hash size. */
	SHA_HASH_SIZE_SHA256   = 8,   /**< SHA256 and SHA512/256 hash size. */
	SHA_HASH_SIZE_SHA384   = 12,  /**< SHA384 hash size. */
	SHA_HASH_SIZE_SHA512   = 16,  /**< SHA512 hash size. */
};

/** Check status: identical hash values */
#define SHA_CHKST_IDENTICAL      5

/** SHA interrupt source type. */
typedef enum sha_interrupt_source {
	/** Data ready interrupt.*/
	SHA_INTERRUPT_DATA_READY = SHA_IER_DATRDY,
	/** End of transmit buffer interrupt. */
	SHA_INTERRUPT_END_OF_TRANSMIT_BUFFER = SHA_IER_ENDTX,
	/** Transmit buffer empty interrupt. */
	SHA_INTERRUPT_TRANSMIT_BUFFER_EMPTY = SHA_IER_TXBUFE,	
	/** Unspecified register access detection interrupt.*/
	SHA_INTERRUPT_UNSPECIFIED_REGISTER_ACCESS = SHA_IER_URAD,
	/** Check done interrupt. */
	SHA_INTERRUPT_CHECK_DONE = SHA_IER_CHECKF,
	/** Security and/or safety event interrupt. */
	SHA_INTERRUPT_SECURITY_SAFETY_EVENT = SHA_IER_SECE,
	/** Input data register write ready interrupt. */
	SHA_INTERRUPT_IDATAR0_WRITE_READY,
} sha_interrupt_source_t;

/** \internal Max number of interrupt sources. */
#define SHA_INTERRUPT_SOURCE_NUM    7

/** SHA interrupt callback function type. */
typedef void (*sha_callback_t)(uint8_t);

/** SHA Configuration structure. */
struct sha_config {
	/** Start mode. */
	enum sha_start_mode_t start_mode;
	/** Always ON enable. */
	bool aoe;
	/** Processing delay. */
	bool procdly;
	/** User initial hash values. */
	bool uihv;
	/** User initial or expected hash value registers. */
	bool uiehv;
	/** Block processing end. */
	bool bpe;
	/** SHA algorithm. */
	enum sha_algo_t algo;
	/** Tamper lock enable. */
	bool tmpclk;
	/** Dual input buffer. */
	bool dualbuff;
	/* Hash check. */
	enum sha_check_t check;
	/* Check counter. */
	uint8_t chkcnt;
};

void sha_get_config_defaults(struct sha_config *p_cfg);

void sha_init(Sha *p_sha, struct sha_config *p_cfg);

/**
 * \brief Perform a SHA software reset.
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_reset(Sha *p_sha)
{
	p_sha->SHA_CR = SHA_CR_SWRST;
}

/**
 * \brief Start a manual process.
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_start(Sha *p_sha)
{
	p_sha->SHA_CR = SHA_CR_START;
}

/**
 * \brief Set first block of a message.
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_set_first_block(Sha *p_sha)
{
	p_sha->SHA_CR = SHA_CR_FIRST;
}

/**
 * \brief Set write user initial hash values
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_set_write_initial_val(Sha *p_sha)
{
	p_sha->SHA_CR = SHA_CR_WUIHV;
}

/**
 * \brief Set write user initial hash values or expectect hash values
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_set_write_initial_expected_val(Sha *p_sha)
{
	p_sha->SHA_CR = SHA_CR_WUIEHV;
}

/**
 * \brief Clear write user initial hash values
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_clear_write_initial_val(Sha *p_sha)
{
	p_sha->SHA_CR &= ~SHA_CR_WUIHV;
}

/**
 * \brief Clear write user initial hash values or expectect hash values
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_clear_write_initial_expected_val(Sha *p_sha)
{
	p_sha->SHA_CR &= ~SHA_CR_WUIEHV;
}

/**
 * \brief Perform a SHA unlock.
 *
 * \param p_sha Module hardware register base address pointer
 */
static inline void sha_unlock(Sha *p_sha)
{
	p_sha->SHA_CR = SHA_CR_UNLOCK;
}

void sha_enable(void);

void sha_disable(void);

void sha_set_config(Sha *p_aes, struct sha_config *p_cfg);

/**
 * \brief Get the SHA interrupt status.
 *
 * \param p_sha Module hardware register base address pointer
 *
 * \return The SHA interrupt status register contents.
 */
static inline uint32_t sha_read_interrupt_status(Sha *p_sha)
{
	return p_sha->SHA_ISR;
}

void sha_set_callback(Sha *p_sha, sha_interrupt_source_t source, sha_callback_t callback, uint8_t irq_level);

/**
 * \brief Enable a SHA interrupt.
 *
 * \param p_sha     Module hardware register base address pointer
 * \param source    Interrupt source
 */
static inline void sha_enable_interrupt(Sha *p_sha, sha_interrupt_source_t source)
{
	if (source != SHA_INTERRUPT_IDATAR0_WRITE_READY) {
		p_sha->SHA_IER = (uint32_t)source;
	}
}

/**
 * \brief Disable a SHA interrupt.
 *
 * \param p_sha     Module hardware register base address pointer
 * \param source    Interrupt source
 */
static inline void sha_disable_interrupt(Sha *p_sha, sha_interrupt_source_t source)
{
	if (source != SHA_INTERRUPT_IDATAR0_WRITE_READY) {
		p_sha->SHA_IDR = (uint32_t)source;
	}
}

/**
 * \brief Get the SHA interrupt mask status.
 *
 * \param p_sha Module hardware register base address pointer
 *
 * \return The SHA interrupt mask contents.
 */
static inline uint32_t sha_read_interrupt_mask(Sha *p_sha)
{
	return p_sha->SHA_IMR;
}

void sha_set_msg_size(Sha *p_sha, uint32_t ul_size); 
void sha_set_byte_count(Sha *p_sha, uint32_t ul_count);  
uint32_t sha_get_byte_count(Sha *p_sha);  
void sha_write_input_data(Sha *p_sha, uint32_t *p_input_data_buffer, uint8_t uc_len);
void sha_read_output_data(Sha *p_sha, uint32_t *p_output_data_buffer);

Pdc *sha_get_pdc_base(Sha *p_sha);

void sha_set_writeprotect(Sha *p_sha, bool enable, bool int_enable, bool control_enable, bool first_error_report_enable, uint8_t uc_action);

uint32_t sha_get_writeprotect_status(Sha *p_sha);

/// @cond
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

/** @} */

#endif /* SHA_H_INCLUDED */
