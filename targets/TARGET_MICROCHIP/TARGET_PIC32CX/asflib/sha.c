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

#include "sha.h"
#include <sysclk.h>
#include <interrupt_sam_nvic.h>

/// @cond
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#ifndef SHA_WPMR_WPKEY_PASSWD
#define SHA_WPMR_WPKEY_PASSWD SHA_WPMR_WPKEY((uint32_t)0x534841)
#endif

/**
 * \internal
 * \brief SHA callback function pointer
 */
sha_callback_t sha_callback_pointer[SHA_INTERRUPT_SOURCE_NUM];

/**
 * \brief Initializes a SHA configuration structure to defaults.
 *
 * Initializes the specified SHA configuration structure to a set of
 * known default values.
 *
 * \note This function should be called to initialize <i>all</i> new instances of
 * SHA configuration structures before they are further modified by the user
 * application.
 *
 *  The default configuration is as follows:
 *  - Manual start mode
 *  - Always ON disabled
 *  - Shortest processing delay
 *  - SHA algorithm started with standard initial values
 *  - Block processing end disabled
 *  - SHA1 algorithm processed
 *  - Tamper lock disabled
 *  - No dual input
 *  - No hash check
 *  - Number of words to check based on selected algorithm
 *
 *  \param p_cfg Pointer to a SHA configuration structure
 */
void sha_get_config_defaults(struct sha_config *p_cfg)
{
	/* Sanity check arguments */
	Assert(p_cfg);

	/* Default configuration values */
	p_cfg->start_mode = SHA_MANUAL_START;
	p_cfg->aoe = false;
	p_cfg->procdly = false;
	p_cfg->uihv = false;
	p_cfg->uiehv = false;
	p_cfg->bpe = false;
	p_cfg->algo = SHA_ALGO_SHA1;
	p_cfg->tmpclk = false;
	p_cfg->dualbuff = false;
	p_cfg->check = SHA_NO_CHECK;
	p_cfg->chkcnt = 0;
}

/**
 * \brief Initialize the SHA module.
 *
 * \param p_sha  Module hardware register base address pointer
 * \param p_cfg  Pointer to a SHA configuration structure
 */
void sha_init(Sha *p_sha, struct sha_config *p_cfg)
{
	/* Sanity check arguments */
	Assert(p_sha);
	Assert(p_cfg);

	/* Enable clock for SHA */
	sysclk_enable_peripheral_clock(ID_SHA);

	/* Perform a software reset */
	sha_reset(p_sha);

	/* Initialize the SHA with new configurations */
	sha_set_config(p_sha, p_cfg);

	/* Disable clock for SHA */
	sysclk_disable_peripheral_clock(ID_SHA);
}

/**
 * \brief Enable the SHA module.
 */
void sha_enable(void)
{
	sysclk_enable_peripheral_clock(ID_SHA);
}

/**
 * \brief Disable the SHA module.
 */
void sha_disable(void)
{
	sysclk_disable_peripheral_clock(ID_SHA);
}

/**
 * \brief Configure the SHA module.
 *
 * \param p_sha  Module hardware register base address pointer
 * \param p_cfg  Pointer to an AES configuration structure
 */
void sha_set_config(Sha *p_sha, struct sha_config *p_cfg)
{
	uint32_t ul_mode = 0;

	/* Validate arguments. */
	Assert(p_sha);
	Assert(p_cfg);

	/* Set start mode */
	ul_mode |= SHA_MR_SMOD(p_cfg->start_mode);

	/* Set always on */
	if (p_cfg->aoe) {
		ul_mode |= SHA_MR_AOE;
	}

	/* Set long processing delay */
	if (p_cfg->procdly) {
		ul_mode |= SHA_MR_PROCDLY_LONGEST;
	}

	/* Set user initial hash values */
	if (p_cfg->uihv) {
		ul_mode |= SHA_MR_UIHV;
	}

	/* Set user initial or expected hash value registers  */
	if (p_cfg->uiehv) {
		ul_mode |= SHA_MR_UIEHV;
	}

	/* Set block procecssing end */
	if (p_cfg->bpe) {
		ul_mode |= SHA_MR_BPE;
	}

	/* Set SHA algorithm */
	ul_mode |= SHA_MR_ALGO(p_cfg->algo);

	/* Set tamper lock enable */
	if (p_cfg->tmpclk) {
		ul_mode |= SHA_MR_TMPLCK;
	}

	/* Set dual buffer */
	if (p_cfg->dualbuff) {
		ul_mode |= SHA_MR_DUALBUFF_ACTIVE;
	}

	/* Set hash check */
	ul_mode |= SHA_MR_CHECK(p_cfg->check);

	/* Set check counter */
	ul_mode |= SHA_MR_CHKCNT(p_cfg->chkcnt);

	p_sha->SHA_MR = ul_mode;
}

/**
 * \brief Enable or disable write protection of SHA registers.
 *
 * \param p_sha                     Pointer to a SHA instance.
 * \param enable                    1 to enable, 0 to disable
 * \param int_enable                1 to enable, 0 to disable
 * \param control_enable            1 to enable, 0 to disable
 * \param first_error_report_enable 1 to enable, 0 to disable
 * \param uc_action                 Action on abnormal event detection
 */
void sha_set_writeprotect(Sha *p_sha, bool enable, bool int_enable, bool control_enable, bool first_error_report_enable, uint8_t uc_action)
{
	uint32_t ul_reg;

	ul_reg = SHA_WPMR_WPKEY_PASSWD;

	if (enable) {
		ul_reg |= SHA_WPMR_WPEN;
	}

	if (int_enable) {
		ul_reg |= SHA_WPMR_WPITEN;
	}

	if (control_enable) {
		ul_reg |= SHA_WPMR_WPCREN;
	}

	if (first_error_report_enable) {
		ul_reg |= SHA_WPMR_FIRSTE;
	}

	ul_reg |= SHA_WPMR_ACTION(uc_action);

	p_sha->SHA_WPMR = ul_reg;
}


/**
 * \brief Indicate write protect status.
 *
 * \param p_sha Pointer to a SHA instance.
 *
 * \return Write protect status.
 */
uint32_t sha_get_writeprotect_status(Sha *p_sha)
{
	return (p_sha->SHA_WPSR);
}

/**
 * \brief Set message size.
 *
 * \param p_sha      Pointer to a SHA instance.
 * \param ul_size    Message size.
 */
void sha_set_msg_size(Sha *p_sha, uint32_t ul_size)
{
	p_sha->SHA_MSR = ul_size;
}

/**
 * \brief Set remaining byte count before auto padding.
 *
 * \param p_sha      Pointer to a SHA instance.
 * \param ul_count   Byte count.
 */
void sha_set_byte_count(Sha *p_sha, uint32_t ul_count)
{
	p_sha->SHA_BCR = ul_count;
}

/**
 * \brief Get remaining byte count before auto padding.
 *
 * \param p_sha      Pointer to a SHA instance.
 *
 * \return Remaining byte count before auto padding.
 */
uint32_t sha_get_byte_count(Sha *p_sha)
{
	return (p_sha->SHA_BCR);
}

/**
 * \brief Write the input data (16/32 consecutive 32-bit words).
 *
 * \note For SHA1, SHA224 and SHA256, block size is 512 (length = 16).
 * For SHA384, SHA512, SHA412/224 and SHA512/256, block size is 1024 (length = 32).
 * For initial and expected hash values:
 *	- length = 5 for SHA1
 *	- length = 7 for SHA224
 *	- length = 8 for SHA256
 *	- length = 12 for SHA384
 *	- length = 16 for SHA512
 *
 * \param p_sha                  Pointer to a SHA instance.
 * \param p_input_data_buffer    Pointer to an input data buffer.
 *\ param uc_len                 Block length or initial/expected hash result length
 */
void sha_write_input_data(Sha *p_sha, uint32_t *p_input_data_buffer, uint8_t uc_len)
{
	uint8_t i;
	uint8_t uc_len1 = 0;
	uint8_t uc_len2 = 0;

	/* Validate arguments. */
	Assert(p_sha);
	Assert(p_input_data_buffer);

	if (uc_len <= 16) {
		uc_len1 = uc_len;
	} else {
		uc_len1 = 16;
		uc_len2 = 16;
	}

	for (i = 0; i < uc_len1; i++) {
			p_sha->SHA_IDATAR[i] = *p_input_data_buffer;
			p_input_data_buffer++;
	}

	if (uc_len2) {
		for (i = 0; i < 16; i++) {
			p_sha->SHA_IODATAR[i] = *p_input_data_buffer;
			p_input_data_buffer++;
		}
	}
}

/**
 * \brief Read the output data.
 *
 * \note The data buffer that holds the processed data must be large enough to hold
 * up to 16 consecutive 32-bit words.
 *
 * \param p_sha                  Pointer to a SHA instance.
 * \param p_output_data_buffer   Pointer to an output buffer.
 */
void sha_read_output_data(Sha *p_sha, uint32_t *p_output_data_buffer)
{
	uint32_t i;

	/* Validate arguments. */
	Assert(p_sha);
	Assert(p_output_data_buffer);

	for (i = 0; i < 16; i++) {
		*p_output_data_buffer = p_sha->SHA_IODATAR[i];
		p_output_data_buffer++;
	}
}

/**
 * \brief Get SHA PDC base address.
 *
 * \param p_sha Pointer to a SHA instance.
 *
 * \return The PDC registers base address for the SHA module.
 */
Pdc *sha_get_pdc_base(Sha *p_sha)
{
	/* Validate arguments. */
	Assert(p_sha);

	Pdc *p_pdc_base;
	if (p_sha == SHA) {
		p_pdc_base = PDC_SHA;
	} else {
		p_pdc_base = NULL;
	}

	return p_pdc_base;
}

/**
 * \brief Set the SHA interrupt callback.
 *
 * \param p_sha     Pointer to a SHA instance.
 * \param source    Interrupt source
 * \param callback  Interrupt callback function pointer
 * \param irq_level Interrupt priority level
 */
void sha_set_callback(Sha *p_sha, sha_interrupt_source_t source, sha_callback_t callback, uint8_t irq_level)
{
	/* Validate arguments. */
	Assert(p_sha);

	switch(source) {
	case SHA_INTERRUPT_DATA_READY:
		sha_callback_pointer[0] = callback;
		break;

	case SHA_INTERRUPT_END_OF_TRANSMIT_BUFFER:
		sha_callback_pointer[1] = callback;
		break;

	case SHA_INTERRUPT_TRANSMIT_BUFFER_EMPTY:
		sha_callback_pointer[2] = callback;
		break;

	case SHA_INTERRUPT_UNSPECIFIED_REGISTER_ACCESS:
		sha_callback_pointer[3] = callback;
		break;

	case SHA_INTERRUPT_CHECK_DONE:
		sha_callback_pointer[4] = callback;
		break;

	case SHA_INTERRUPT_SECURITY_SAFETY_EVENT:
		sha_callback_pointer[5] = callback;
		break;

	case SHA_INTERRUPT_IDATAR0_WRITE_READY:
		 sha_callback_pointer[6] = callback;
		break;

	default:
		return;
	}

	irq_register_handler((IRQn_Type)SHA_IRQn, irq_level);
	sha_enable_interrupt(p_sha, source);
}

/**
 * \internal The SHA interrupt handler.
 */
void SHA_Handler(void)
{
	uint32_t status = sha_read_interrupt_status(SHA);
	uint32_t mask = sha_read_interrupt_mask(SHA);

	if ((status & SHA_ISR_DATRDY) && (mask & SHA_IMR_DATRDY)) {
		if (sha_callback_pointer[0]) {
			sha_callback_pointer[0](0);
		}
	}

	if ((status & SHA_ISR_ENDTX) && (mask & SHA_IMR_ENDTX)) {
		if (sha_callback_pointer[1]) {
			sha_callback_pointer[1](0);
		}
	}

	if ((status & SHA_ISR_TXBUFE) && (mask & SHA_IMR_TXBUFE)) {
		if (sha_callback_pointer[2]) {
			sha_callback_pointer[2](0);
		}
	}

	if ((status & SHA_ISR_URAD) && (mask & SHA_IMR_URAD)) {
		if (sha_callback_pointer[3]) {
			sha_callback_pointer[3]((status & SHA_ISR_URAT_Msk) >> SHA_ISR_URAT_Pos);
		}
	}

	if ((status & SHA_ISR_CHECKF) && (mask & SHA_IMR_CHECKF)) {
		if (sha_callback_pointer[4]) {
			sha_callback_pointer[4]((status & SHA_ISR_CHKST_Msk) >> SHA_ISR_CHKST_Pos);
		}
	}

	if ((status & SHA_ISR_SECE) && (mask & SHA_IMR_SECE)) {
		if (sha_callback_pointer[5]) {
			sha_callback_pointer[5](0);
		}
	}

	if (status & SHA_ISR_WRDY) {
		if (sha_callback_pointer[6]) {
			sha_callback_pointer[6](0);
		}
	}
}
/// @cond
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
