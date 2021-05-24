/**
 * \file
 *
 * \brief Analog-to-Digital Converter (ADC/ADC12B) driver for SAM.
 *
 * Copyright (c) 2011-2020 Microchip Technology Inc. and its subsidiaries.
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

#include "adc.h"
#include <status_codes.h>
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

const PioGroup *adc_piogroup[] = {PIOA, PIOA, PIOA, PIOB, PIOB};
const uint32_t adc_piomask[] = {PIO_PA29, PIO_PA30, PIO_PA31, PIO_PB0, PIO_PB1};

static volatile uint32_t adc_imr_reg = 0;

/**
 * \defgroup sam_drivers_adc_group Analog-to-digital Converter (ADC)
 *
 * See \ref sam_adc_quickstart.
 *
 * Driver for the Analog-to-digital Converter. This driver provides access to the main
 * features of the ADC controller.
 *
 * @{
 */

/**
 * \brief Initialize the given ADC with the specified ADC clock and startup time.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_mck Main clock of the device (value in Hz).
 * \param ul_adc_clock Analog-to-Digital conversion clock (value in Hz).
 * \param uc_startup ADC start up time. Please refer to the product datasheet
 * for details.
 *
 * \return 0 on success.
 */
uint32_t adc_init(Adc *p_adc, const uint32_t ul_mck,
        const uint32_t ul_adc_clock, const enum adc_startup_time startup)
{
    uint32_t ul_prescal;

    /*  Reset the controller. */
    p_adc->ADC_CR = ADC_CR_SWRST;

    /* Reset Mode Register. */
    p_adc->ADC_MR = ADC_MR_TRANSFER(2) | ADC_MR_ALWAYS1;

    /* Reset PDC transfer. */
    p_adc->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS);
    p_adc->ADC_RCR = 0;
    p_adc->ADC_RNCR = 0;

    ul_prescal = ul_mck / (2 * ul_adc_clock) - 1;
    p_adc->ADC_MR |=  ADC_MR_PRESCAL(ul_prescal) | startup;

    return 0;
}

/**
 * \brief Configure the conversion resolution.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param resolution ADC resolution.
 *
 */
void adc_set_resolution(Adc *p_adc, const enum adc_resolution_t resolution)
{
    p_adc->ADC_EMR &= ~ADC_EMR_OSR_Msk;
    p_adc->ADC_EMR |= resolution;
}


/**
 * \brief Configure conversion trigger and free run mode.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param trigger Conversion trigger.
 */
void adc_configure_trigger(Adc *p_adc, const enum adc_trigger_t trigger)
{
    p_adc->ADC_MR &= ~ADC_MR_TRGSEL_Msk;
    p_adc->ADC_MR |= trigger;
}

/**
 * \brief Configures ADC power saving mode.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_sleep ADC_MR_SLEEP_NORMAL keeps the ADC Core and reference voltage
 * circuitry ON between conversions.
 * ADC_MR_SLEEP_SLEEP keeps the ADC Core and reference voltage circuitry OFF
 * between conversions.
 * \param uc_fwup ADC_MR_FWUP_OFF configures sleep mode as uc_sleep setting,
 * ADC_MR_FWUP_ON keeps voltage reference ON and ADC Core OFF between conversions.
 */
void adc_configure_power_save(Adc *p_adc, const uint8_t uc_sleep, const uint8_t uc_fwup)
{
    p_adc->ADC_MR |= (((uc_sleep << 5) & ADC_MR_SLEEP) | ((uc_fwup << 6) & ADC_MR_FWUP));
}

/**
 * \brief Configure conversion sequence.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ch_list Channel sequence list.
 * \param number Number of channels in the list.
 */
void adc_configure_sequence(Adc *p_adc, const enum adc_channel_num_t ch_list[],
        uint8_t uc_num)
{
    uint8_t uc_counter;
    volatile uint32_t *adc_seqr = &p_adc->ADC_SEQR1;

    if (uc_num <= 8) {
        for (uc_counter = 0; uc_counter < uc_num; uc_counter++) {
            adc_seqr[0] |= ch_list[uc_counter] << (4 * uc_counter);
        }
    } else {
        for (uc_counter = 0; uc_counter < 8; uc_counter++) {
            adc_seqr[0] |= ch_list[uc_counter] << (4 * uc_counter);
        }
        for (uc_counter = 0; uc_counter < uc_num - 8; uc_counter++) {
            adc_seqr[1] |= ch_list[8 + uc_counter] << (4 * uc_counter);
        }
    }
}

/**
 * \brief Configure ADC timing.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_tracking ADC tracking time.
 * \param uc_transfer Data transfer time.
 * \note Transfer Time must be set to 2 to ensure the optimal transfer time
 */
void adc_configure_timing(Adc *p_adc, const uint8_t uc_tracking, const uint8_t uc_transfer)
{
    p_adc->ADC_MR |= ADC_MR_TRANSFER(2) | ADC_MR_TRACKTIM(uc_tracking);
}

/**
 * \brief Enable FIFO.
 *
 * \param p_Adc Pointer to an ADC instance.
 */
void adc_enable_fifo(Adc *p_adc)
{
    p_adc->ADC_FMR |= ADC_FMR_ENFIFO;
}

/**
 * \brief Disable FIFO.
 *
 * \param p_Adc Pointer to an ADC instance.
 */
void adc_disable_fifo(Adc *p_adc)
{
    p_adc->ADC_FMR &= ~ADC_FMR_ENFIFO;
}

/**
 * \brief Enable level for FIFO.
 *
 * \param p_Adc Pointer to an ADC instance.
 */
void adc_enable_level(Adc *p_adc)
{
    p_adc->ADC_FMR |= ADC_FMR_ENLEVEL;
}

/**
 * \brief Disable level for FIFO.
 *
 * \param p_Adc Pointer to an ADC instance.
 */
void adc_disable_level(Adc *p_adc)
{
    p_adc->ADC_FMR &= ~ADC_FMR_ENLEVEL;
}

/**
 * \brief Configure chunk size for FIFO.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_chunk ADC chunk size.
 */
void adc_set_chunk(Adc *p_adc, const uint8_t uc_chunk)
{
    if ((uc_chunk >= 1) && (uc_chunk <= 15)) {
        p_adc->ADC_FMR |= ADC_FMR_CHUNK(uc_chunk);
    }
}

/**
 * \brief Configure FIFO count.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_cnt ADC FIFO count.
 */
void adc_set_fifo_count(Adc *p_adc, const uint8_t uc_cnt)
{
    p_adc->ADC_FMR |= ADC_FMR_FIFOCNT(uc_cnt);
}

/**
 * \brief Set ADC trigger.
 *
 * \param p_adc     Pointer to an ADC instance.
 * \param uc_mode   ADC trigger mode.
 * \param ul_period ADC trigger period.
 */
void adc_set_trigger(Adc *p_adc, const uint8_t uc_mode, const uint32_t ul_period)
{
    p_adc->ADC_TRGR = ADC_TRGR_TRGMOD(uc_mode);
    if (uc_mode == ADC_TRGR_TRGMOD_PERIOD_TRIG) {
        p_adc->ADC_TRGR |= ADC_TRGR_TRGPER(ul_period);
    }
}

/**
 * \brief Set ADC channel correction select.
 *
 * \param p_adc       Pointer to an ADC instance.
 * \param uc_channel  ADC channel.
 */
void adc_select_channel_correction(Adc *p_adc, const uint8_t uc_channel)
{
    p_adc->ADC_COSR = ADC_COSR_CSEL(uc_channel);
}

/**
 * \brief Set ADC correction values.
 *
 * \param p_adc       Pointer to an ADC instance.
 * \param us_offset   ADC offset correction.
 * \param us_gain     ADC gain correction.
 */
void adc_set_correction_values(Adc *p_adc, const uint16_t us_offset, const uint16_t us_gain)
{
    p_adc->ADC_CVR = ADC_CVR_OFFSETCORR(us_offset);
    p_adc->ADC_CVR = ADC_CVR_GAINCORR(us_gain);
}

/**
 * \brief Enable automatic error correction for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param channel ADC channel number.
 */
void adc_enable_automatic_error_correction(Adc *p_adc, const enum adc_channel_num_t channel)
{
    p_adc->ADC_CECR |= (0x01u << channel);
}

/**
 * \brief Get ADC status of VDD supply monitor output.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return VDD ADC voltage validity.
 */
uint32_t adc_get_sm_status(const Adc *p_adc)
{
    return p_adc->ADC_SR;
}

/**
 * \brief Enable analog change.
 *
 * \note It allows different analog settings for each channel.
 *
 * \param p_Adc Pointer to an ADC instance.
 */
void adc_enable_anch(Adc *p_adc)
{
    p_adc->ADC_MR |= ADC_MR_ANACH;
}

/**
 * \brief Disable analog change.
 *
 * \note DIFF0, GAIN0 and OFF0 are used for all channels.
 *
 * \param p_Adc Pointer to an ADC instance.
 */
void adc_disable_anch(Adc *p_adc)
{
    p_adc->ADC_MR &= ~ADC_MR_ANACH;
}

/**
 * \brief Enable differential input for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param channel ADC channel number.
 */
void adc_enable_channel_differential_input(Adc *p_adc, const enum adc_channel_num_t channel)
{
    p_adc->ADC_CCR |= (0x01u << channel);
}

/**
 * \brief Disable differential input for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param channel ADC channel number.
 */
void adc_disable_channel_differential_input(Adc *p_adc, const enum adc_channel_num_t channel)
{
    uint32_t ul_temp;
    ul_temp = p_adc->ADC_CCR;
    p_adc->ADC_CCR &= (0xfffffffeu << channel);
    p_adc->ADC_CCR |= ul_temp;
}

/**
 * \brief Enable ADC Internal Positive Voltage Reference.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_set_int_ref_voltage(Adc *p_adc)
{
    p_adc->ADC_ACR |= ADC_ACR_INTVREFEN;
}

/**
 * \brief Enable ADC VBAT load.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_set_battery(Adc *p_adc)
{
    p_adc->ADC_ACR |= ADC_ACR_ZBAT;
}

/**
 * \brief Enable ADC supply monitor.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_set_sm(Adc *p_adc)
{
    p_adc->ADC_ACR |= ADC_ACR_SMEN;
}

/**
 * \brief Enable ADC supply monitor voltage threshold.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_set_sm_threshold(Adc *p_adc)
{
    p_adc->ADC_ACR |= ADC_ACR_SMVT;
}

/**
 * \brief Start analog-to-digital conversion.
 *
 * \note If one of the hardware event is selected as ADC trigger,
 * this function can NOT start analog to digital conversion.
 *
 * \param p_adc Pointer to an ADC instance.
 */

void adc_start(Adc *p_adc)
{
    p_adc->ADC_CR = ADC_CR_START;
}

/**
 * \brief Reset ADC.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_reset(Adc *p_adc)
{
    p_adc->ADC_CR = ADC_CR_SWRST;
}

/**
 * \brief Reset internal FIFO.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_reset_fifo(Adc *p_adc)
{
    p_adc->ADC_CR = ADC_CR_SWFIFO;
}

/**
 * \brief Restart comparison.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_restart_comparison(Adc *p_adc)
{
    p_adc->ADC_CR = ADC_CR_CMPRST;
}

/**
 * \brief Enable the specified ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 */
void adc_enable_channel(Adc *p_adc, const enum adc_channel_num_t adc_ch)
{
    /* Disable Pull Up/Down */
    if (adc_ch < ADC_BATTERY_VOLTAGE) {
        PioGroup *p_pio_group;

        p_pio_group = (PioGroup *)adc_piogroup[adc_ch];

        /* Select mask */
        p_pio_group->PIO_MSKR = adc_piomask[adc_ch];
        /* Disable Pull-up / Pull-down */
        p_pio_group->PIO_CFGR &= ~(PIO_CFGR_PUEN | PIO_CFGR_PDEN);
    }

    p_adc->ADC_CHER = 1 << adc_ch;
}

/**
 * \brief Enable all ADC channels.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_enable_all_channel(Adc *p_adc)
{
    p_adc->ADC_CHER = 0xFF;
}

/**
 * \brief Disable the specified ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 */
void adc_disable_channel(Adc *p_adc, const enum adc_channel_num_t adc_ch)
{
    p_adc->ADC_CHDR = 1 << adc_ch;
}

/**
 * \brief Disable all ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_disable_all_channel(Adc *p_adc)
{
    p_adc->ADC_CHDR = 0xFF;
}

/**
 * \brief Read the ADC channel status.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 *
 * \retval 1 if channel is enabled.
 * \retval 0 if channel is disabled.
 */
uint32_t adc_get_channel_status(const Adc *p_adc, const enum adc_channel_num_t adc_ch)
{
    return p_adc->ADC_CHSR & (1 << adc_ch);
}

/**
 * \brief Read the ADC result data of the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 *
 * \return ADC value of the specified channel.
 */
uint32_t adc_get_channel_value(const Adc *p_adc, const enum adc_channel_num_t adc_ch)
{
    uint32_t ul_data = 0;

    if (7 >= adc_ch) {
        ul_data = *(p_adc->ADC_CDR + adc_ch);
    }

    return ul_data;
}

/**
 * \brief Read the last ADC result data.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC latest value.
 */
uint32_t adc_get_latest_value(const Adc *p_adc)
{
    return p_adc->ADC_LCDR;
}

/**
 * \brief Enable TAG option so that the number of the last converted channel
 * can be indicated.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_enable_tag(Adc *p_adc)
{
    p_adc->ADC_EMR |= ADC_EMR_TAG;
}

/**
 * \brief Disable TAG option.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_disable_tag(Adc *p_adc)
{
    p_adc->ADC_EMR &= ~ADC_EMR_TAG;
}

/**
 * \brief Indicate the last converted channel.
 *
 * \note If TAG option is NOT enabled before, an incorrect channel
 * number is returned.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return The last converted channel number.
 */
enum adc_channel_num_t adc_get_tag(const Adc *p_adc)
{
    /* Check for oversampling mode */
    if ((p_adc->ADC_EMR & ADC_EMR_OSR_Msk) >> ADC_EMR_OSR_Pos) {
        return (enum adc_channel_num_t)
            ((p_adc->ADC_LCDR & ADC_LCDR_CHNBOSR_Msk) >> ADC_LCDR_CHNBOSR_Pos);
    } else {
        return (enum adc_channel_num_t)
            ((p_adc->ADC_LCDR & ADC_LCDR_NO_OSR_CHNB_Msk) >> ADC_LCDR_NO_OSR_CHNB_Pos);
    }
}

/**
 * \brief Enable conversion sequencer.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_start_sequencer(Adc *p_adc)
{
    p_adc->ADC_MR |= ADC_MR_USEQ;
}

/**
 * \brief Disable conversion sequencer.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_stop_sequencer(Adc *p_adc)
{
    p_adc->ADC_MR &= ~ADC_MR_USEQ;
}

/**
 * \brief Configure comparison mode.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_mode ADC comparison mode.
 */
void adc_set_comparison_mode(Adc *p_adc, const uint8_t uc_mode)
{
    p_adc->ADC_EMR &= (uint32_t) ~ (ADC_EMR_CMPMODE_Msk);
    p_adc->ADC_EMR |= (uc_mode & ADC_EMR_CMPMODE_Msk);
}

/**
 * \brief Get comparison mode.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \retval Compare mode value.
 */
uint32_t adc_get_comparison_mode(const Adc *p_adc)
{
    return p_adc->ADC_EMR & ADC_EMR_CMPMODE_Msk;
}

/**
 * \brief Configure comparison type.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param type ADC comparison type: comparison conditions must be met if true.
 */
void adc_set_comparison_type(Adc *p_adc, bool type)
{
    if (type) {
        p_adc->ADC_EMR |= ADC_EMR_CMPTYPE;
    } else {
        p_adc->ADC_EMR &= ~ADC_EMR_CMPTYPE;
    }
}

/**
 * \brief Configure external clock selection.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param type ADC external clock selection: GCLK is the source clock if true. The
 * peripheral clock is the source if false.
 */
void adc_set_clock_selection(Adc *p_adc, bool ext_clk_sel)
{
    if (ext_clk_sel) {
        p_adc->ADC_EMR |= ADC_EMR_SRCCLK;
    } else {
        p_adc->ADC_EMR &= ~ADC_EMR_SRCCLK;
    }
}

/**
 * \brief Configure tracking time x4, x8, x16.
 *
 * \param p_adc          Pointer to an ADC instance.
 * \param tracking_timex ADC tracking time
 */
void adc_set_tracking_timex(Adc *p_adc, const uint8_t tracking_timex)
{
    p_adc->ADC_EMR |= ADC_EMR_TRACKX(tracking_timex);
}

/**
 * \brief Configure sing mode.
 *
 * \param p_adc     Pointer to an ADC instance.
 * \param sign_mode ADC sign mode
 */
void adc_set_sign_mode(Adc *p_adc, const uint8_t sign_mode)
{
    p_adc->ADC_EMR |= ADC_EMR_SIGNMODE(sign_mode);
}

/**
 * \brief Configure running mode.
 *
 * \param p_adc        Pointer to an ADC instance.
 * \param running_mode ADC running mode
 */
void adc_set_running_mode(Adc *p_adc, const uint8_t running_mode)
{
    p_adc->ADC_EMR |= ADC_EMR_ADCMODE(running_mode);
}

/**
 * \brief Configure ADC compare window.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param w_low_threshold Low threshold of compare window.
 * \param w_high_threshold High threshold of compare window.
 */
void adc_set_comparison_window(Adc *p_adc, const uint16_t us_low_threshold,
        const uint16_t us_high_threshold)
{
    p_adc->ADC_CWR = ADC_CWR_LOWTHRES(us_low_threshold) |
            ADC_CWR_HIGHTHRES(us_high_threshold);
}

/**
 * \brief Configure comparison selected channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param channel ADC channel number.
 */
void adc_set_comparison_channel(Adc *p_adc, const enum adc_channel_num_t channel)
{
    if (channel < 16) {
        p_adc->ADC_EMR &= (uint32_t) ~ (ADC_EMR_CMPALL);
        p_adc->ADC_EMR &= (uint32_t) ~ (ADC_EMR_CMPSEL_Msk);
        p_adc->ADC_EMR |= (channel << ADC_EMR_CMPSEL_Pos);
    } else {
        p_adc->ADC_EMR |= ADC_EMR_CMPALL;
    }
}

/**
 * \brief Return the actual ADC clock.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_mck Main clock of the device (in Hz).
 *
 * \return The actual ADC clock (in Hz).
 */
uint32_t adc_get_actual_adc_clock(const Adc *p_adc, const uint32_t ul_mck)
{
    uint32_t ul_adcfreq;
    uint32_t ul_prescal;

    /* ADCClock = MCK / ( (PRESCAL+1) * 2 ) */
    ul_prescal = ((p_adc->ADC_MR & ADC_MR_PRESCAL_Msk) >> ADC_MR_PRESCAL_Pos);
    ul_adcfreq = ul_mck / ((ul_prescal + 1) * 2);
    return ul_adcfreq;
}

/**
 * \brief Enable ADC interrupts.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_source Interrupts to be enabled.
 * \Note PIC32CX knowed issues: some bits of ADC_IMR are not possible to read
 */
void adc_enable_interrupt(Adc *p_adc, const uint32_t ul_source)
{
    adc_imr_reg |= ul_source;
    p_adc->ADC_IER = ul_source;
}

/**
 * \brief Disable ADC interrupts.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_source Interrupts to be disabled.
 * \Note PIC32CX knowed issues: some bits of ADC_IMR are not possible to read
 */
void adc_disable_interrupt(Adc *p_adc, const uint32_t ul_source)
{
    adc_imr_reg &= (~ul_source);
    p_adc->ADC_IDR = ul_source;
}

/**
 * \brief Get ADC interrupt and overrun error status.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC status structure.
 */
uint32_t adc_get_status(const Adc *p_adc)
{
    return p_adc->ADC_ISR;
}

/**
 * \brief Get ADC interrupt and overrun error status.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC status structure.
 */
uint32_t adc_get_overrun_status(const Adc *p_adc)
{
    return p_adc->ADC_OVER;
}

/**
 * \brief Read ADC interrupt mask.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return The interrupt mask value.
 * \Note PIC32CX knowed issues: some bits of ADC_IMR are not possible to read
 */
uint32_t adc_get_interrupt_mask(const Adc *p_adc)
{
    (void)p_adc;
    return adc_imr_reg;
}

/**
 * \brief Enable ADC EOC interrupts.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_source Interrupts to be enabled.
 */
void adc_enable_eoc_interrupt(Adc *p_adc, const uint32_t ul_source)
{
    p_adc->ADC_EOC_IER = ul_source;
}

/**
 * \brief Disable ADC EOC interrupts.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_source Interrupts to be disabled.
 */
void adc_disable_eoc_interrupt(Adc *p_adc, const uint32_t ul_source)
{
    p_adc->ADC_EOC_IDR = ul_source;
}

/**
 * \brief Get ADC EOC interrupt and overrun error status.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC EOC status structure.
 */
uint32_t adc_get_eoc_status(const Adc *p_adc)
{
    return p_adc->ADC_EOC_ISR;
}

/**
 * \brief Read ADC EOC interrupt mask.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return The interrupt mask value.
 */
uint32_t adc_get_eoc_interrupt_mask(const Adc *p_adc)
{
    return p_adc->ADC_EOC_IMR;
}


#ifndef ADC_WPMR_WPKEY_PASSWD
#define ADC_WPMR_WPKEY_PASSWD ADC_WPMR_WPKEY(0x414443u)
#endif
/**
 * \brief Enable or disable write protection of ADC registers.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param enable 1 to enable, 0 to disable.
 * \param int_enable 1 to enable, 0 to disable
 * \param control_enable 1 to enable, 0 to disable
 */
void adc_set_writeprotect(Adc *p_adc, const bool enable, const bool int_enable, const bool control_enable)
{
    uint32_t ul_reg;

    ul_reg = ADC_WPMR_WPKEY_PASSWD;

    if (enable) {
        ul_reg |= ADC_WPMR_WPEN;
    }

    if (int_enable) {
        ul_reg |= ADC_WPMR_WPITEN;
    }

    if (control_enable) {
        ul_reg |= ADC_WPMR_WPCREN;
    }

    p_adc->ADC_WPMR = ul_reg;
}

/**
 * \brief Indicate write protect status.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return 0 if no write protect violation occurred, or 16-bit write protect
 * violation source.
 */
uint32_t adc_get_writeprotect_status(const Adc *p_adc)
{
    uint32_t reg_value;

    reg_value = p_adc->ADC_WPSR;
    if (reg_value & ADC_WPSR_WPVS) {
        return (reg_value & ADC_WPSR_WPVSRC_Msk) >> ADC_WPSR_WPVSRC_Pos;
    } else {
        return 0;
    }
}

/**
 * \brief calcul_startup
 */
static uint32_t calcul_startup(const uint32_t ul_startup)
{
    uint32_t ul_startup_value = 0;

    if (ul_startup == 0)
        ul_startup_value = 0;
    else if (ul_startup == 1)
        ul_startup_value = 8;
    else if (ul_startup == 2)
        ul_startup_value = 16;
    else if (ul_startup == 3)
        ul_startup_value = 24;
    else if (ul_startup == 4)
        ul_startup_value = 64;
    else if (ul_startup == 5)
        ul_startup_value = 80;
    else if (ul_startup == 6)
        ul_startup_value = 96;
    else if (ul_startup == 7)
        ul_startup_value = 112;
    else if (ul_startup == 8)
        ul_startup_value = 512;
    else if (ul_startup == 9)
        ul_startup_value = 576;
    else if (ul_startup == 10)
        ul_startup_value = 640;
    else if (ul_startup == 11)
        ul_startup_value = 704;
    else if (ul_startup == 12)
        ul_startup_value = 768;
    else if (ul_startup == 13)
        ul_startup_value = 832;
    else if (ul_startup == 14)
        ul_startup_value = 896;
    else if (ul_startup == 15)
        ul_startup_value = 960;

    return ul_startup_value;
}

/**
 * \brief Check ADC configurations.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_mck Main clock of the device (in Hz).
 */
void adc_check(Adc *p_adc, const uint32_t ul_mck)
{
    uint32_t ul_adcfreq;
    uint32_t ul_prescal;
    uint32_t ul_startup;

    /* ADCClock = MCK / ( (PRESCAL+1) * 2 ) */
    ul_prescal = ((p_adc->ADC_MR & ADC_MR_PRESCAL_Msk) >>
            ADC_MR_PRESCAL_Pos);
    ul_adcfreq = ul_mck / ((ul_prescal + 1) * 2);
    printf("ADC clock frequency = %d Hz\r\n", (int)ul_adcfreq);

    if (ul_adcfreq < ADC_FREQ_MIN) {
        printf("adc frequency too low (out of specification: %d Hz)\r\n", (int)ADC_FREQ_MIN);
    }
    if (ul_adcfreq > ADC_FREQ_MAX) {
        printf("adc frequency too high (out of specification: %d Hz)\r\n", (int)ADC_FREQ_MAX);
    }

    ul_startup = ((p_adc->ADC_MR & ADC_MR_STARTUP_Msk) >> ADC_MR_STARTUP_Pos);
    if (!(p_adc->ADC_MR & ADC_MR_SLEEP_SLEEP)) {
        /* 40ms */
        if (ADC_STARTUP_NORM * ul_adcfreq / 1000000 > calcul_startup(ul_startup)) {
            printf("Startup time too small: %d, programmed: %d\r\n",
                    (int)(ADC_STARTUP_NORM * ul_adcfreq / 1000000),
                    (int)calcul_startup(ul_startup));
        }
    } else {
        if (!(p_adc->ADC_MR & ADC_MR_FWUP_ON)) {
            /* Sleep 40ms */
            if (ADC_STARTUP_NORM * ul_adcfreq / 1000000 > calcul_startup(ul_startup)) {
                printf("Startup time too small: %d, programmed: %d\r\n",
                    (int)(ADC_STARTUP_NORM * ul_adcfreq / 1000000),
                    (int)(calcul_startup(ul_startup)));
            }
        } else {
            if (p_adc->ADC_MR & ADC_MR_FWUP_ON) {
                /* Fast Wake Up Sleep Mode: 12ms */
                if (ADC_STARTUP_FAST * ul_adcfreq / 1000000 > calcul_startup(ul_startup)) {
                    printf("Startup time too small: %d, programmed: %d\r\n",
                        (int)(ADC_STARTUP_NORM * ul_adcfreq / 1000000),
                        (int)(calcul_startup(ul_startup)));
                }
            }
        }
    }
}

/**
 * \brief Get PDC registers base address.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC PDC register base address.
 */
Pdc *adc_get_pdc_base(const Adc *p_adc)
{
    UNUSED(p_adc);
    return PDC_ADC;
}

/**
 * \brief Set digital averaging trigger.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param multi The average requests several trigger events if true. The
 * average requests only one trigger event.
 */
void adc_set_averaging_trigger(Adc *p_adc, bool multi)
{
    if (multi) {
        p_adc->ADC_EMR &= ~ADC_EMR_ASTE;
    } else {
        p_adc->ADC_EMR |= ADC_EMR_ASTE;
    }
}

/**
 * \brief Set comparison filter.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param filter Number of consecutive compare events necessary to raise the
 * flag = filter + 1.
 */
void adc_set_comparison_filter(Adc *p_adc, uint8_t filter)
{
    p_adc->ADC_EMR &= ~ADC_EMR_CMPFILTER_Msk;
    p_adc->ADC_EMR |= ADC_EMR_CMPFILTER(filter);
}

/**
 * \brief Turn on temperature sensor.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_enable_ts(Adc *p_adc)
{
    p_adc->ADC_TEMPMR |= ADC_TEMPMR_TEMPON;
}

/**
 * \brief Turn off temperature sensor.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc_disable_ts(Adc *p_adc)
{
    p_adc->ADC_TEMPMR &= ~ADC_TEMPMR_TEMPON;
}

/**
 * \brief Configure temperature sensor comparison.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param mode  Temperature comparison mode.
 * \param low_threshold Temperature low threshold.
 * \param high_threshold Temperature high threshold.
 */
void adc_configure_ts_comparison(Adc *p_adc, enum adc_temp_cmp_mode mode,
        uint16_t low_threshold, uint16_t high_threshold)
{
    uint32_t tmp = p_adc->ADC_TEMPMR;
    tmp &= ~ADC_TEMPMR_TEMPCMPMOD_Msk;
    tmp |= mode;

    p_adc->ADC_TEMPCWR = ADC_TEMPCWR_TLOWTHRES(low_threshold) |
            ADC_TEMPCWR_THIGHTHRES(high_threshold);
    p_adc->ADC_TEMPMR = tmp;
}

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
