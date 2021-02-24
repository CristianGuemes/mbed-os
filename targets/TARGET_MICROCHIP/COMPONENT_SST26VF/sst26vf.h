/* Microchip SST26VF Component
 * Copyright (c) 2021 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MBED_SST26VF_H
#define MBED_SST26VF_H

#include "mbed.h"

/** SST26VF return status
 */
typedef enum sst26vf_status {
    SST26VF_STATUS_ERROR = -1,             /**< Generic error >*/
    SST26VF_STATUS_INVALID_PARAMETER = -2, /**< The parameter is invalid >*/
    SST26VF_STATUS_IS_BUSY = -3,           /**< Device is busy >*/
    SST26VF_STATUS_IS_PROTECTED = -4,      /**< Device is protected >*/
    SST26VF_STATUS_OK    =  0,             /**< Function executed sucessfully  >*/
} sst26vf_status_t;

class SST26VF {

public:

    /** Initialize the SST26VF memory and set the qspi bus clock frequency
     *
     *  @param qspi QSPI instance
     *  @param hz SCLK frequency in hz (default = 1MHz)
     *
     */
    SST26VF(PinName io0, PinName io1, PinName io2, PinName io3, PinName sclk, PinName ssel, int hz = 1000000);

    virtual ~SST26VF();

    /** Read from SST26VF memory
     *
     *  @param address Address to be accessed in SST26VF memory
     *  @param rx_buffer Buffer for data to be read from the peripheral
     *  @param rx_length Pointer to a variable containing the length of rx_buffer, and on return this variable will be updated with the actual number of bytes read
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful reads and SST26VF_STATUS_ERROR on failed reads.
     */
    sst26vf_status_t read(int address, char *rx_buffer, size_t *rx_length);

    /** Write to SST26VF memory
     *
     *  @param address Address to be accessed in SST26VF memory
     *  @param tx_buffer Buffer containing data to be sent to peripheral
     *  @param tx_length Pointer to a variable containing the length of data to be transmitted, and on return this variable will be updated with the actual number of bytes written
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful writes and SST26VF_STATUS_ERROR on failed writes.
     */
    sst26vf_status_t write(int address, const char *tx_buffer, size_t *tx_length);

    /** Erases all the content of the memory chip
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful erase and SST26VF_STATUS_ERROR on failed erase.
     */
    sst26vf_status_t erase_chip();

    /** Erases the specified block
     *
     *  @param address Address of the block to erase
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful erase and SST26VF_STATUS_ERROR on failed erase.
     *  @note Block sizes can be 8 KByte, 32 KByte or 64 KByte depending on address.
     */
    sst26vf_status_t erase_block(int address);

    /** Erases the specified 4KB sector of the serial firmware dataflash
     *
     *  @param address Address of the 4KB sector to erase
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful erase and SST26VF_STATUS_ERROR on failed erase.
     */
    sst26vf_status_t erase_sector(int address);

    /** The Global Unlock clears all write-protection bits in the Block-Protection register
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful unlock and SST26VF_STATUS_ERROR on failed unlock.
     */
    sst26vf_status_t global_unlock();

    /** Protect block of the SST26VF memory against write operations
     *
     *  @param address Address of the block to protect
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful lock and SST26VF_STATUS_ERROR on failed lock.
     */
    sst26vf_status_t lock(int address);

    /** Unprotect block of the SST26VF memory for write operations
     *
     *  @param address Address of the block to unprotect
     *
     *  @returns
     *    Returns SST26VF_STATUS_OK on successful unlock and SST26VF_STATUS_ERROR on failed unlock.
     */
    sst26vf_status_t unlock(int address);

#if !defined(DOXYGEN_ONLY)
private:
    /* Commands : Listed below are commands supported */
    enum cmdSupported {
        SST26VF_RESET_ENABLE             = 0x66, /**< Configuration Inst: Reset Enable */
        SST26VF_RESET_MEMORY             = 0x99, /**< Configuration Inst: Reset Memory */
        SST26VF_ENABLE_QUAD_IO           = 0x38, /**< Configuration Inst: Enable QUAD I/O */
        SST26VF_RESET_EQUAD_IO           = 0xFF, /**< Configuration Inst: Reset QUAD I/O */
        SST26VF_READ_STATUS              = 0x05, /**< Configuration Inst: Read Status Register */
        SST26VF_WRITE_CONFIG             = 0x01, /**< Configuration Inst: Write Configuration Register */
        SST26VF_READ_CONFIG              = 0x35, /**< Configuration Inst: Read Configuration Reg */
        SST26VF_READ_MEMORY              = 0x03, /**< Read Inst: Read Memory */
        SST26VF_READ_MEMORY_HIGH_SPEED   = 0x0B, /**< Read Inst: Read memory at High Speed */
        SST26VF_SPI_QUAD_OUTPUT_READ     = 0x6B, /**< Read Inst: SPI QUAD Output Read */
        SST26VF_SPI_QUAD_IO_READ         = 0xEB, /**< Read Inst: SPI QUAD I/O Read */
        SST26VF_SPI_DUAL_OUTPUT_READ     = 0x3B, /**< Read Inst: SPI DUAL Output Read */
        SST26VF_SPI_DUAL_IO_READ         = 0xBB, /**< Read Inst: SPI DUAL I/O Read */
        SST26VF_SET_BURST_LEN            = 0xC0, /**< Read Inst: Set Burst Length */
        SST26VF_SQI_READ_BURST           = 0x0C, /**< Read Inst: SQI Read Burst with Wrap */
        SST26VF_SPI_READ_BURST           = 0xEC, /**< Read Inst: SPI Read Burst with Wrap */
        SST26VF_READ_JDEC_ID             = 0x9F, /**< Identification Inst: JEDEC-ID Read */
        SST26VF_READ_QUAD_JDEC_ID        = 0xAF, /**< Identification Inst: Quad I/O J-ID Read */
        SST26VF_READ_SERIAL_FLASH_PARAMS = 0x5A, /**< Identification Inst: Serial Flash Discoverable Parameters */
        SST26VF_WRITE_ENABLE             = 0x06, /**< Write Inst: Write Enable */
        SST26VF_WRITE_DISABLE            = 0x04, /**< Write Inst: Write Disable */
        SST26VF_ERASE_SECTOR_4KB         = 0x20, /**< Write Inst: Erase 4 KBytes of Memory Array */
        SST26VF_ERASE_BLOCK_MEMORY       = 0xD8, /**< Write Inst: Erase 64, 32 or 8 KBytes of Memory Array */
        SST26VF_ERASE_FULL               = 0xC7, /**< Write Inst: Erase Full Array */
        SST26VF_PAGE_PROGRAM             = 0x02, /**< Write Inst: Page Program */
        SST26VF_SQO_QUAD_PAGE_PROGRAM    = 0x32, /**< Write Inst: SQI Quad Page Program */
        SST26VF_SUSPEND_PE               = 0xB0, /**< Write Inst: Suspends Program/Erase */
        SST26VF_RESUME_PE                = 0x30, /**< Write Inst: Resumes Program/Erase */
        SST26VF_READ_BPR                 = 0x72, /**< Protection Inst: Read Block-Protection Register */
        SST26VF_WRITE_BPR                = 0x42, /**< Protection Inst: Write Block-Protection Register */
        SST26VF_LOCK_DOWN_BPR            = 0x8D, /**< Protection Inst: Lock Down Block-Protection Register */
        SST26VF_NV_WRITE_LOCK_LDR        = 0xE8, /**< Protection Inst: non-Volatile Write Lock-Down Register */
        SST26VF_GLOBAL_BP_UNLOCK         = 0x98, /**< Protection Inst: Global Block Protection Unlock */
        SST26VF_READ_SECURITY_ID         = 0x88, /**< Protection Inst: Read Security ID */
        SST26VF_PROGRAM_USER_SEC_ID_AREA = 0xA5, /**< Protection Inst: Program User Security ID area */
        SST26VF_LOCKOUT_SEC_ID_PROG      = 0x85, /**< Protection Inst: Lockout Security ID Programming */
        SST26VF_DEEP_POWERDOWN           = 0xB9, /**< Power Saving Inst: Deep Power-down mode */
        SST26VF_RELEASE_DEEP_POWERDOWN   = 0xAB  /**< Power Saving Inst: Release from Deep Power-down mode */
    };

    mbed::QSPI _qspi;
    bool _initialized;
    int _hz; // Bus Frequency
    int _mode; // QSPI mode
    char _status; // Status register
    char _jedec_id[3]; // Jedec-ID
    int _bpr_size;
    char _bpr[18]; // Block Protection Register

    /*
     * This function waits for transfer to finish
     */
    inline void _wait_to_finish(int delayms);

    /*
     * This function reads the Block Protection Register and updates _bpr
     */
    inline void _update_bpr(void);

    /*
     * Check if memory address is protected
     */
    inline bool _block_is_protected(int address, uint8_t *bpr_idx, uint8_t *bit_offset);

    /*
     * Check if memory address is protected
     */
    inline size_t _get_size_first_fragment(int address, size_t size);

#endif
};

#endif //MBED_SST26VF_H
