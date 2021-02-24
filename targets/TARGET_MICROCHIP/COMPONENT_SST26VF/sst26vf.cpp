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


#include "mbed.h"
#include "sst26vf.h"

/** Device ready/busy status bit. */
#define SST26VF_STATUS_BUSY                      (1 << 0)
/** Device is ready. */
#define SST26VF_STATUS_BUSY_READY                (0 << 0)
/** Device is busy with internal operations. */
#define SST26VF_STATUS_BUSY_BUSY                 (1 << 0)
/** Write enable latch status bit. */
#define SST26VF_STATUS_WEL                       (1 << 1)
/** Device is not write enabled. */
#define SST26VF_STATUS_WEL_DISABLED              (0 << 1)
/** Device is write enabled. */
#define SST26VF_STATUS_WEL_ENABLED               (1 << 1)
/** Write Suspend-Erase status bit. */
#define SST26VF_STATUS_WSE                       (1 << 2)
/** Device erase is not suspended. */
#define SST26VF_STATUS_WSE_DISABLED              (0 << 2)
/** Device erase is suspended. */
#define SST26VF_STATUS_WSE_ENABLED               (1 << 2)
/** Write protect pin status bit. */
#define SST26VF_STATUS_WSP                       (1 << 3)
/** Device program is not suspended. */
#define SST26VF_STATUS_WSP_DISABLED              (0 << 3)
/** Device program is suspended. */
#define SST26VF_STATUS_WSP_ENABLED               (1 << 3)
/** Write Protection Lock-Down status bit. */
#define SST26VF_STATUS_WPLD                      (1 << 4)
/** Write Protection Lock-Down is disabled. */
#define SST26VF_STATUS_WPLD_DISABLED             (0 << 4)
/** Write Protection Lock-Down is enabled. */
#define SST26VF_STATUS_WPLD_ENABLED              (1 << 4)
/** Security ID status bit. */
#define SST26VF_STATUS_SEC                       (1 << 5)
/** Security ID space is not locked. */
#define SST26VF_STATUS_SEC_UNLOCKED              (0 << 5)
/** Security ID space is locked. */
#define SST26VF_STATUS_SEC_LOCKED                (1 << 5)

/** I/O Configuration bit for SPI Mode. */
#define SST26VF_CONFIG_IOC                       (1 << 1)
/** WP# and HOLD# pins are enabled. */
#define SST26VF_STATUS_IOC_ENABLED               (0 << 1)
/** WP# and HOLD# pins are disabled. */
#define SST26VF_STATUS_IOC_DISABLED              (1 << 1)

/** Block-Protection Volatility State bit. */
#define SST26VF_CONFIG_BPNV                      (1 << 3)
/** Any block has been permanently locked. */
#define SST26VF_STATUS_BPNV_PERM_LOCKED          (0 << 3)
/** No memory block has been permanently locked. */
#define SST26VF_STATUS_BPNV_NO_PERM_LOCKED       (1 << 3)

/** Write-Protection Pin (WP#) Enable bit. */
#define SST26VF_CONFIG_WPEN                      (1 << 7)
/** WP# is disabled. */
#define SST26VF_STATUS_WPEN_DISABLED             (0 << 7)
/** WP# is enabled. */
#define SST26VF_STATUS_WPEN_ENABLED              (1 << 7)

/** SST26VF Flash Manufacturer JEDEC ID */
#define SST26VF_MUNFACTURER                      0xBF
#define SST26VF_MEM_TYPE                         0x26
#define SST26VF016B_DEVICE_ID                    0x41
#define SST26VF032B_DEVICE_ID                    0x42
#define SST26VF064B_DEVICE_ID                    0x43

/** SST26VF Block Protection Register Size */
#define SST26VF016B_BPR_SIZE                     6
#define SST26VF032B_BPR_SIZE                     10
#define SST26VF064B_BPR_SIZE                     18

#define SST26VF_PAGE_SIZE                        256

SST26VF::SST26VF(PinName io0, PinName io1, PinName io2, PinName io3, PinName sclk, PinName ssel, int hz) : _qspi(io0, io1, io2, io3, sclk, ssel), _hz(hz)
{
    int status;

    _initialized = false;
    _bpr_size = 0;

    status = _qspi.set_frequency(_hz);
    MBED_ASSERT(status == QSPI_STATUS_OK);

    status = _qspi.configure_format(QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE,
                                       QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                                       QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 0);

    // Reset Sequence
    status =  _qspi.command_transfer(SST26VF_RESET_ENABLE, -1, NULL, 0, NULL, 0);
    MBED_ASSERT(status == QSPI_STATUS_OK);
    status =  _qspi.command_transfer(SST26VF_RESET_MEMORY, -1, NULL, 0, NULL, 0);
    MBED_ASSERT(status == QSPI_STATUS_OK);

    // Enable Quad IO mode
    status =  _qspi.command_transfer(SST26VF_ENABLE_QUAD_IO, -1, NULL, 0, NULL, 0);
    MBED_ASSERT(status == QSPI_STATUS_OK);

    status = _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                       QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                       QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 2);

    // Read Jedec-ID
    status =  _qspi.command_transfer(SST26VF_READ_QUAD_JDEC_ID, -1, NULL, 0, _jedec_id, 3);
    MBED_ASSERT(status == QSPI_STATUS_OK);

    if ((_jedec_id[0] == SST26VF_MUNFACTURER) && (_jedec_id[1] == SST26VF_MEM_TYPE)) {
        _initialized = true;

        if (_jedec_id[2] == SST26VF016B_DEVICE_ID) {
            _bpr_size = SST26VF016B_BPR_SIZE;
        } else if (_jedec_id[2] == SST26VF032B_DEVICE_ID) {
            _bpr_size = SST26VF032B_BPR_SIZE;
        } else if (_jedec_id[2] == SST26VF064B_DEVICE_ID) {
            _bpr_size = SST26VF064B_BPR_SIZE;
        } else {
            _initialized = false;
        }
    }

    // Read Block Protection Register
    _update_bpr();
}

SST26VF::~SST26VF()
{

}

sst26vf_status_t SST26VF::read(int address, char *rx_buffer, size_t *rx_length)
{
    sst26vf_status_t ret_status = SST26VF_STATUS_ERROR;

    if (_initialized) {
        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 4);

        // Read memory
        _qspi.read(SST26VF_READ_MEMORY_HIGH_SPEED, 0, address, rx_buffer, rx_length);

       ret_status = SST26VF_STATUS_OK;

    }

    return ret_status;
}

sst26vf_status_t SST26VF::write(int address, const char *tx_buffer, size_t *tx_length)
{
    sst26vf_status_t ret_status = SST26VF_STATUS_ERROR;
    size_t size_frag, size;
    char *data;

    if (_initialized) {
        /* Manage Page Boundary (256 bytes) */
        data = (char *)tx_buffer;
        size = *tx_length;
        size_frag = _get_size_first_fragment(address, size);

        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        while (size) {
            // Write Enable
            _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

            // Page Program
            _qspi.command_transfer(SST26VF_PAGE_PROGRAM, address, data, size_frag, NULL, 0);

            // Update counters
            size -= size_frag;
            address += size_frag;
            data += size_frag;

            if (size < SST26VF_PAGE_SIZE) {
                size_frag = size;
            } else {
                size_frag = SST26VF_PAGE_SIZE;
            }

            // Wait for transfer to finish
            _wait_to_finish(1);
        }

        ret_status = SST26VF_STATUS_OK;
    }

    return ret_status;
}

sst26vf_status_t SST26VF::erase_chip()
{
    sst26vf_status_t ret_status = SST26VF_STATUS_ERROR;

    if (_initialized) {
        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        // Check that the flash is ready
        _qspi.command_transfer(SST26VF_READ_STATUS, -1, NULL, 0, &_status, 1);
        if (_status & SST26VF_STATUS_BUSY) {
            return SST26VF_STATUS_IS_BUSY;
        }

        // Write Enable
        _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

        // Chip Erase
        _qspi.command_transfer(SST26VF_ERASE_FULL, -1, NULL, 0, NULL, 0);

        /* Wait for transfer to finish */
        _wait_to_finish(20);

        ret_status = SST26VF_STATUS_OK;
    }

    return ret_status;
}

sst26vf_status_t SST26VF::erase_block(int address)
{
    sst26vf_status_t ret_status = SST26VF_STATUS_ERROR;

    if (_initialized) {
        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        // Write Enable
        _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

        // Chip Erase
        _qspi.command_transfer(SST26VF_ERASE_BLOCK_MEMORY, address, NULL, 0, NULL, 0);

        /* Wait for transfer to finish */
        _wait_to_finish(10);

        ret_status = SST26VF_STATUS_OK;
    }

    return ret_status;
}

sst26vf_status_t SST26VF::erase_sector(int address)
{
    sst26vf_status_t ret_status = SST26VF_STATUS_ERROR;

    if (_initialized) {
        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        // Write Enable
        _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

        // Chip Erase
        _qspi.command_transfer(SST26VF_ERASE_SECTOR_4KB, address, NULL, 0, NULL, 0);

        /* Wait for transfer to finish */
        _wait_to_finish(10);

        ret_status = SST26VF_STATUS_OK;
    }

    return ret_status;
}

sst26vf_status_t SST26VF::global_unlock()
{
    sst26vf_status_t ret_status = SST26VF_STATUS_ERROR;

    if (_initialized) {
        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        // Write Enable
        _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

        // Global Block Protection Unlock
        _qspi.command_transfer(SST26VF_GLOBAL_BP_UNLOCK, -1, NULL, 0, NULL, 0);

        // Update Block-Protection map
        _update_bpr();

        ret_status = SST26VF_STATUS_OK;
    }

    return ret_status;
}

sst26vf_status_t SST26VF::lock(int address)
{
    sst26vf_status_t ret_status = SST26VF_STATUS_OK;
    uint8_t bpr_index, bpr_bit_offset;

    if (_initialized) {
        // Check if block is already locked
        if (_block_is_protected(address, &bpr_index, &bpr_bit_offset)) {
            return SST26VF_STATUS_OK;
        }

        if (bpr_index == 0xFF) {
            return SST26VF_STATUS_INVALID_PARAMETER;
        }

        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        // Write Enable
        _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

        // Set the bit in Block Protection Register
        _bpr[bpr_index] |= (1 << bpr_bit_offset);
        _qspi.command_transfer(SST26VF_WRITE_BPR, -1, _bpr, _bpr_size, NULL, 1);

        // Update Block-Protection map
        _update_bpr();
    }

    return ret_status;
}

sst26vf_status_t SST26VF::unlock(int address)
{
    sst26vf_status_t ret_status = SST26VF_STATUS_OK;
    uint8_t bpr_index, bpr_bit_offset;

    if (_initialized) {
        // Check if block is already locked
        if (!_block_is_protected(address, &bpr_index, &bpr_bit_offset)) {
            if (bpr_index == 0xFF) {
                return SST26VF_STATUS_INVALID_PARAMETER;
            }

            return SST26VF_STATUS_OK;
        }
        _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                                  QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 0);

        // Write Enable
        _qspi.command_transfer(SST26VF_WRITE_ENABLE, -1, NULL, 0, NULL, 0);

        // Clear the bit in Block Protection Register
        _bpr[bpr_index] &= ~(1 << bpr_bit_offset);
        _qspi.command_transfer(SST26VF_WRITE_BPR, -1, _bpr, _bpr_size, NULL, 1);

        // Update Block-Protection map
        _update_bpr();
    }

    return ret_status;
}

void SST26VF::_wait_to_finish(int delayms)
{
    _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                              QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                              QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 2);

    _qspi.command_transfer(SST26VF_READ_STATUS, -1, NULL, 0, &_status, 1);
    while (_status & SST26VF_STATUS_BUSY) {
         ThisThread::sleep_for(delayms);
        _qspi.command_transfer(SST26VF_READ_STATUS, -1, NULL, 0, &_status, 1);
    }
}

void SST26VF::_update_bpr(void)
{
    _qspi.configure_format(QSPI_CFG_BUS_QUAD, QSPI_CFG_BUS_QUAD,
                              QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_QUAD,
                              QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_QUAD, 2);

    _qspi.command_transfer(SST26VF_READ_BPR, -1, NULL, 0, _bpr, _bpr_size);
}

bool SST26VF::_block_is_protected(int address, uint8_t *bpr_idx, uint8_t *bit_offset)
{
    int addr_8kb_up, addr_32kb_up;
    uint8_t bpr_bit_idx, bpr_index, bpr_bit_offset;
    uint8_t total_bits;

    if (_jedec_id[2] == SST26VF016B_DEVICE_ID) {
        addr_8kb_up = 0x1F8000;
        addr_32kb_up = 0x1F0000;
        /* Check maximum size */
        if (address > 0x1FFFFF) {
            *bpr_idx = 0xFF;
            return false;
        }
    } else if (_jedec_id[2] == SST26VF032B_DEVICE_ID) {
        addr_8kb_up = 0x3F8000;
        addr_32kb_up = 0x3F0000;
        /* Check maximum size */
        if (address > 0x3FFFFF) {
            *bpr_idx = 0xFF;
            return false;
        }
    } else {
        /* SST26VF064B_DEVICE_ID */
        addr_8kb_up = 0x7F8000;
        addr_32kb_up = 0x7F0000;
        /* Check maximum size */
        if (address > 0x7FFFFF) {
            *bpr_idx = 0xFF;
            return false;
        }
    }

    total_bits = _bpr_size << 3;

    if (address < 0x7FFF) {
        /* 8KByte block */
        bpr_bit_idx = 15 - (((uint8_t)(address >> 13) & 0x03) << 1);
    } else if (address < 0xFFFF) {
        /* 32KByte block */
        bpr_bit_idx = 18;
    } else if (address > addr_8kb_up) {
        /* 8KByte block */
        bpr_bit_idx = 7 - (((uint8_t)(address >> 13) & 0x03) << 1);
    } else if (address > addr_32kb_up) {
        /* 32KByte block */
        bpr_bit_idx = 17;
    } else {
        /* 64KByte block */
        bpr_bit_idx = total_bits - (uint8_t)(address >> 16);
    }

    bpr_index = bpr_bit_idx >> 3;
    bpr_bit_offset = 7 - (bpr_bit_idx & 0x7);

    if (bpr_idx != NULL) {
        *bpr_idx = bpr_index;
    }

    if (bit_offset != NULL) {
        *bit_offset = bpr_bit_offset;
    }

    if (_bpr[bpr_index] & (1 << bpr_bit_offset)) {
        return true;
    } else {
        return false;
    }

}

size_t SST26VF::_get_size_first_fragment(int address, size_t size)
{
	size_t size_frag;

	if (address & 0x000000FF) {
		if (size > SST26VF_PAGE_SIZE) {
			size_frag = SST26VF_PAGE_SIZE - (address & 0xFF);
		} else {
			size_frag = size - (address & 0xFF);
		}
	} else {
		if (size > SST26VF_PAGE_SIZE) {
			size_frag = SST26VF_PAGE_SIZE;
		} else {
			size_frag = size;
		}
	}

	return size_frag;
}
