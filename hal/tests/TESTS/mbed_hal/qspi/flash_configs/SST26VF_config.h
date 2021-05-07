/* mbed Microcontroller Library
 * Copyright (c) 2018-2021 ARM Limited
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
#ifndef MBED_QSPI_FLASH_SST26VF_H
#define MBED_QSPI_FLASH_SST26VF_H

#define QSPI_FLASH_CHIP_STRING "Microchip SST26VF"
#define QSPI_FLASH_MICROCHIP_SST26VF

// Command for reading configuration register
#define QSPI_CMD_RDCR0                         0x35  // To read Quad (QE) enable bit
// Command for writing status/configuration register
#define QSPI_CMD_WRSR                          0x01 // To write Qual (QE) enable bit
// Command for reading status register
#define QSPI_CMD_RDSR                          0x05 // To read WIP bit of status register 1
// Command for Global Block Protection Unlock
#define QSPI_CMD_GLOBAL_BP_UNLOCK              0x98



// Command for reading security register
#define QSPI_CMD_RDSCUR                         0x72

// Command for setting Reset Enable
#define QSPI_CMD_RSTEN                          0x66
// Command for setting Reset
#define QSPI_CMD_RST                            0x99

// Command for setting write enable
#define QSPI_CMD_WREN                           0x06
// Command for setting write disable
#define QSPI_CMD_WRDI                           0x04
// Command for enabling Quad IO
#define QSPI_CMD_EQIO                           0x38
// Command for reseting Quad IO
#define QSPI_CMD_RSTQIO                         0xFF

// WRSR operations max time [us] (datasheet max time + 33%)
#define QSPI_WRSR_MAX_TIME                      2000   // 2 ms
// general wait max time [us]
#define QSPI_WAIT_MAX_TIME                      50000  // 50 ms


// Commands for writing (page programming)
#define QSPI_CMD_WRITE_1IO                      0x02    // 1-1-1 mode
#define QSPI_CMD_WRITE_4IO                      0x32    // 1-4-4 mode

// write operations max time [us] (datasheet max time + 15%)
#define QSPI_PAGE_PROG_MAX_TIME                 1725   // 1725 us

#define QSPI_PAGE_SIZE                          256     // 256B
#define QSPI_SECTOR_SIZE                        4096    // 4kB
#define QSPI_SECTOR_COUNT                       2048    // 8MB (SST26VF064)

// Commands for reading
#define QSPI_CMD_READ_1IO_FAST                  0x0B   // 1-1-1 mode
#define QSPI_CMD_READ_1IO                       0x03   // 1-1-1 mode
#define QSPI_CMD_READ_1I2O                      0x3B   // 1-1-2 mode    - dual output
#define QSPI_CMD_READ_2IO                       0xBB   // 1-2-2 mode    - dual I/O
#define QSPI_CMD_READ_1I4O                      0x6B   // 1-1-4 mode    - quad output
#define QSPI_CMD_READ_4IO                       0xEB   // 1-4-4 mode    - quad I/O

// Alt (mode) value for quad I/O read
#define QSPI_ALT_READ_4IO                       0x01 // 1-4-4 mode only

#define QSPI_READ_1IO_DUMMY_CYCLE               0
#define QSPI_READ_FAST_DUMMY_CYCLE              8
#define QSPI_READ_2IO_DUMMY_CYCLE               4 // dual I/O
#define QSPI_READ_1I2O_DUMMY_CYCLE              8 // dual output
#define QSPI_READ_4IO_DUMMY_CYCLE               6 // quad I/O - 2 cycles for Mode or Alt (4 bits per cycle x 2 cycles = 1 byte) + 4 dummy cycles
#define QSPI_READ_1I4O_DUMMY_CYCLE              8 // quad output

// Commands for erasing
#define QSPI_CMD_ERASE_SECTOR                   0x20    // Erase 4 KBytes of Memory Array
#define QSPI_CMD_ERASE_CHIP                     0xC7

// erase operations max time [us] (datasheet max time + 15%)
#define QSPI_ERASE_SECTOR_MAX_TIME              57500      // 1.15*50 ~ 57.5 ms

// max frequency for basic rw operation (for fast mode)
#define QSPI_COMMON_MAX_FREQUENCY               40000000

#define QSPI_STATUS_REG_SIZE                    1
#define QSPI_CONFIG_REG_0_SIZE                  1
#define QSPI_SECURITY_REG_SIZE                  18
#define QSPI_MAX_REG_SIZE                       18

// status register
#define STATUS_BIT_WIP   (1 << 0)   // write in progress bit
#define STATUS_BIT_WEL   (1 << 1)   // write enable latch

// configuration register
#define CONFIG_BIT_IOC   (1 << 1)   // I/O Configuration bit for SPI Mode

#define EXTENDED_SPI_ENABLE()                                                          \
	uint8_t reg_data[2] = { 0 };                                                       \
																					   \
    if (read_register(CONFIG_REG0, &reg_data[1], 1, qspi) != QSPI_STATUS_OK) {         \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    if (write_enable(qspi) != QSPI_STATUS_OK) {                                        \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    reg_data[1] |= CONFIG_BIT_IOC;                                                     \
    if (write_register(QSPI_CMD_WRSR, reg_data, 2, qspi) != QSPI_STATUS_OK) {          \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    reg_data[1] = 0;                                                                   \
    if (read_register(CONFIG_REG0, &reg_data[1], 1, qspi) != QSPI_STATUS_OK) {         \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    return ((reg_data[1] & CONFIG_BIT_IOC) != 0 ? QSPI_STATUS_OK : QSPI_STATUS_ERROR)

#define EXTENDED_SPI_DISABLE()                                                         \
	uint8_t reg_data[2] = { 0 };                                                       \
																					   \
    if (read_register(CONFIG_REG0, &reg_data[1], 1, qspi) != QSPI_STATUS_OK) {         \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    if (write_enable(qspi) != QSPI_STATUS_OK) {                                        \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    reg_data[1] &= ~CONFIG_BIT_IOC;                                                    \
    if (write_register(QSPI_CMD_WRSR, reg_data, 2, qspi) != QSPI_STATUS_OK) {          \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    reg_data[1] = 0;                                                                   \
    if (read_register(CONFIG_REG0, &reg_data[1], 1, qspi) != QSPI_STATUS_OK) {         \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    return ((reg_data[1] & CONFIG_BIT_IOC) == 0 ? QSPI_STATUS_OK : QSPI_STATUS_ERROR)

#define QUAD_ENABLE()                                                                  \
	uint8_t reg_data[2] = { 0 };                                                       \
																					   \
    if (read_register(CONFIG_REG0, &reg_data[1], 1, qspi) != QSPI_STATUS_OK) {         \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    if (write_enable(qspi) != QSPI_STATUS_OK) {                                        \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    reg_data[1] |= CONFIG_BIT_IOC;                                                     \
    if (write_register(QSPI_CMD_WRSR, reg_data, 2, qspi) != QSPI_STATUS_OK) {          \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    if (write_register(QSPI_CMD_EQIO, 0, 0, qspi) != QSPI_STATUS_OK) {                 \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
    return QSPI_STATUS_OK

#define QUAD_DISABLE()                                                                 \
	uint8_t reg_data[2] = { 0 };                                                       \
                                                                                       \
    if (write_register(QSPI_CMD_RSTQIO, 0, 0, qspi) != QSPI_STATUS_OK) {               \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    if (read_register(CONFIG_REG0, &reg_data[1], 1, qspi) != QSPI_STATUS_OK) {         \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    if (write_enable(qspi) != QSPI_STATUS_OK) {                                        \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
																					   \
    reg_data[1] &= ~CONFIG_BIT_IOC;                                                    \
    if (write_register(QSPI_CMD_WRSR, reg_data, 2, qspi) != QSPI_STATUS_OK) {          \
        return QSPI_STATUS_ERROR;                                                      \
    }                                                                                  \
    return QSPI_STATUS_OK

#endif // MBED_QSPI_FLASH_SST26VF_H
