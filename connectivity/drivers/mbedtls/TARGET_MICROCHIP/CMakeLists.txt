# Copyright (c) 2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_include_directories(mbed-mbedtls
    INTERFACE
        .
)

target_sources(mbed-mbedtls
    INTERFACE
        aes_alt.c
		aria_alt.c
		sha1_alt.c
		sha256_alt.c
		sha512_alt.c
)
