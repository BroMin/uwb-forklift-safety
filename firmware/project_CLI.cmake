# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2

set(MY_BOARD DWM3001CDK)
set(MY_OS FreeRTOS)
set(MY_SAMPLE "CLI")
set(MY_HAL nrfx)
set(MY_BSP Nordic)
set(MY_CPU NRF52833_XXAA)
set(MY_LD_FILE nRF52833.ld)

set(CMAKE_CUSTOM_C_FLAGS
    "-Werror \
    -DBOARD_CUSTOM \
    -DCONFIG_GPIO_AS_PINRESET"
)

set(MY_TARGET ${MY_BOARD}-${MY_SAMPLE}-${MY_OS})

add_definitions(-Os)
add_definitions(-DUSE_USB_ENUM_WORKAROUND)

set(PROJECT_ARCH "m4")
set(PROJECT_FP "hard")
set(PROJECT_FPU "fpv4-sp-d16")

set(USE_DRV_DW3000 1)
set(USE_DRV_DW3720 0)

set(USE_CRYPTO_BACKEND_MBEDTLS 1)
set(USE_CRYPTO_SPEED_OPTIMIZATION 0)