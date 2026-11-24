cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/../cmake/arm-none-eabi.cmake)

project(f429 C ASM)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(LIB ${CMAKE_CURRENT_LIST_DIR}/../lib)
set(HAL ${LIB}/stm32f4xx-hal-driver)
set(DEV ${LIB}/cmsis-device-f4)

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

add_executable(f429.elf
    Src/main.c
    Src/stm32f4xx_it.c
    ${DEV}/Source/Templates/system_stm32f4xx.c
    ${DEV}/Source/Templates/gcc/startup_stm32f429xx.s
    ${HAL}/Src/stm32f4xx_hal.c
    ${HAL}/Src/stm32f4xx_hal_can.c
    ${HAL}/Src/stm32f4xx_hal_cortex.c
    ${HAL}/Src/stm32f4xx_hal_dma.c
    ${HAL}/Src/stm32f4xx_hal_dma_ex.c
    ${HAL}/Src/stm32f4xx_hal_exti.c
    ${HAL}/Src/stm32f4xx_hal_flash.c
    ${HAL}/Src/stm32f4xx_hal_flash_ex.c
    ${HAL}/Src/stm32f4xx_hal_gpio.c
    ${HAL}/Src/stm32f4xx_hal_pwr.c
    ${HAL}/Src/stm32f4xx_hal_pwr_ex.c
    ${HAL}/Src/stm32f4xx_hal_rcc.c
    ${HAL}/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL}/Src/stm32f4xx_hal_uart.c
)

target_include_directories(f429.elf PRIVATE
    Inc
    ${HAL}/Inc
    ${DEV}/Include
    ${LIB}/cmsis_core/Include
)

target_compile_definitions(f429.elf PRIVATE STM32F429xx USE_HAL_DRIVER)

target_compile_options(f429.elf PRIVATE
    ${MCU_FLAGS}
    -Wall -Wextra
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-O2>
)

target_link_options(f429.elf PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_CURRENT_LIST_DIR}/STM32F429ZITx_FLASH.ld
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/f429.map
    -Wl,--print-memory-usage
    -Wl,--no-warn-rwx-segments
)

add_custom_command(TARGET f429.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary f429.elf f429.bin
    COMMAND ${CMAKE_SIZE} f429.elf
)
