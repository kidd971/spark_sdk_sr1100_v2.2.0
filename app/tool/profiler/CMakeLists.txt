add_library(profiler_backend "")
target_include_directories(profiler_backend PUBLIC facade)

add_subdirectory(config)

# DUT executable
add_executable(profiler_dut.elf "")
target_sources(profiler_dut.elf PRIVATE profiler_dut.c)
target_link_libraries(profiler_dut.elf
    PRIVATE
        profiler_backend
        swc
)
add_custom_command(TARGET profiler_dut.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary profiler_dut.elf profiler_dut.bin
)

if (NOT MULTI_TRANSCEIVER STREQUAL "DUAL_TRANSCEIVER")
    # Helper executable
    add_executable(profiler_helper.elf "")
    target_sources(profiler_helper.elf PRIVATE profiler_helper.c)

    target_link_libraries(profiler_helper.elf
        PRIVATE
            profiler_backend
            swc
    )
    add_custom_command(TARGET profiler_helper.elf
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O binary profiler_helper.elf profiler_helper.bin
    )
endif()

# UART RX test -- standalone echo test, no SWC/radio needed
add_executable(uart_rx_test.elf "")
target_sources(uart_rx_test.elf PRIVATE uart_rx_test.c)
target_link_libraries(uart_rx_test.elf
    PRIVATE
        profiler_backend
        swc
)
add_custom_command(TARGET uart_rx_test.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary uart_rx_test.elf uart_rx_test.bin
)
