# at_command: portable AT module library (no BSP dependency)
add_library(at_command STATIC at_module.c)
target_include_directories(at_command PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# at_command_test: integration test using profiler_backend for hardware access
add_executable(at_command_test.elf at_command_test.c)
target_link_libraries(at_command_test.elf
    PRIVATE
        at_command
        profiler_backend
        swc
)
add_custom_command(TARGET at_command_test.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary at_command_test.elf at_command_test.bin
)
