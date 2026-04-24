target_link_libraries(audio_unidirectional_backend PRIVATE hardware module_tinyusb_baremetal max98091_driver)
target_sources(audio_unidirectional_backend PRIVATE audio_unidirectional_backend.c)

# Build for AV IND Coordinator (no codec, SAI master)
# target_compile_definitions(audio_unidirectional_backend PRIVATE AV_IND_COORD)
target_compile_definitions(audio_unidirectional_backend PRIVATE SPARK_EVK_NODE)

# Uncomment to build for SPARK EVK Node (with codec, SAI slave)
# target_compile_definitions(audio_unidirectional_backend PRIVATE SPARK_EVK_NODE)
