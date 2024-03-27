add_executable(bio_combo Schedule_FM_DAC_old.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(bio_combo pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_spi hardware_sync hardware_irq hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(bio_combo)
