add_executable(PWM_Demo)

# must match with executable name and source file names
target_sources(PWM_Demo PRIVATE pwm-demo.c vga16_graphics.c)

# must match with pio filename and executable name from above
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(PWM_Demo pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pwm hardware_irq hardware_pio hardware_i2c hardware_dma)

# create map/bin/hex file etc.
pico_add_extra_outputs(PWM_Demo)