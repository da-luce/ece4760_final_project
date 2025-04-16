add_executable(LIDAR)

# must match with executable name and source file names
target_sources(LIDAR PRIVATE main.c vga16_graphics.c)

# must match with pio filename and executable name from above
# VGA PIOs
pico_generate_pio_header(LIDAR ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(LIDAR ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(LIDAR ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# Stepper PIOs
pico_generate_pio_header(LIDAR ${CMAKE_CURRENT_LIST_DIR}/stepper.pio)
pico_generate_pio_header(LIDAR ${CMAKE_CURRENT_LIST_DIR}/pacer.pio)
pico_generate_pio_header(LIDAR ${CMAKE_CURRENT_LIST_DIR}/counter.pio)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(LIDAR pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pwm hardware_irq hardware_pio hardware_i2c hardware_dma)

# create map/bin/hex file etc.
pico_add_extra_outputs(LIDAR)