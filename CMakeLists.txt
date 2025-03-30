add_executable(MotorSync)

# must match with executable name and source file names
target_sources(MotorSync PRIVATE main.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(MotorSync pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pwm hardware_irq hardware_adc hardware_i2c hardware_timer)

# create map/bin/hex file etc.
pico_add_extra_outputs(MotorSync)