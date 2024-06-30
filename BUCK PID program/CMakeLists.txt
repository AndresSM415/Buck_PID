
cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

project(BUCK_2_PID)
add_executable (BUCK_2_PID BUCK_2_PID.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries (BUCK_2_PID pico_stdlib)
target_link_libraries (BUCK_2_PID pico_stdlib
                            hardware_gpio hardware_pwm hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(BUCK_2_PID)