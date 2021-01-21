# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

target_sources(app PRIVATE
        pwm_switch_driver.c
        main.c
        hrtim.c
)

add_subdirectory(ext)

if(${CONFIG_CUSTOM_DATA_NODES_FILE})
        target_sources(app PRIVATE data_nodes_custom.cpp)
endif()

# Using this option enables user-specific customization of the firmware. The
# custom subfolder should have its own CMakeLists.txt file.
if(${CONFIG_CUSTOMIZATION})
        add_subdirectory(custom build/custom)
endif()
