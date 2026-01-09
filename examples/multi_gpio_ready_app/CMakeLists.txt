cmake_minimum_required(VERSION 3.16)

# Indique à ESP-IDF où trouver les composants additionnels
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(vl53l0x_multi_gpio_ready_app)
