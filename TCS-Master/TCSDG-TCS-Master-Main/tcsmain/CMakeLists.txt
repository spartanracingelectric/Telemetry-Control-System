cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS 
../components/dht
../components/esp_idf_lib_helpers
../components/icm42670
../components/i2cdev
../components/ra01s
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ssd1306)
