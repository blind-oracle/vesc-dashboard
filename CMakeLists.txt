# boat-motor: ESP-IDF project, driven by PlatformIO (pio run). Project components live in
# components/ (u8g2 is a git submodule there) and are discovered by ESP-IDF automatically.
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(boat-motor)
