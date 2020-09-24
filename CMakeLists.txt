cmake_minimum_required(VERSION 3.17)
project(virtualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -fPIC")

add_subdirectory(src)
add_subdirectory(tools)