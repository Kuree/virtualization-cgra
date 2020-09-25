cmake_minimum_required(VERSION 3.11)
project(virtualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -fPIC")

# extern libraries
add_subdirectory(extern/fmt)

add_subdirectory(src)
add_subdirectory(tools)

# tests
add_subdirectory(extern/googletest/)
include(GoogleTest)
include (CTest)
enable_testing()
add_subdirectory(tests)