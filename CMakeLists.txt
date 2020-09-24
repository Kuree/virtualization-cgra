cmake_minimum_required(VERSION 3.17)
project(virtualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -fPIC")

add_library(virtualization graph.cc graph.hh io.cc io.hh)

add_executable(partition main.cpp)