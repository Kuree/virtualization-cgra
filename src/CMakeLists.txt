add_library(virtualization graph.cc graph.hh io.cc io.hh pass.cc pass.hh)

target_link_libraries(virtualization fmt)
target_include_directories(virtualization PUBLIC ../extern/fmt/include)