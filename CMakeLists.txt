cmake_minimum_required(VERSION 3.28)
project(minimal)

set(CMAKE_CXX_STANDARD 23)

add_subdirectory(thirdparty/flatbuffers)
add_subdirectory(thirdparty/argparse)

add_library(common INTERFACE)
target_include_directories(common INTERFACE thirdparty/common/include)
target_link_libraries(common INTERFACE)
target_compile_features(common INTERFACE cxx_std_23)

##### subdirectories ###################################################################################################

add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(msg)