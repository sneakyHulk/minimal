cmake_minimum_required(VERSION 3.28)
project(minimal)

set(CMAKE_CXX_STANDARD 23)

get_filename_component(HOME_DIRECTORY ~ ABSOLUTE)

add_subdirectory(thirdparty/flatbuffers)
add_subdirectory(thirdparty/argparse)
add_subdirectory(thirdparty/libOpenDRIVE)
set_property(TARGET OpenDrive PROPERTY POSITION_INDEPENDENT_CODE ON)

add_subdirectory(thirdparty/cvplot/CvPlot)
add_subdirectory(thirdparty/json)
add_subdirectory(thirdparty/ranges-v3)

#set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
#set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
#set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
#set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_TESTS OFF)
set(AUTODIFF_BUILD_PYTHON OFF)
set(AUTODIFF_BUILD_EXAMPLES OFF)
set(AUTODIFF_BUILD_DOCS OFF)
add_subdirectory(thirdparty/autodiff)

add_library(common INTERFACE)
target_include_directories(common INTERFACE thirdparty/common/include)
target_link_libraries(common INTERFACE)
target_compile_features(common INTERFACE cxx_std_23)
find_package(TBB REQUIRED)

##### subdirectories ###################################################################################################

add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(msg)
add_subdirectory(test)


