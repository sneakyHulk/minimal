cmake_minimum_required(VERSION 3.28)
project(minimal)

set(CMAKE_CXX_STANDARD 23)

##### CPM ##############################################################################################################

set(CPM_DOWNLOAD_VERSION 0.39.0)
set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")

if (NOT (EXISTS ${CPM_DOWNLOAD_LOCATION}))
    message(STATUS "Downloading CPM.cmake")
    file(DOWNLOAD https://github.com/TheLartians/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake ${CPM_DOWNLOAD_LOCATION})
endif ()

include(${CPM_DOWNLOAD_LOCATION})

CPMAddPackage(
        NAME common
        GITHUB_REPOSITORY sneakyHulk/common
        GIT_TAG origin/main
        DOWNLOAD_ONLY YES
)

add_library(common INTERFACE)
target_include_directories(common INTERFACE ${common_SOURCE_DIR}/include)
target_link_libraries(common INTERFACE)
target_compile_features(common INTERFACE cxx_std_23)

CPMAddPackage(
        NAME argparse
        GITHUB_REPOSITORY p-ranav/argparse
        VERSION 3.0
)

CPMAddPackage(
        NAME flatbuffers
        GITHUB_REPOSITORY google/flatbuffers
        VERSION 24.3.25
)

CPMAddPackage(
        NAME nlohmann_json
        GITHUB_REPOSITORY nlohmann/json
        VERSION 3.11.3)

##### subdirectories ###################################################################################################

add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(msg)