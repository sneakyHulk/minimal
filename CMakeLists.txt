cmake_minimum_required(VERSION 3.22)
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

##### common_messages ##################################################################################################

project(common_messages)

find_package(msgpack REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
        INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} INTERFACE msgpackc)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)

##### minimal_sub ######################################################################################################

project(minimal_sub)

find_package(eCAL REQUIRED)

add_executable(${PROJECT_NAME} src/sub.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME}
        eCAL::core
        common_messages
        common
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

##### minimal_sub ######################################################################################################

project(minimal_pub)

find_package(eCAL REQUIRED)

add_executable(${PROJECT_NAME} src/pub.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME}
        eCAL::core
        common_messages
        common
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)