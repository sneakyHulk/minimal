cmake_minimum_required(VERSION 3.22)

##### common_messages ##################################################################################################

project(ecal)

find_package(eCAL REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
        INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} INTERFACE eCAL::core)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)