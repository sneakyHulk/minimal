cmake_minimum_required(VERSION 3.22)

##### camera_simulator #################################################################################################

project(camera_simulator)

find_package(OpenCV REQUIRED)
find_package(argparse REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        ${OpenCV_LIBS}
        argparse
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

##### image_display ####################################################################################################

project(image_display)

#find_package(eCAL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(argparse REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        ${OpenCV_LIBS}
        argparse
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)