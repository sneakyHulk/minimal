cmake_minimum_required(VERSION 3.22)

##### tracking #########################################################################################################

project(tracking)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME} INTERFACE
        ecal
        common
        transformation
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} INTERFACE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

##### tracking_node ####################################################################################################

project(tracking_node)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ecal
        common
        tracking
        Detection2D
        flatbuffers
        ImageTrackerResult
        CompactObject
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D GENERATE_ImageTrackerResult GENERATE_CompactObject)