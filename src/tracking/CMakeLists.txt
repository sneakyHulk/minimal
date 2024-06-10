cmake_minimum_required(VERSION 3.22)

##### tracking #########################################################################################################

project(tracking)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC)
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        transformation
        Detection2D
        flatbuffers
        CompactObject
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D GENERATE_CompactObject)