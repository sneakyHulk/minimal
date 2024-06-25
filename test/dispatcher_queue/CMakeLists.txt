project(test_dispatcher)

add_subdirectory(src)

set(CMAKE_CXX_STANDARD 23)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        test_yolo_dispatcher
        test_camera_dispatcher
        test_visualization_dispatcher
        test_transformation_dispatcher
        test_tracking_dispatcher
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")