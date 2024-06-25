project(test_dispatcher)

add_subdirectory(src)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        test_yolo_dispatcher
        test_camera_dispatcher
        test_visualization_dispatcher
        test_transformation_dispatcher
        test_tracking_dispatcher
)
set_target_properties(${PROJECT_NAME}
        PROPERTIES
        CXX_STANDARD 23
        CXX_EXTENSIONS off
)
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++20)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")