project(test_camera_dispatcher)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 23)
add_library(${PROJECT_NAME} SHARED camera.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/test/dispatcher_queue/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        ${OpenCV_LIBS}
        test_msg_dispatcher
        test_node_dispatcher
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")