project(test_tracking_dispatcher)

add_library(${PROJECT_NAME} SHARED tracking.cpp KalmanFilter.cpp KalmanBoxTracker.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/test/dispatcher_queue/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE
        common
        test_node_dispatcher
        test_msg_dispatcher
        transformation
)
set_target_properties(${PROJECT_NAME}
        PROPERTIES
        CXX_STANDARD 23
        CXX_EXTENSIONS off
)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")