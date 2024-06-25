project(test_msg_dispatcher)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 23)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/test/dispatcher_queue/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${OpenCV_LIBS}
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} INTERFACE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")