project(test_msg_dispatcher)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/test/dispatcher_queue/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${OpenCV_LIBS}
)
set_target_properties(${PROJECT_NAME}
        PROPERTIES
        CXX_STANDARD 23
        CXX_EXTENSIONS off
)
target_compile_definitions(${PROJECT_NAME} INTERFACE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")