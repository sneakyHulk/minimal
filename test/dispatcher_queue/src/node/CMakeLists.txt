project(test_node_dispatcher)

find_package(TBB REQUIRED)

set(CMAKE_CXX_STANDARD 23)
add_library(${PROJECT_NAME} SHARED node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/test/dispatcher_queue/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        TBB::tbb
)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")