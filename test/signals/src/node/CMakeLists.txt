project(test_node)

find_package(TBB REQUIRED)

add_library(${PROJECT_NAME} SHARED node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/test/signals/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        TBB::tbb
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")