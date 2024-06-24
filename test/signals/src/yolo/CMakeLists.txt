project(test_yolo_signals)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)

add_library(${PROJECT_NAME} SHARED yolo.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/test/signals/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        TBB::tbb
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")