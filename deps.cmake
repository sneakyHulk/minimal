##### test_UserMatDebugAllocator #######################################################################################

project(test_UserMatDebugAllocator)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} test/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        ${OpenCV_LIBS}
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

##### common_messages ##################################################################################################

project(common_messages)

find_package(msgpack REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
        INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} INTERFACE msgpackc)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)

