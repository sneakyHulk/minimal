project(test_yolo_dispatcher)

find_package(OpenCV REQUIRED)

if (IS_DIRECTORY ${HOME_DIRECTORY}/src/libtorch)
    set(TORCH_CUDA_ARCH_LIST "8.0 8.6 8.9 9.0") # new CUDA arch is 9.0a, therefore the CAFFE2 regex failed, uff
    find_package(Torch REQUIRED PATHS ~/src/libtorch NO_DEFAULT_PATH)
    message("${TORCH_LIBRARIES}")
    message("${TORCH_INCLUDE_DIRS}")
endif ()

add_library(${PROJECT_NAME} STATIC yolo.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/test/dispatcher_queue/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
        common
        ${OpenCV_LIBS}
        test_node_dispatcher
        test_msg_dispatcher
)

target_include_directories(${PROJECT_NAME} PUBLIC ${TORCH_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${TORCH_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PRIVATE ${TORCH_CXX_FLAGS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")