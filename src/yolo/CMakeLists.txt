cmake_minimum_required(VERSION 3.28)

##### yolo #############################################################################################################

project(yolo)

set(TORCH_CUDA_ARCH_LIST "8.0 8.6 8.9 9.0") # new CUDA arch is 9.0a, therefore the CAFFE2 regex failed, uff
set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
if (IS_DIRECTORY ${HOME_DIRECTORY}/src/libtorch)
    find_package(Torch REQUIRED PATHS ~/src/libtorch NO_DEFAULT_PATH)
    message("${TORCH_LIBRARIES}")
    message("${TORCH_INCLUDE_DIRS}")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME} INTERFACE
        common
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} INTERFACE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

##### yolo_node ########################################################################################################

project(yolo_node)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC)
target_link_libraries(${PROJECT_NAME}
        ecal
        yolo
        common
        flatbuffers
        Detection2D
        Image
        CompactObject
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D GENERATE_Image GENERATE_CompactObject)