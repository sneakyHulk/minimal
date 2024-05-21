cmake_minimum_required(VERSION 3.28)

##### yolo #############################################################################################################

project(yolo)


#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
if (IS_DIRECTORY ${HOME_DIRECTORY}/src/libtorch)
    find_package(Torch REQUIRED PATHS ~/src/libtorch NO_DEFAULT_PATH)
    message("${TORCH_LIBRARIES}")
    message("${TORCH_INCLUDE_DIRS}")
endif ()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
        flatbuffers
        Detection2D
        Image
        CompactObject
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D GENERATE_Image GENERATE_CompactObject)

##### yolo_test ########################################################################################################

project(yolo_test)

find_package(OpenCV REQUIRED)
if (IS_DIRECTORY ${HOME_DIRECTORY}/src/libtorch)
    find_package(Torch REQUIRED PATHS ~/src/libtorch NO_DEFAULT_PATH)
    message("${TORCH_LIBRARIES}")
    message("${TORCH_INCLUDE_DIRS}")
endif ()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
