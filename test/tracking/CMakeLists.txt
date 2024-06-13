cmake_minimum_required(VERSION 3.22)

##### create_detections_file ###########################################################################################

project(create_detections_file)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC)
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        transformation
        Detection2D
        flatbuffers
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D)

##### test_timestamps ##################################################################################################

project(test_timestamps)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC)
target_link_libraries(${PROJECT_NAME}
        common
        nlohmann_json::nlohmann_json
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

##### test_sort ########################################################################################################

project(test_sort)

find_package(OpenCV REQUIRED)

file(GLOB ${PROJECT_NAME}_SRC_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp") #CONFIGURE_DEPENDS

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp ${${PROJECT_NAME}_SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME}
        common
        nlohmann_json::nlohmann_json
        ${OpenCV_LIBS}
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")