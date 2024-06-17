cmake_minimum_required(VERSION 3.22)

##### transformation ###################################################################################################

project(transformation)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} INTERFACE ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE
        common
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        nlohmann_json::nlohmann_json
        Eigen3::Eigen
        range-v3::range-v3
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} INTERFACE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

##### image_to_world ###################################################################################################

project(image_to_world)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC)
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        transformation
        Detection2D
        flatbuffers
        CompactObject
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D GENERATE_CompactObject)