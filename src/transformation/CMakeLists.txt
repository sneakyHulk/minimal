cmake_minimum_required(VERSION 3.22)

##### image_to_world ###################################################################################################

project(image_to_world)

find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        Detection2D
        flatbuffers
        CompactObject
        nlohmann_json::nlohmann_json
        Eigen3::Eigen
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Detection2D GENERATE_CompactObject)

##### transformation_display ########################################################################################################

project(transformation_display)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
        ${OpenCV_LIBS}
        Image
        CompactObject
        flatbuffers
        OpenDrive
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        CvPlot
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(${PROJECT_NAME} GENERATE_Image GENERATE_CompactObject)