cmake_minimum_required(VERSION 3.22)

get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)

##### minimal_sub ######################################################################################################

project(sub)

add_executable(${PROJECT_NAME} sub.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

##### minimal_sub ######################################################################################################

project(pub)

add_executable(${PROJECT_NAME} pub.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include/${CURRENT_DIR})
target_link_libraries(${PROJECT_NAME}
        ecal
        common
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)