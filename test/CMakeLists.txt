cmake_minimum_required(VERSION 3.22)

project(test_automatic_differentiation)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} transformation/automatic_differentiation.cpp)
target_link_libraries(${PROJECT_NAME}
        common
        Eigen3::Eigen
        CvPlot
        autodiff::autodiff
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")