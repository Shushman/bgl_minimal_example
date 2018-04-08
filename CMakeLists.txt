cmake_minimum_required(VERSION 2.8.12)
project(bgl_minimal_example)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Dependencies
find_package(Boost REQUIRED COMPONENTS graph program_options)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(
    ${EIGEN3_DEFINITIONS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${Boost_LIBRARIES})