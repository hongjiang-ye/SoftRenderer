cmake_minimum_required(VERSION 3.14)

project (SoftRenderer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(render render.cpp)

target_include_directories(render
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
)

# Use OpenMP for multithreading, if availale
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(render PUBLIC OpenMP::OpenMP_CXX)
endif()
