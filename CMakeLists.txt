cmake_minimum_required(VERSION 3.21)
project(PGSE)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.8 REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(PGSE
        ${HEADERS}
        ${SOURCES}
        )

target_link_libraries(PGSE ${PCL_LIBRARIES})
