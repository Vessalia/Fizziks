cmake_minimum_required(VERSION 3.28...4.1)

project(Fizziks VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Collect all source files in src/
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_executable(Fizziks ${SOURCES})

target_include_directories(Fizziks PRIVATE include/Fizziks)

add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/eigen)
target_link_libraries(Fizziks PRIVATE eigen)
