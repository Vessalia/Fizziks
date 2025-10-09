cmake_minimum_required(VERSION 3.28...4.1)

project(Fizziks_ngin VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Bin)

add_subdirectory(dependencies)

add_subdirectory(engine)
