file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE INCLUDES CONFIGURE_DEPENDS include/Fizziks/*.h)
file(GLOB_RECURSE INTERNALS CONFIGURE_DEPENDS internal/Fizziks/*.h)

add_executable(Fizziks ${SOURCES} ${INCLUDES} ${INTERNALS})

target_include_directories(Fizziks PUBLIC include/Fizziks
                                   PRIVATE internal/Fizziks)

target_link_libraries(Fizziks PRIVATE Eigen SDL3::SDL3)
