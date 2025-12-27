file(GLOB_RECURSE FIZZIKS_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE FIZZIKS_INCLUDES CONFIGURE_DEPENDS include/Fizziks/*.h)
file(GLOB_RECURSE FIZZIKS_INTERNALS CONFIGURE_DEPENDS internal/Fizziks/*.h)

# add_executable(Fizziks 
#     ${FIZZIKS_SOURCES} 
#     ${FIZZIKS_INCLUDES} 
#     ${FIZZIKS_INTERNALS}
# )

add_library(Fizziks STATIC ${FIZZIKS_SOURCES})

target_include_directories(Fizziks PUBLIC include/Fizziks
                                   PRIVATE internal/Fizziks)

target_link_libraries(Fizziks PRIVATE Eigen SDL3::SDL3)

target_compile_features(Fizziks PUBLIC cxx_std_20)
