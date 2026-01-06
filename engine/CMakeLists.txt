file(GLOB_RECURSE FIZZIKS_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE FIZZIKS_INCLUDES CONFIGURE_DEPENDS include/Fizziks/*.h)
file(GLOB_RECURSE FIZZIKS_INTERNALS CONFIGURE_DEPENDS internal/Fizziks/*.h)

if(FIZZIKS_STATIC)
    add_library(Fizziks STATIC ${FIZZIKS_SOURCES} ${FIZZIKS_INCLUDES} ${FIZZIKS_INTERNALS})
else()
    add_library(Fizziks SHARED ${FIZZIKS_SOURCES} ${FIZZIKS_INCLUDES} ${FIZZIKS_INTERNALS})
endif()

target_include_directories(Fizziks PUBLIC include/Fizziks
                                   PRIVATE internal/Fizziks)

target_link_libraries(Fizziks PRIVATE Eigen)

target_compile_features(Fizziks PRIVATE cxx_std_20)
target_compile_definitions(Fizziks PRIVATE FIZZIKS_BUILDING_DLL)
