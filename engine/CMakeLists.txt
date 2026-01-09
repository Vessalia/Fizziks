file(GLOB_RECURSE FIZZIKS_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE FIZZIKS_INCLUDES CONFIGURE_DEPENDS include/Fizziks/*.h)
file(GLOB_RECURSE FIZZIKS_INTERNALS CONFIGURE_DEPENDS internal/Fizziks/*.h)

add_library(Fizziks ${FIZZIKS_SOURCES} ${FIZZIKS_INCLUDES} ${FIZZIKS_INTERNALS})

target_include_directories(Fizziks
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal>
)

target_link_libraries(Fizziks PRIVATE Eigen3::Eigen)


target_compile_features(Fizziks PRIVATE cxx_std_20)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(Fizziks PRIVATE FIZZIKS_BUILDING_DLL)
endif()
