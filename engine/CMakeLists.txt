file(GLOB_RECURSE FIZZIKS_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE FIZZIKS_INCLUDES CONFIGURE_DEPENDS include/Fizziks/*.h)
file(GLOB_RECURSE FIZZIKS_INTERNALS CONFIGURE_DEPENDS internal/Fizziks/*.h)

add_library(Fizziks ${FIZZIKS_SOURCES} ${FIZZIKS_INCLUDES} ${FIZZIKS_INTERNALS})
add_library(Fizziks::Fizziks ALIAS Fizziks)

target_include_directories(Fizziks
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/internal>
)

target_compile_features(Fizziks PUBLIC cxx_std_20)

# Warning flags
if(MSVC)
	target_compile_options(Fizziks PRIVATE
		/W4             # high warning level
		/WX             # warnings as errors
		/permissive-    # strict standard conformance
		/MP             # parallel compilation
		/Zc:__cplusplus # report correct __cplusplus value
		/wd4251         # disable C4251: needs dll-interface for STL members
	)
else()
	target_compile_options(Fizziks PRIVATE
		-Wall
		-Wextra
		-Wpedantic
		-Werror
	)
endif()

# Allow manual override of log level (-1 means "use config default")
set(FIZZIKS_LOG_LEVEL -1 CACHE STRING "Override ACTIVE_LOG_LEVEL (leave -1 to use config defaults)")

if(NOT FIZZIKS_LOG_LEVEL EQUAL -1)
    if(FIZZIKS_LOG_LEVEL LESS 0 OR FIZZIKS_LOG_LEVEL GREATER 5)
        message(FATAL_ERROR "FIZZIKS_LOG_LEVEL must be -1 (default) or in range [0, 5]")
    endif()
endif()

# Compile definitions
target_compile_definitions(Fizziks
    PRIVATE
        $<$<BOOL:${BUILD_SHARED_LIBS}>:FIZZIKS_BUILDING_DLL>
        $<IF:$<EQUAL:${FIZZIKS_LOG_LEVEL},-1>,
            $<$<CONFIG:Debug>:          ACTIVE_LOG_LEVEL=5>
            $<$<CONFIG:RelWithDebInfo>: ACTIVE_LOG_LEVEL=2>
            $<$<CONFIG:Release>:        ACTIVE_LOG_LEVEL=1>
            $<$<CONFIG:MinSizeRel>:     ACTIVE_LOG_LEVEL=1>,
            ACTIVE_LOG_LEVEL=${FIZZIKS_LOG_LEVEL}
        >
)

# Dependencies
option(FIZZIKS_USE_GLM "Use GLM as the math backend instead of Eigen" OFF)

if(FIZZIKS_USE_GLM)
    target_link_libraries(Fizziks PRIVATE glm::glm)
    target_compile_definitions(Fizziks PRIVATE FIZZIKS_USE_GLM)
else()
    target_link_libraries(Fizziks PRIVATE Eigen3::Eigen)
endif()
