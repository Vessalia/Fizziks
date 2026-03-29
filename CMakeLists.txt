cmake_minimum_required(VERSION 3.28...4.2)

project(Fizziks_ngin VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(BUILD_SHARED_LIBS          "Build Fizziks as a shared library" ON)
option(FIZZIKS_BUILD_DIST         "Copy outputs to dist folder"       OFF)
option(FIZZIKS_BUILD_TESTS        "Build Fizziks tests"               OFF)

# stolen from goldDaniel's GoldEngine CMake

# Git submodules
find_package(Git QUIET)

if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
	execute_process(
		COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		RESULT_VARIABLE GIT_SUBMOD_RESULT
	)
	if(NOT GIT_SUBMOD_RESULT EQUAL "0")
		message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
	endif()
endif()


# IDE folder support
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

define_property(TARGET PROPERTY FOLDER INHERITED
	BRIEF_DOCS "Set the folder name."
	FULL_DOCS  "Use to organize targets in an IDE."
)

function(add_subdirectory_with_folder _folder_name _folder)
	add_subdirectory(${_folder} ${ARGN})
	set_property(DIRECTORY "${_folder}" PROPERTY FOLDER "${_folder_name}")
endfunction()


# Subdirectories
add_subdirectory_with_folder(dependencies dependencies)
add_subdirectory_with_folder(engine engine)

# Distributable packaging
if(FIZZIKS_BUILD_DIST)
	set(DIST_DIR ${CMAKE_BINARY_DIR}/dist/Fizziks)
	set(DIST_INCLUDE_DIR ${DIST_DIR}/include/Fizziks)

	set(DIST_LIB_DIR ${DIST_DIR}/lib/$<CONFIG>)
	if(BUILD_SHARED_LIBS)
		set(DIST_BIN_DIR ${DIST_DIR}/bin/$<CONFIG>)
		set(DIST_PDB_DIR ${DIST_DIR}/bin/$<CONFIG>)
	else()
		set(DIST_BIN_DIR ${DIST_DIR}/lib/$<CONFIG>)
		set(DIST_PDB_DIR ${DIST_DIR}/lib/$<CONFIG>)
	endif()

	# Set output directories for your library target
	set_target_properties(Fizziks PROPERTIES
		ARCHIVE_OUTPUT_DIRECTORY ${DIST_LIB_DIR}  # .lib
		LIBRARY_OUTPUT_DIRECTORY ${DIST_BIN_DIR}  # .dll
		RUNTIME_OUTPUT_DIRECTORY ${DIST_BIN_DIR}  # .dll
		PDB_OUTPUT_DIRECTORY     ${DIST_PDB_DIR}  # .pdb
	)

	# Copy public headers
	add_custom_target(fizziks_copy_headers ALL
		COMMAND ${CMAKE_COMMAND} -E copy_directory
			"${CMAKE_SOURCE_DIR}/engine/include/Fizziks"
			"${DIST_INCLUDE_DIR}"
		COMMENT "Copying Fizziks public headers to dist"
	)
endif()

# Install rules

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Library binaries
install(TARGETS Fizziks
	EXPORT FizziksTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/Fizziks
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(FIZZIKS_USE_GLM)
	install(DIRECTORY dependencies/glm/glm
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	)
else()
	install(DIRECTORY dependencies/eigen/Eigen
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	)
endif()

install(DIRECTORY
	${CMAKE_CURRENT_SOURCE_DIR}/dependencies/eigen/Eigen
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/eigen
)

if(MSVC)
	install(FILES $<TARGET_PDB_FILE:Fizziks>
		DESTINATION ${CMAKE_INSTALL_BINDIR}
		OPTIONAL
	)
endif()

# Cmake package config
install(EXPORT FizziksTargets
	FILE FizziksTargets.cmake
	NAMESPACE Fizziks::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Fizziks
)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/FizziksConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/FizziksConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Fizziks
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/FizziksConfigVersion.cmake
	VERSION       ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

install(FILES
	"${CMAKE_CURRENT_BINARY_DIR}/FizziksConfig.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Fizziks
)

# Tests
if(FIZZIKS_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()
