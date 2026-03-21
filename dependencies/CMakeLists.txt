add_library(Eigen3 INTERFACE)

add_library(Eigen3::Eigen ALIAS Eigen3)

target_include_directories(Eigen3
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/eigen>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/eigen>
)

add_library(glm INTERFACE)

add_library(glm::glm ALIAS glm)

target_include_directories(glm
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/glm>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/glm>
)
