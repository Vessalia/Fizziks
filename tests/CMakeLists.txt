include(FetchContent)
FetchContent_Declare(
	googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG        v1.15.2
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
	unit/*.cpp
	integration/*.cpp
	regression/*.cpp
)

add_executable(FizziksTests ${TEST_SOURCES})

target_link_libraries(FizziksTests PRIVATE
	Fizziks::Fizziks
	GTest::gtest_main
)

target_include_directories(FizziksTests PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(FizziksTests PRIVATE cxx_std_20)

add_custom_command(TARGET FizziksTests POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		$<TARGET_FILE:Fizziks>
		$<TARGET_FILE_DIR:FizziksTests>
)

gtest_discover_tests(FizziksTests
	EXTRA_ARGS
		--gtest_catch_exceptions=1
		--gtest_color=yes
	PROPERTIES
		TIMEOUT 120
)
