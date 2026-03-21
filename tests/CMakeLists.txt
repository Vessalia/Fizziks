include(FetchContent)
FetchContent_Declare(
	googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG        v1.15.2
)
FetchContent_MakeAvailable(googletest)

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

target_compile_features(FizziksTests PRIVATE cxx_std_20)

include(GoogleTest)
gtest_discover_tests(FizziksTests)
