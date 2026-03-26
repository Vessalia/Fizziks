#pragma once
#include <Fizziks/Fizziks.h>
#include <type_traits>

// double precision -> half the epsilon
static constexpr val_t eps = std::is_same_v<val_t, float> ? val_t(1.0e-5) : val_t(1.0e-10);

#define EXPECT_VAL_EQ(a, b) EXPECT_NEAR((a), (b), eps)
