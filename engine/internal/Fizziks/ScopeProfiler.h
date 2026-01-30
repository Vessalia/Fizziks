#pragma once
#include <chrono>
#include <iostream>
#include <string_view>

#if defined(_MSC_VER)
#define PROFILE_FUNCTION_SIGNATURE __FUNCSIG__
#elif defined(__clang__) || defined(__GNUC__)
#define PROFILE_FUNCTION_SIGNATURE __PRETTY_FUNCTION__
#else
#define PROFILE_FUNCTION_SIGNATURE __func__
#endif

class ScopeProfiler
{
public:
    using clock = std::chrono::steady_clock;

    ScopeProfiler(std::string_view label, const char* functionSignature)
        : label(label)
        , functionSignature(functionSignature)
        , start(clock::now()) { }

    ~ScopeProfiler()
    {
        const auto end = clock::now();
        const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        std::cout << "[PROFILE] " << label
            << " | " << functionSignature
            << " | " << microseconds << " us\n";
    }

    ScopeProfiler(const ScopeProfiler&) = delete;
    ScopeProfiler& operator=(const ScopeProfiler&) = delete;

private:
    std::string_view label;
    const char* functionSignature;
    clock::time_point start;
};

#define PROFILE_CONCAT_INNER(a, b) a##b
#define PROFILE_CONCAT(a, b) PROFILE_CONCAT_INNER(a, b)

#define PROFILE_SCOPE(label) \
    ScopeProfiler PROFILE_CONCAT(scopeProfiler, __LINE__)(label, PROFILE_FUNCTION_SIGNATURE)

#define PROFILE_FUNCTION() \
    PROFILE_SCOPE(__func__)
