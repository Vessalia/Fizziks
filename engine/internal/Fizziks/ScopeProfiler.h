#pragma once
#include <chrono>
#include <iostream>
#include <string_view>
#include <unordered_map>

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

    ScopeProfiler(std::string_view label, const char* functionSignature, bool average = false)
        : label(label)
        , functionSignature(functionSignature)
        , start(clock::now())
        , average(average) { }

    ~ScopeProfiler()
    {
        const auto end = clock::now();
        const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        if (average)
        {
            if (!averages.contains(label)) averages[label] = { microseconds, 1 };
            else
            {
                averages[label].first += microseconds;
                ++averages[label].second;
            }

            auto average = averages[label].first / averages[label].second;
            print(average);
        }
        else
        {
            print(microseconds);
        }
    }

    ScopeProfiler(const ScopeProfiler&) = delete;
    ScopeProfiler& operator=(const ScopeProfiler&) = delete;

    void print(long long time) const
    {
        std::cout << "[PROFILE] " << label
        << " | " << functionSignature
        << " | " << time << " us\n";
    }

private:
    static std::unordered_map<std::string_view, std::pair<long long, long long>> averages;

    bool average;
    std::string_view label;
    const char* functionSignature;
    clock::time_point start;
};

#define PROFILE_CONCAT_INNER(a, b) a##b
#define PROFILE_CONCAT(a, b) PROFILE_CONCAT_INNER(a, b)

#define PROFILE_SCOPE(label) \
    ScopeProfiler PROFILE_CONCAT(scopeProfiler, __LINE__)(label, PROFILE_FUNCTION_SIGNATURE)

#define PROFILE_SCOPE_AVG(label) \
    ScopeProfiler PROFILE_CONCAT(scopeProfiler, __LINE__)(label, PROFILE_FUNCTION_SIGNATURE, true)

#define PROFILE_FUNCTION() \
    PROFILE_SCOPE(__func__)

#define PROFILE_FUNCTION_AVG() \
    PROFILE_SCOPE_AVG(__func__)
