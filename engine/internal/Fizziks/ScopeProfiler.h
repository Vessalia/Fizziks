#pragma once
#include <chrono>
#include <iostream>
#include <string_view>
#include <unordered_map>
#include <deque>

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

	ScopeProfiler(std::string_view label, const char* functionSignature, bool useAverage = false, int windowSize = 100)
	: label(label)
	, functionSignature(functionSignature)
	, start(clock::now())
	, useAverage(useAverage)
	, windowSize(windowSize) { }

	~ScopeProfiler()
	{
		const auto end = clock::now();
		const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

		if (useAverage)
		{
			auto window = averages[label];

			averages[label].push_front(microseconds);
			while (averages[label].size() > windowSize)
			{
				averages[label].pop_back();
			}

			long long average = 0;
			for (long long measurement : window)
			{
				average += measurement;
			}
			average /= window.size();

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
	static std::unordered_map<std::string_view, std::deque<long long>> averages;

	bool useAverage;
	uint16_t windowSize;
	std::string_view label;
	const char* functionSignature;
	clock::time_point start;
};

#define PROFILE_CONCAT_INNER(a, b) a##b
#define PROFILE_CONCAT(a, b) PROFILE_CONCAT_INNER(a, b)

#define PROFILE_SCOPE(label) \
	ScopeProfiler PROFILE_CONCAT(scopeProfiler, __LINE__)(label, PROFILE_FUNCTION_SIGNATURE)

#define PROFILE_SCOPE_AVG(label) \
	ScopeProfiler profile = PROFILE_CONCAT(scopeProfiler, __LINE__)(label, PROFILE_FUNCTION_SIGNATURE, true)

#define PROFILE_SCOPE_AVG(label, windowSize) \
	ScopeProfiler profile = PROFILE_CONCAT(scopeProfiler, __LINE__)(label, PROFILE_FUNCTION_SIGNATURE, true, windowSize)

#define PROFILE_FUNCTION() \
	PROFILE_SCOPE(__func__)

#define PROFILE_FUNCTION_AVG() \
	PROFILE_SCOPE_AVG(__func__)

#define PROFILE_FUNCTION_AVG(windowSize) \
	PROFILE_SCOPE_AVG(__func__, windowSize)
