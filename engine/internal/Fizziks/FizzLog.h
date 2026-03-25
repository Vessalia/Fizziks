#pragma once

#include <Fizziks/Log.h>
#include <array>
#include <vector>
#include <format>
#include <source_location>
#include <shared_mutex>

namespace Fizziks::internal
{
constexpr int LevelCount = static_cast<int>(LogLevel::DEBUG) + 1;

struct RegisteredSink
{
	LogSink sink;
	std::shared_ptr<std::mutex>  mutex; // non-null if not thread-safe
};

struct LogState
{
	std::array<std::vector<RegisteredSink>, LevelCount> sinks;
	mutable std::shared_mutex mutex;
};

LogState& globalLogState();

template<LogLevel L, typename... Args>
inline void emit(std::source_location loc, std::format_string<Args...> fmt, Args&&... args)
{
	if constexpr (static_cast<int>(L) <= ACTIVE_LOG_LEVEL)
	{
		auto& state = globalLogState();

		std::vector<RegisteredSink> localSinks;
		{
			std::shared_lock lock(state.mutex);
			auto& sinks = state.sinks[static_cast<int>(L)];
			if (sinks.empty()) return;
			localSinks = sinks;
		}

		std::string msg = std::format(fmt, std::forward<Args>(args)...);
		for (auto& registered : localSinks)
		{
			if (registered.mutex)
			{
				std::unique_lock sinkLock(*registered.mutex);
				registered.sink(L, msg, loc.file_name(), loc.line());
			}
			else
			{
				registered.sink(L, msg, loc.file_name(), loc.line());
			}
		}
	}
}
}

#define FIZZIKS_LOG(level, fmt, ...)   ::Fizziks::internal::emit<level>(std::source_location::current(), fmt, ##__VA_ARGS__)

#define FIZZIKS_LOG_CRITICAL(fmt, ...) FIZZIKS_LOG(Fizziks::LogLevel::CRITICAL, fmt, ##__VA_ARGS__)
#define FIZZIKS_LOG_ERROR(fmt, ...)    FIZZIKS_LOG(Fizziks::LogLevel::ERROR,    fmt, ##__VA_ARGS__)
#define FIZZIKS_LOG_WARNING(fmt, ...)  FIZZIKS_LOG(Fizziks::LogLevel::WARNING,  fmt, ##__VA_ARGS__)
#define FIZZIKS_LOG_INFO(fmt, ...)     FIZZIKS_LOG(Fizziks::LogLevel::INFO,     fmt, ##__VA_ARGS__)
#define FIZZIKS_LOG_DEBUG(fmt, ...)    FIZZIKS_LOG(Fizziks::LogLevel::DEBUG,    fmt, ##__VA_ARGS__)
