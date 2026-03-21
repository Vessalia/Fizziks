#pragma once

#include <Fizziks/Log.h>
#include <array>
#include <vector>
#include <format>
#include <source_location>

namespace Fizziks::internal
{
constexpr int LevelCount = static_cast<int>(LogLevel::DEBUG) + 1;

struct LogState 
{
	std::array<std::vector<LogSink>, LevelCount> sinks;
};

LogState& globalLogState();

template<LogLevel L, typename... Args>
inline void emit(std::format_string<Args...> fmt, Args&&... args, std::source_location loc = std::source_location::current())
{
	if constexpr (static_cast<int>(L) <= ACTIVE_LOG_LEVEL)
	{
		auto& sinks = globalLogState().sinks[static_cast<int>(L)];
		if (!sinks.empty())
		{
			std::string msg = std::format(fmt, std::forward<Args>(args)...);
			for (auto& sink : sinks)
				sink(L, msg, loc.file_name(), loc.line());
		}
	}
};
}

#define FIZZIKS_LOG(level, ...)   ::Fizziks::internal::emit<level>(__VA_ARGS__)
#define FIZZIKS_LOG_CRITICAL(...) FIZZIKS_LOG(Fizziks::LogLevel::CRITICAL, __VA_ARGS__)
#define FIZZIKS_LOG_ERROR(...)    FIZZIKS_LOG(Fizziks::LogLevel::ERROR,    __VA_ARGS__)
#define FIZZIKS_LOG_WARNING(...)  FIZZIKS_LOG(Fizziks::LogLevel::WARNING,  __VA_ARGS__)
#define FIZZIKS_LOG_INFO(...)     FIZZIKS_LOG(Fizziks::LogLevel::INFO,     __VA_ARGS__)
#define FIZZIKS_LOG_DEBUG(...)    FIZZIKS_LOG(Fizziks::LogLevel::DEBUG,    __VA_ARGS__)
