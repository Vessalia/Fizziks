#pragma once
#include <Fizziks/Fizziks.h>
#include <functional>
#include <string_view>

namespace Fizziks
{
enum class LogLevel // inclusive levelling. Use DEBUG for all levels
{
	NONE, // optional setting for no logging
	CRITICAL, // program breaking problems
	ERROR, WARNING, // issues that may or may not require attention
	INFO, DEBUG, // high level logging mostly for development
};

constexpr std::string_view toString(LogLevel level) noexcept
{
	switch (level)
	{
		case LogLevel::NONE:	 return "NONE";
		case LogLevel::CRITICAL: return "CRITICAL";
		case LogLevel::ERROR:	 return "ERROR";
		case LogLevel::WARNING:  return "WARNING";
		case LogLevel::INFO:	 return "INFO";
		case LogLevel::DEBUG:	 return "DEBUG";
		default:				 return "UNKNOWN";
	}
}

using LogSink = std::function<void(LogLevel, std::string_view msg, std::string_view file, int line)>;
struct SinkOptions
{
	LogLevel level  = LogLevel::DEBUG;
	bool multiLevel = true; // set this option to false if your sink should only hear signals at the requested level
	bool threadSafe = false; // set this option to true if your sink is thread-safe
};

FIZZIKS_API void addLogSink(LogSink sink, SinkOptions options = {});
FIZZIKS_API void clearLogSinks();
}

#ifndef ACTIVE_LOG_LEVEL
#define ACTIVE_LOG_LEVEL Fizziks::LogLevel::ERROR
#endif
