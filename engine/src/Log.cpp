#include <Fizziks/InternalLog.h>
#include <Fizziks/Log.h>

namespace Fizziks::internal
{
LogState& globalLogState()
{
	static LogState instance;
	return instance;
}
}

namespace Fizziks
{
void addLogSink(LogSink sink, LogLevel maxlevel)
{
	auto& sinks = internal::globalLogState().sinks;
	for (int i = static_cast<int>(LogLevel::NONE) + 1; i <= static_cast<int>(maxlevel); ++i) 
	{
		sinks[i].push_back(sink);
	}
}

void clearLogSinks()
{
	for (auto& level : internal::globalLogState().sinks)
	{
		level.clear();
	}
}
}
