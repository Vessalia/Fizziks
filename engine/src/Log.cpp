#include <Fizziks/Log.h>
#include <Fizziks/FizzLog.h>

namespace Fizziks::internal
{
LogState& globalLogState()
{
	static LogState instance; // magic static ala C++11 is thread-safe
	return instance;
}
}

namespace Fizziks
{
void addLogSink(LogSink sink, SinkOptions options)
{
	auto& state = internal::globalLogState();
	std::unique_lock lock(state.mutex);

	internal::RegisteredSink registered;
	registered.sink  = std::move(sink);
	registered.mutex = options.threadSafe ? nullptr : std::make_shared<std::mutex>();

	if (!options.multiLevel) 
	{
		state.sinks[static_cast<int>(options.level)].push_back(registered);
	}
	else
	{
		for (int i = static_cast<int>(LogLevel::NONE) + 1; i <= static_cast<int>(options.level); ++i)
		{
			state.sinks[i].push_back(registered);
		}
	}
}

void clearLogSinks()
{
	auto& state = internal::globalLogState();
	std::unique_lock lock(state.mutex);
	for (auto& level : state.sinks)
		level.clear();
}
}
