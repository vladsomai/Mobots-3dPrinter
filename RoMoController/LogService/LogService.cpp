#ifdef _CRT_SECURE_NO_WARNINGS
#undef _CRT_SECURE_NO_WARNINGS
#endif
#define _CRT_SECURE_NO_WARNINGS 1 /*for localtime and ctime warning*/

#include "LogService.h"
#include <fstream>
#include <filesystem>

namespace LogServiceNS
{
	std::mutex LogService::LogMutex{};
	uint32_t LogService::MaxLogFileSize = 2 * 1024 * 1024;

	LogService::LogService()
	{
		mLogFilePath = currentDate() + ".log";
		std::string logFileName{ mLogFilePath };
		mLogFile.open(logFileName, std::ofstream::out | std::ofstream::app);
	}

	LogService::~LogService()
	{
		if (mLogFile.is_open())
		{
			mLogFile.close();
		}
	}

	std::unique_ptr<LogService>& LogService::Instance()
	{
		std::lock_guard<std::mutex> instanceLock{ LogMutex };


		if (mInstance == nullptr)
		{
			mInstance = std::unique_ptr<LogService>(new LogService);
		}

		return mInstance;
	}

	ErrorCode LogService::LogInfo(std::string text)
	{
		std::lock_guard<std::mutex> instanceLock{ LogMutex };

		if (!mLogFile.is_open())
			return ErrorCode::LOGFILENOTFOUND;

		std::string log = currentDate() + "|" + currentTime() + "|Info|" + text + '\n';
		mLogFile << log;

		mLogFile.flush();

		return ErrorCode::NO_ERR;
	}

	void LogService::StartTimer()
	{
		LogInfo("Timer start");
		mStartTimestamp = Clock::now();
	}

	void LogService::StopTimer()
	{
		time_point<Clock> endTimestamp = Clock::now();

		duration<double> diffDur = duration_cast<duration<double>>(endTimestamp - mStartTimestamp);
		//milliseconds diff = duration_cast<milliseconds>(endTimestamp - mStartTimestamp);
		//nanoseconds diffNs = duration_cast<nanoseconds>(endTimestamp - mStartTimestamp);

		LogInfo("Timer stop");
		LogInfo(std::string("Elapsed time: ") + std::string(std::to_string(diffDur.count())) + std::string("ms"));
		//LogInfo(std::string("Elapsed miliseconds: ") + std::string(std::to_string(diff.count())) + std::string("ms"));
		//LogInfo(std::string("Elapsed nanoseconds: ") + std::string(std::to_string(diffNs.count())) + std::string("ns"));
	}


	const std::string LogService::currentTime()
	{
		const auto now = std::chrono::system_clock::now();
		const auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
		const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
			now.time_since_epoch()) % 1000;

		auto tstruct = *localtime(&nowAsTimeT);

		char       bufTime[80];
		strftime(bufTime, sizeof(bufTime), "%H-%M-%S", &tstruct);

		std::stringstream nowSs;
		nowSs
			<< bufTime
			<< '-' << std::setfill('0') << std::setw(3) << nowMs.count();
		return nowSs.str();
	}

	const std::string LogService::currentDate()
	{
		time_t     now = time(0);
		struct tm  tstruct;
		tstruct = *localtime(&now);

		char       bufDate[80];
		strftime(bufDate, sizeof(bufDate), "%Y-%m-%d", &tstruct);

		return std::string{ bufDate };
	};
}