#ifndef LOGSERVICE
#define LOGSERVICE

#include "../includes.h"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace LogServiceNS
{
	using Clock = std::chrono::steady_clock;
	using std::chrono::time_point;
	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::hours;
	using std::chrono::seconds;
	using std::chrono::minutes;
	using std::chrono::milliseconds;
	using std::chrono::nanoseconds;
	using namespace std::literals::chrono_literals;
	using std::this_thread::sleep_for;

	class LogService
	{
	private:
		std::ofstream mLogFile;
		std::string mLogFilePath_1;
		std::string mLogFilePath_2;
		std::string mLogFilePath;

		static std::unique_ptr<LogService> inline mInstance{ nullptr };

		LogService();

		static std::mutex LogMutex;

		static uintmax_t MaxLogFileSize;

		time_point<Clock> mStartTimestamp{};

		const std::string currentDate();
		const std::string currentTime();

		ErrorCode OpenFile(std::string filePath);

		friend std::unique_ptr<LogService> std::make_unique<LogService>();

	public:

		LogService& operator=(LogService&) = delete;
		LogService(LogService&) = delete;
		LogService(LogService&&) = delete;

		~LogService();

		static std::unique_ptr<LogService>& Instance();

		/*Use StartTimer to start an internal timer.*/
		void StartTimer();

		/*Use StopTimer to stop and log the elapsed time. 
		StartTimer() must be used prior to StopTimer.*/
		void StopTimer();

		ErrorCode LogInfo(std::string text);
	};
}
#endif // !LOGSERVICE