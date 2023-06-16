#ifndef SERIAL_PORT
#define SERIAL_PORT

#include "../includes.h"

namespace SerialPortNS
{
	using SerialPortHandle = FILE*;

	class SerialPort
	{

	private:

		SerialPortHandle mPort{ NULL };

		static std::unique_ptr<SerialPort> inline mInstance{ nullptr };

		ErrorCode Send(const std::vector<uint8_t>& command);

		SerialPort() = default;

		static std::mutex SerialPortMutex;

	public:

		SerialPort& operator=(SerialPort& port) = delete;
		SerialPort(SerialPort& port) = delete;
		SerialPort(SerialPort&& port) = delete;

		~SerialPort();

	    static std::unique_ptr<SerialPort>& Instance();

		ErrorCode Connect(const std::string& COM_PORT);

		ErrorCode SendAndWaitForReply(
			const std::vector<uint8_t>& command,
			std::vector<uint8_t>& result);
	};
}
#endif