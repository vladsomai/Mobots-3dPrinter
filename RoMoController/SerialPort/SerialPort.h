#ifndef SERIAL_PORT
#define SERIAL_PORT

#include "../includes.h"

namespace SerialPortNS
{
	using SerialPortHandle = FILE*;

	/* Implements the serial port communication using C standard lib 
	 * fopen
	 * fclose
	 * fwrite
	 * fread
	 * The desirable solution would be to implement the communication based on the OS.
	 */
	class SerialPort
	{

	private:

		SerialPortHandle mPort{ NULL };

		static std::unique_ptr<SerialPort> inline mInstance{ nullptr };

		ErrorCode Send(const ByteList& command);

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
			const ByteList& command,
			ByteList& result);
	};
}
#endif