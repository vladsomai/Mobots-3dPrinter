#ifdef _CRT_SECURE_NO_WARNINGS
#undef _CRT_SECURE_NO_WARNINGS
#endif
#define _CRT_SECURE_NO_WARNINGS 1 /*for fopen warning*/

#include "SerialPort.h"
#include "../Motor/MotorUtils.h"
#include "../LogService/LogService.h"

namespace SerialPortNS
{
	using namespace LogServiceNS;
	using namespace MotorNS;

	std::mutex SerialPort::SerialPortMutex{};

	SerialPort::~SerialPort()
	{
		if (mPort != NULL)
		{
			fclose(mPort);
		}
	}

	ErrorCode SerialPort::Send(const ByteList& command)
	{
		if (!mPort)
			return ErrorCode::INVALID_PORT;

		auto cmdSize = command.size();

		fwrite(command.data(),
			sizeof(uint8_t),
			cmdSize,
			mPort);

		fflush(mPort);

		return ErrorCode::NO_ERR;
	}

	std::unique_ptr<SerialPort>& SerialPort::Instance()
	{
		std::scoped_lock<std::mutex> instanceLock{ SerialPortMutex };

		if (mInstance == nullptr)
		{
			mInstance = std::unique_ptr<SerialPort>(new SerialPort);
		}

		return mInstance;
	}

	ErrorCode SerialPort::Connect(const std::string& COM_PORT)
	{
		std::scoped_lock<std::mutex> instanceLock{ SerialPortMutex };

		if (mPort != NULL)
		{
			/*Close the current port and continue opening a new one */
			fclose(mPort);
		}

#ifdef _WIN32
		std::string WIN_COM_PATH{ "\\\\.\\" };

		WIN_COM_PATH.append(COM_PORT);

		mPort = fopen(WIN_COM_PATH.c_str(), "rb+");

		if (mPort != NULL)
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is now open.");
			return ErrorCode::NO_ERR;
		}
		else 
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT+" is invalid.");
			return ErrorCode::INVALID_PORT;
		}

#else
		//MAC or Linux
		std::string COM_PATH{ "/dev/tty" };

		COM_PATH.append(COM_PORT);

		mPort = fopen(COM_PATH.c_str(), "rb+");

		if (mPort != NULL)
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is now open.");
			return ErrorCode::NO_ERR;
		}
		else {
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is invalid.");
			return ErrorCode::INVALID_PORT;
		}
#endif

	}

	ErrorCode SerialPort::SendAndWaitForReply(
		const ByteList& command,
		ByteList& result)
	{
		std::scoped_lock<std::mutex> instanceLock{ SerialPortMutex };
		
		if (mPort == NULL)
		{
			LogService::Instance()->LogInfo("SendAndWaitForReply -> Invalid port");

			return ErrorCode::INVALID_PORT;
		}

		Send(command);

		if (command[0] != 255)
		{
			uint8_t buffer[255]{ 0 };
			size_t readSize = MotorUtils::GetRcvCommandSize(command[1]);

			fread(buffer,
				sizeof(uint8_t),
				readSize,
				mPort);

			for (size_t i = 0; i < readSize; i++)
			{
				result.push_back(buffer[i]);
			}
		}

		return ErrorCode::NO_ERR;
	}

}