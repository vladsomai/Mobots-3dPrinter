
#include "../includes.h"

#include "MoveQueue.h"
#include "../LogService/LogService.h"

namespace MoveQueueNS
{
	using namespace LogServiceNS;

	std::vector<uint8_t> MoveQueue::back() const
	{
		std::lock_guard<std::mutex> instanceLock{ MoveQueueMutex };

		if (mMoveQueue.size() > 0)
			return mMoveQueue.back();
		else
			return std::vector<uint8_t>();
	}

	std::vector<uint8_t> MoveQueue::front() const
	{
		std::lock_guard<std::mutex> instanceLock{ MoveQueueMutex };

		if (mMoveQueue.size() > 0)
			return mMoveQueue.front();
		else
			return std::vector<uint8_t>();
	}

	void MoveQueue::pop()
	{
		std::lock_guard<std::mutex> instanceLock{ MoveQueueMutex };

		if (mMoveQueue.size() > 0)
		{
			mMoveQueue.erase(mMoveQueue.begin());
		}
	}

	void MoveQueue::push(const std::vector<uint8_t>& move)
	{
		std::lock_guard<std::mutex> instanceLock{ MoveQueueMutex };


		mMoveQueue.push_back(std::move(move));
	}

	void MoveQueue::insert(const std::vector<std::vector<uint8_t>>& moves)
	{
		std::lock_guard<std::mutex> instanceLock{ MoveQueueMutex };

		mMoveQueue.insert(mMoveQueue.end(), moves.begin(), moves.end());
	}
}