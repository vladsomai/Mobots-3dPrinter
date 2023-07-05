#ifndef MOVEQUEUE
#define MOVEQUEUE
#include "../includes.h"

namespace MoveQueueNS
{
	class MoveQueue
	{
	private:
		std::vector<std::vector<uint8_t>> mMoveQueue{};
		mutable std::mutex MoveQueueMutex;

	public:
		MoveQueue() = default;
		~MoveQueue() = default;

		/*Get a copy of the last item*/
		std::vector<uint8_t> back() const;

		/*Get a copy of the first item*/
		std::vector<uint8_t> front() const;

		/*Remove from front*/
		void pop();

		/*Add to back*/
		void push(const std::vector<uint8_t>& move);

		/*Insert multiple items starting the end*/
		void insert(const std::vector<std::vector<uint8_t>>& moves);
	};
}

#endif