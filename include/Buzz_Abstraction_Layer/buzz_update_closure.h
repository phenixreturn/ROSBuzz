#pragma once
namespace rosbuzz{
	class BuzzUpdateClosures;
}
#include "Types/position.h"
#include <buzz/buzzvm.h>
namespace rosbuzz{
	class BuzzUpdateClosures{
	public:
		BuzzUpdateClosures();
		~BuzzUpdateClosures();
		int UpdateCurrentPostion(buzzvm_t m_vm, Position::Gps current_postion);
	

	};
}