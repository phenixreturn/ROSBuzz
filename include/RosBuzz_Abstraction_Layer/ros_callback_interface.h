#pragma once
namespace rosbuzz {
class RosCallbackInterface;
}

namespace rosbuzz {
class RosCallbackInterface {
public:
	virtual ~RosCallbackInterface(){}
	virtual void Init() {}
	virtual void Execute() {}

};
}
