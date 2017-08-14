#pragma once

#include "RosBuzz_Abstraction_Layer/ros_callback_interface.h"
namespace rosbuzz {
class RciCurrentPosition : public RosCallbackInterface {
public:
	virtual ~RciCurrentPosition(){}
	virtual void SetData() {}
};

}
