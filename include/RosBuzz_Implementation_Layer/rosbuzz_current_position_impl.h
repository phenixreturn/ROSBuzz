#pragma once

#include <sensor_msgs/NavSatFix.h>
#include "RosBuzz_Abstraction_Layer/rci_current_position.h"
#include "roscontroller.h"
#include "Types/position.h"
#include <stdlib.h>

namespace rosbuzz {
class CurrentPositionImpl : public RciCurrentPosition {
public:
	CurrentPositionImpl();
	void SetPosition(const sensor_msgs::NavSatFix::ConstPtr& msg);
	Position::Gps GetPosition();
	void Init(RosController* controller,
			ros::Subscriber subscriber);
	std::string GetName();
private:
	RosController* controller_;
	std::map<std::string, RosCallbackInterface*> *update_callbacks_;
	Position::Gps current_position_;
	std::string name_;
};

}
