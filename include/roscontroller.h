#pragma once
#define BUZZRATE 10
namespace rosbuzz{
	class RosController;
}
#include <ros/ros.h>
#include <map>
#include <string.h>
#include "RosBuzz_Abstraction_Layer/ros_callback_interface.h"
#include "RosBuzz_Implementation_Layer/rosbuzz_current_position_impl.h"
#include "Buzz_Abstraction_Layer/buzz_utility.h"

namespace rosbuzz {

	class RosController {
	public:
		RosController(ros::NodeHandle& nh_global, ros::NodeHandle& nh_private);
		void Init();
		void Loop();
		ros::NodeHandle& GetNodeHandlePrivate();
		ros::NodeHandle& GetNodeHandleGlobal();
		std::map<std::string, RosCallbackInterface*> *GetUpdateCallbacks();
		std::map<std::string, RosCallbackInterface*> *GetControlCallbacks();
	private:
		std::map<std::string, RosCallbackInterface*> *control_callbacks_;
		std::map<std::string, RosCallbackInterface*> *update_callbacks_;
		BuzzUtility* buzz_utility_;
		ros::NodeHandle& nh_global_;
		ros::NodeHandle& nh_private_;
		ros::Subscriber current_position_subscriber_;
		std::string bzzfile_name_;
		int robot_id_;
		void ObtainBuzzParameters();
	};

}
