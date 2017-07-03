/** @file      rosbuzz.cpp
 *  @version   2.0 
 *  @date      27.06.2017
 *  @brief     Buzz as a node in ROS. 
 *  @author    Vivek Shankar Varadharajan
 *  @copyright 2017 MistLab. All rights reserved.
 */

#include "roscontroller.h"

/**
 * This program implements Buzz node in ros using mavros_msgs.
 */

int main(int argc, char **argv) {
	/*Initialize rosBuzz node*/
	ros::init(argc, argv, "rosBuzz");
	ros::NodeHandle nh_private("~");
	ros::NodeHandle nh_global;
	rosbuzz::RosController roscontroller(nh_global, nh_private);
	roscontroller.Init();
	/*Register ros controller object inside buzz*/
	//buzzuav_closures::set_ros_controller_ptr(&RosController);
	roscontroller.Loop();
	return 0;
}

