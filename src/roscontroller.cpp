
#include "roscontroller.h"

namespace rosbuzz {

	/*------------ Constructor ---------------*/
	RosController::RosController(ros::NodeHandle& nh_global,
			ros::NodeHandle& nh_private) :
			nh_global_(nh_global), nh_private_(nh_private) {
			ROS_INFO("Buzz_node");
	}
	/*Intializes all the Control and update implementaions*/
	void RosController::Init() {
		this->control_callbacks_ = new std::map<std::string, RosCallbackInterface*>;
		this->update_callbacks_  = new std::map<std::string, RosCallbackInterface*>;
		CurrentPositionImpl *current_position_callback = new CurrentPositionImpl();
		current_position_callback->Init(this, current_position_subscriber_);
		/*Obtain buzz parameters from parameter server*/
		ObtainBuzzParameters();
		// TODO Obtain ROBOT ID from Xbee
		this->robot_id_=1;
		buzz_utility_ = new BuzzUtility(this->control_callbacks_, this->update_callbacks_,
			this->bzzfile_name_,robot_id_);

	}
	/*Spins the whole system one step*/
	void RosController::Loop() {
		/*loop rate of ros*/
		ros::Rate loop_rate(BUZZRATE);
		while (ros::ok()) {
			buzz_utility_->ControlStep();
	    	ros::spinOnce();
			loop_rate.sleep();
		}
	}
	/*Private node handle */
	ros::NodeHandle& RosController::GetNodeHandlePrivate(){
		return this->nh_private_;
	}
	/**/
	ros::NodeHandle& RosController::GetNodeHandleGlobal(){
		return this->nh_global_;
	}
	std::map<std::string, RosCallbackInterface*>* RosController::GetUpdateCallbacks(){
		return this->update_callbacks_;
	}
	std::map<std::string, RosCallbackInterface*>* RosController::GetControlCallbacks(){
		return this->control_callbacks_;
	}
	/*Obtain .bzz file name from parameter server*/
	void RosController::ObtainBuzzParameters(){
	  	if(this->nh_private_.getParam("bzzfile_name", this->bzzfile_name_));
	  	else {ROS_ERROR("Provide a .bzz file to run in Launch file"); system("rosnode kill rosbuzz2_node");}  
		
	}
}
