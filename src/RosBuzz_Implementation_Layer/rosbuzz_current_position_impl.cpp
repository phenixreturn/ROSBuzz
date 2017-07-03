#include "RosBuzz_Implementation_Layer/rosbuzz_current_position_impl.h"

namespace rosbuzz {
	CurrentPositionImpl::CurrentPositionImpl() {
		
	}

	void CurrentPositionImpl::Init(RosController* controller,
			ros::Subscriber subscriber){
		// Name of the implementation
		this->name_ = "CurrentPosition";
		this->controller_ = controller;
		std::string current_position_topic, current_position_type;
		this->controller_->GetNodeHandlePrivate().getParam("topics/gps",
				current_position_topic);
		this->controller_->GetNodeHandlePrivate().getParam("type/gps",
				current_position_type);
		// Different "type" of current position subscriber could be implemented along with associated methods
		if (current_position_type == "sensor_msgs/NavSatFix") {
			subscriber = this->controller_->GetNodeHandleGlobal().subscribe(
					current_position_topic, 1000, &CurrentPositionImpl::SetPosition,
					this);
		}
		else{
			/*Default implementation*/
			subscriber = this->controller_->GetNodeHandleGlobal().subscribe(
					current_position_topic, 1000, &CurrentPositionImpl::SetPosition,
					this);
		}
		this->update_callbacks_= this->controller_->GetUpdateCallbacks();
		//Insert the object within the update map
		this->update_callbacks_->insert(std::pair <std::string, RosCallbackInterface*> (
				this->GetName(),this) );
	}
	//Methods of current position implementation
	// Callback for the subscriber
	void CurrentPositionImpl::SetPosition(
			const sensor_msgs::NavSatFix::ConstPtr& msg) {
		this->current_position_.SetLatitude(msg->latitude);
		this->current_position_.SetLongitude(msg->longitude);
		this->current_position_.SetAltitude(msg->altitude);
	}
	// returns the name of the implementaiton 
	std::string CurrentPositionImpl::GetName() {
		return this->name_;
	}
	Position::Gps CurrentPositionImpl::GetPosition(){
		return this->current_position_;
	}

}
