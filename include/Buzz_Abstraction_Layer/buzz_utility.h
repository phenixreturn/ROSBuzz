#pragma once
namespace rosbuzz {
	class BuzzUtility;
}
#define MSG_SIZE 250;
#include <ros/ros.h>
#include <buzz/buzzvm.h>
#include <buzz/buzzdebug.h>
#include <stdint.h>
#include "RosBuzz_Abstraction_Layer/ros_callback_interface.h"
#include "Buzz_Abstraction_Layer/buzz_vm_interface.h"
#include "Buzz_Abstraction_Layer/buzz_update_closure.h"
#include "RosBuzz_Implementation_Layer/rosbuzz_current_position_impl.h"
#include <map>
namespace rosbuzz {
	class BuzzUtility
	{
	public:
		BuzzUtility(std::map<std::string, RosCallbackInterface*> *control_callbacks,
			std::map<std::string, RosCallbackInterface*> *update_callbacks,
			std::string bzzfile_name, int robot_id);
		~BuzzUtility();
		void ControlStep();
	private:
		//BuzzControlClosures* controlClosures;
		//BuzzUpdateClosures* updateClosures;
		//std::string current_command;
		std::map<std::string, RosCallbackInterface*> *control_callbacks_;
		std::map<std::string, RosCallbackInterface*> *update_callbacks_;
		BuzzUpdateClosures *update_closure_;
		std::string  m_bzzfile_name_;  
		std::string  m_bofile_name_;
		std::string	 m_dbgfile_name_;
		buzzvm_t     m_vm_        = 0;
		uint8_t*	 m_bo_buf_	  = 0;
		buzzdebug_t  m_dbeg_info_ = 0;
		int 		 m_robot_id_  = 0;

		/*compiles buzz script from the specified .bzz file*/
		std::string CompileBzz(std::string bzzfile_name);
		void ProcessInMessage();
		void ProcessOutMessage();
		void UpdateSensors();
		int BuzzRegisterHooks();
		int BuzzScriptSet(const char* bo_filename,
	                    const char* bdbg_filename, int robot_id);
		const char* BuzzErrorInfo();
		int CreateVstigTable();
	};
}
