#include "Buzz_Abstraction_Layer/buzz_utility.h"

namespace rosbuzz{
	
	BuzzUtility::BuzzUtility(std::map<std::string, RosCallbackInterface*> *control_callbacks,
			std::map<std::string, RosCallbackInterface*> *update_callbacks,
			std::string bzzfile_name, int robot_id){
		this->control_callbacks_=control_callbacks;
		this->update_callbacks_=update_callbacks;
		this->m_bzzfile_name_=bzzfile_name;
		std::string base_name = CompileBzz(bzzfile_name);
		this->m_bofile_name_ = base_name + ".bo";
		this->m_dbgfile_name_= base_name + ".bdb";
		BuzzScriptSet(this->m_bofile_name_.c_str(),
	                    this->m_dbgfile_name_.c_str(),robot_id);
		this->update_closure_= new BuzzUpdateClosures();
	}

	void BuzzUtility::ControlStep() {
		/*Process input messages*/
		ProcessInMessage();
		/*Update sensors*/
		UpdateSensors();
		/* Call Buzz step() function */
		if(buzzvm_function_call(this->m_vm_, "step", 0) != BUZZVM_STATE_READY) {
		fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
		      this->m_bofile_name_,
		      BuzzErrorInfo());
		buzzvm_dump(this->m_vm_);
		// TODO : Obtain the name of the node from parameter server
		system("rosnode kill rosbuzz2_node");
		}
		/*Process Out messages*/
		ProcessOutMessage();
	}

	/*--------------------------------------------------------
	/ Create Buzz bytecode from the bzz script inputed
	/-------------------------------------------------------*/
	std::string BuzzUtility::CompileBzz(std::string bzzfile_name){
		std::stringstream bzzfile_in_compile;
	    std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
		std::string  name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
 		name = name.substr(0,name.find_last_of("."));
		bzzfile_in_compile << "bzzc -I " << path << "include/"; //<<" "<<path<< name<<".basm";
		bzzfile_in_compile << " -b " << path << name << ".bo";
		bzzfile_in_compile << " -d " << path << name << ".bdb ";
		bzzfile_in_compile << bzzfile_name;
		ROS_WARN("Launching buzz compilation: %s", bzzfile_in_compile.str().c_str());
   		system(bzzfile_in_compile.str().c_str());
		return path + name;
	}

	/****************************************/
	/*Buzz hooks that can be used inside .bzz file*/
	/****************************************/
	int BuzzUtility::BuzzRegisterHooks() {
		buzzvm_pushs(this->m_vm_,  buzzvm_string_register(this->m_vm_, "print", 1));
   		buzzvm_pushcc(this->m_vm_, buzzvm_function_register(this->m_vm_, BuzzRosPrint));
   		buzzvm_gstore(this->m_vm_);
   		buzzvm_pushs(this->m_vm_,  buzzvm_string_register(this->m_vm_, "log", 1));
        buzzvm_pushcc(this->m_vm_, buzzvm_function_register(this->m_vm_, BuzzRosPrint));
        buzzvm_gstore(this->m_vm_);
        return this->m_vm_->state;
    }

    /****************************************/
	/* Intialize BuzzVM */
	/****************************************/
	int BuzzUtility::BuzzScriptSet(const char* bo_filename,
	                    const char* bdbg_filename, int robot_id) {
	   	ROS_INFO(" Robot ID: " , robot_id);
	   	/* Reset the Buzz VM */
	   	if(this->m_vm_) buzzvm_destroy(&this->m_vm_);
	   	this->m_vm_ = buzzvm_new((int)robot_id);
	   	/* Get rid of debug info */
	   	if(this->m_dbeg_info_) buzzdebug_destroy(&this->m_dbeg_info_);
	   	this->m_dbeg_info_ = buzzdebug_new();
	   	/* Read bytecode and fill in data structure */
	   	FILE* fd = fopen(bo_filename, "rb");
	   	if(!fd) {
	      		perror(bo_filename);
      		return 0;
	   	}
	   	fseek(fd, 0, SEEK_END);
	   	size_t bcode_size = ftell(fd);
	   	rewind(fd);
	   	this->m_bo_buf_ = (uint8_t*)malloc(bcode_size);
	   	if(fread(this->m_bo_buf_, 1, bcode_size, fd) < bcode_size) {
	      		perror(bo_filename);
	      		buzzvm_destroy(&(this->m_vm_));
      			buzzdebug_destroy(&(this->m_dbeg_info_));
	      		fclose(fd);
      		return 0;
	   	}
	   	fclose(fd);
	   	/* Read debug information */
	   	if(!buzzdebug_fromfile(this->m_dbeg_info_, bdbg_filename)) {
      		buzzvm_destroy(&(this->m_vm_));
  		 	buzzdebug_destroy(&(this->m_dbeg_info_));
      		perror(bdbg_filename);
      		return 0;
   		}
	   	/* Set byte code */
	   	if(buzzvm_set_bcode(this->m_vm_, this->m_bo_buf_, bcode_size) != BUZZVM_STATE_READY) {
	  		buzzvm_destroy(& (this->m_vm_));
	  		buzzdebug_destroy(& (this->m_dbeg_info_));
	  		ROS_ERROR("%s: Error loading Buzz script", bo_filename);
	  		return 0;
	   	}
	   	/* Register hook functions */
	   	if(BuzzRegisterHooks() != BUZZVM_STATE_READY) {
	  		buzzvm_destroy(&(this->m_vm_));
	  		buzzdebug_destroy(&(this->m_dbeg_info_));
	  		ROS_ERROR("[%i] Error registering hooks",robot_id);
			return 0;
	   	}
	   	/* TODO : David will refactor this Create vstig tables */
		if(CreateVstigTable() != BUZZVM_STATE_READY) {
	  		buzzvm_destroy(&this->m_vm_);
	  		buzzdebug_destroy(&(this->m_dbeg_info_));
	  		ROS_ERROR("[%i] Error creating stigmergy tables", robot_id);
			std::cout << "ERROR!!!!   ----------  " << this->m_vm_->errormsg << std::endl;
			std::cout << "ERROR!!!!   ----------  " << buzzvm_strerror(this->m_vm_) << std::endl;
			return 0;
	   	}
	   	// Execute the global part of the script
	   	if(buzzvm_execute_script(this->m_vm_)!= BUZZVM_STATE_DONE){
			ROS_ERROR("Error executing global part, VM state : %i",this->m_vm_->state);
			return 0;
		}
	   	// Call the Init() function
	   	if(buzzvm_function_call(this->m_vm_, "init", 0) != BUZZVM_STATE_READY){
			ROS_ERROR("Error in  calling init, VM state : %i", this->m_vm_->state);
			return 0;
		}
   		/* All OK */
    	ROS_INFO("[%i] INIT DONE!!!", robot_id);
   		return 1;//buzz_update_set(BO_BUF, bdbg_filename, bcode_size);
	}

	/*Fetches the Error info from buzz*/
    const char* BuzzUtility::BuzzErrorInfo() {
		buzzdebug_entry_t dbg = *buzzdebug_info_get_fromoffset(this->m_dbeg_info_, &this->m_vm_->pc);
   		char* msg;
   		if(dbg != NULL) {
      			asprintf(&msg,
               		"%s: execution terminated abnormally at %s:%" PRIu64 ":%" PRIu64 " : %s\n\n",
               		this->m_bofile_name_,
               		dbg->fname,
               		dbg->line,
               		dbg->col,
               		this->m_vm_->errormsg);
   		}
   		else {
      			asprintf(&msg,
               			"%s: execution terminated abnormally at bytecode offset %d: %s\n\n",
               			 this->m_bofile_name_,
              			 this->m_vm_->pc,
      			         this->m_vm_->errormsg);
   		}
 		return msg;
	}
	/*Create user VStig table*/
	int BuzzUtility::CreateVstigTable() {
		buzzobj_t t = buzzheap_newobj(this->m_vm_->heap, BUZZTYPE_TABLE);
		buzzvm_pushs(this->m_vm_, buzzvm_string_register(this->m_vm_, "users", 1));
		buzzvm_push(this->m_vm_,t);
		buzzvm_gstore(this->m_vm_);
		buzzvm_pushs(this->m_vm_, buzzvm_string_register(this->m_vm_, "users", 1));
		buzzvm_gload(this->m_vm_);
		buzzvm_pushs(this->m_vm_, buzzvm_string_register(this->m_vm_, "dataG", 1));
		buzzvm_pusht(this->m_vm_);
		buzzobj_t data = buzzvm_stack_at(this->m_vm_, 1);
		buzzvm_tput(this->m_vm_);
		buzzvm_push(this->m_vm_, data);
		buzzvm_pushs(this->m_vm_, buzzvm_string_register(this->m_vm_, "users", 1));
		buzzvm_gload(this->m_vm_);
		buzzvm_pushs(this->m_vm_, buzzvm_string_register(this->m_vm_, "dataL", 1));
		buzzvm_pusht(this->m_vm_);
		data = buzzvm_stack_at(this->m_vm_, 1);
		buzzvm_tput(this->m_vm_);
		buzzvm_push(this->m_vm_, data);
   		return this->m_vm_->state;
	}
	/*TODO Process In messages*/
	void BuzzUtility::ProcessInMessage(){

	}
	/*TODO Process Out messages*/
	void BuzzUtility::ProcessOutMessage(){

	}
	/*Update Sensor readings*/
	void BuzzUtility::UpdateSensors(){
		std::map<std::string, RosCallbackInterface*>::iterator it = this->update_callbacks_->begin();
		/*TODO Implement rest of the sensors*/
		for(; it != this->update_callbacks_->end(); ++it){
			if(it->first == "CurrentPosition"){
				CurrentPositionImpl* current_postion_impl =	dynamic_cast<CurrentPositionImpl*> (it->second);
				this->update_closure_->UpdateCurrentPostion(this->m_vm_, current_postion_impl->GetPosition());
			}
		}

	}
}