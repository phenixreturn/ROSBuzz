#include "Buzz_Abstraction_Layer/buzz_update_closure.h"

namespace rosbuzz{
	BuzzUpdateClosures::BuzzUpdateClosures(){

	
	}

	int BuzzUpdateClosures::UpdateCurrentPostion(buzzvm_t m_vm, Position::Gps current_postion){
		buzzvm_pushs(m_vm, buzzvm_string_register(m_vm, "position", 1));
		buzzvm_pusht(m_vm);
		buzzvm_dup(m_vm);
		buzzvm_pushs(m_vm, buzzvm_string_register(m_vm, "latitude", 1));
		buzzvm_pushf(m_vm, current_postion.GetLatitude());
		buzzvm_tput(m_vm);
		buzzvm_dup(m_vm);
		buzzvm_pushs(m_vm, buzzvm_string_register(m_vm, "longitude", 1));
		buzzvm_pushf(m_vm, current_postion.GetLongitude());
		buzzvm_tput(m_vm);
		buzzvm_dup(m_vm);
		buzzvm_pushs(m_vm, buzzvm_string_register(m_vm, "altitude", 1));
		buzzvm_pushf(m_vm, current_postion.GetAltitude());
		buzzvm_tput(m_vm);
		buzzvm_gstore(m_vm);
		return m_vm->state;
	}
}