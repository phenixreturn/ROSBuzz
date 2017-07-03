#include "Buzz_Abstraction_Layer/buzz_vm_interface.h"

namespace rosbuzz{
	int BuzzRosPrint(buzzvm_t vm) {
	int i;
	char buffer [50] = "";
        sprintf(buffer,"%s [%i]", buffer, (int)vm->robot);
	   for(i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
	      buzzvm_lload(vm, i);
	      buzzobj_t o = buzzvm_stack_at(vm, 1);
	      buzzvm_pop(vm);
	      switch(o->o.type) {
		 case BUZZTYPE_NIL:
		    sprintf(buffer,"%s BUZZ - [nil]", buffer);
		    break;
		 case BUZZTYPE_INT:
		    sprintf(buffer,"%s %d", buffer, o->i.value);
		    //fprintf(stdout, "%d", o->i.value);
		    break;
		 case BUZZTYPE_FLOAT:
		    sprintf(buffer,"%s %f", buffer, o->f.value);
		    break;
		 case BUZZTYPE_TABLE:
		    sprintf(buffer,"%s [table with %d elems]", buffer, (buzzdict_size(o->t.value)));
		    break;
		 case BUZZTYPE_CLOSURE:
		    if(o->c.value.isnative)
		    	sprintf(buffer,"%s [n-closure @%d]", buffer, o->c.value.ref);
		    else
		    	sprintf(buffer,"%s [c-closure @%d]", buffer, o->c.value.ref);
		    break;
		 case BUZZTYPE_STRING:
		    sprintf(buffer,"%s %s", buffer, o->s.value.str);
		    break;
		 case BUZZTYPE_USERDATA:
		    sprintf(buffer,"%s [userdata @%p]", buffer, o->u.value);
		    break;
		 default:
		    break;
	      }
	   }
	   ROS_INFO(buffer);
	   //fprintf(stdout, "\n");
	   return buzzvm_ret0(vm);
	}	


}