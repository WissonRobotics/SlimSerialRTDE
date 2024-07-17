 
#ifndef __SLIMDRIVE_DEF_H_
#define __SLIMDRIVE_DEF_H_



enum SLIMDRIVE_FUNCODE_t {
	/*PC serial interface */

	FUNC_NONE = 0x00,


	FUNC_RESET = 0x01,
  
	FUNC_SAVE_EEPROM,


	FUNC_WRITE_ADDRESS,

	FUNC_READ_ADDRESS,

	FUNC_WRITE_REGISTER,

	FUNC_READ_REGISTER,

  
	//temp added
	FUNC_WRITE_SEGMENT_STATE_KN600 = 0x13,

	FUNC_WRITE_ARM_XYZRPY = 0x14,


	//direct command
	FUNC_GET_OUTPUT,

	FUNC_START_VALVEPUMP,

	FUNC_STOP_VALVEPUMP,

	FUNC_ZERO_PRESSURE,

	FUNC_ZERO_IMUNIJOINT,

	FUNC_SET_PID,

	FUNC_SET_FEEDBACK,

	FUNC_SET_CONTROLMETHOD,



	//func command
	FUNC_COMMAND_TCP,

	FUNC_COMMAND_JOINT,

	FUNC_COMMAND_PRESSURE,

	FUNC_COMMAND_OPENING,




	//customize command
	
	//KN1000
	FUNC_WRITE_SEGMENT_STATE_KN1000 = 0x63,





	//MONO

	FUNC_CUSTOM_MONO_PC_ACTION = 0xA0,

	FUNC_CUSTOM_MONO_PC_QUERY,

	FUNC_CUSTOM_MONO_PC_RESPONSE,
	

	

	FUNC_CUSTOM_MONO_MOTOR_ACTION = 0xA4,

	FUNC_CUSTOM_MONO_MOTOR_QUERY,

	FUNC_CUSTOM_MONO_MOTOR_RESPONSE,

  
	FUNC_CUSTOM_MONO_PC_ERROR = 0xEE,


};

#endif /* __SLIMDRIVE_DEF_H_ */
