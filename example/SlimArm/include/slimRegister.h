#pragma once
#include <array>
#include <stdio.h>
#include <string>
#include <vector>

// #ifdef __GNUC__
// #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
// #endif

// #ifdef _MSC_VER
// #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__
// __pragma( pack(pop)) #endif


#define  SlimArm_Type_Default 1
#define  SlimArm_Type_Monos3  2
 
#ifndef SLIM_ARM_TYPE
#define SLIM_ARM_TYPE SlimArm_Type_Default
#endif


#pragma pack(1)
 
#if SLIM_ARM_TYPE == SlimArm_Type_Default
enum MOTOR_CONFIG {
  MOTOR_CONFIG_RElATIVE = 0x00,
  MOTOR_CONFIG_ABSOLUTE,
  MOTOR_CONFIG_VELOCITY,
  MOTOR_CONFIG_ACCELERATION,
};
  
enum PressureArrayIndex{
  IndexPressureInner = 0,
  IndexPressureOuter = 1,
  IndexPressureElongate = 2,
  IndexPressureCable = 3,
  IndexPressureGuide = 4,
  IndexPressureShift = 5,
  IndexPressureGrasp = 6,
  IndexPressureLock = 7
};
 

union AXIS_CHOSEN {
  struct {
		uint16_t fan : 1;
		uint16_t heat : 1;
    uint16_t ph : 1;
    uint16_t rgy : 1;
    uint16_t led : 1;
    uint16_t lock : 1;
    uint16_t grasp : 1;
    uint16_t shift : 1;
    uint16_t guide : 1;
    uint16_t cable : 1;
    uint16_t elongate : 1;
    uint16_t Pitch : 1;
    uint16_t Rchosen : 1;
    uint16_t Zchosen : 1;
    uint16_t Ychosen : 1;
    uint16_t Xchosen : 1;
  };
  uint16_t axisChosenU16;
};

union AXIS_CONFIG {
  struct {

    uint16_t reserved : 12;
    uint16_t chamber : 2;
    uint16_t motor : 2;
  };
  uint16_t axisConfigU16;
};


union PressureUnion{
  std::array<int16_t,8> pressureArray;
  struct {
    int16_t Ppitch_inner;
    int16_t Ppitch_outer;
    int16_t Pelongate;
    int16_t Pcable;
    int16_t Pguide;
    int16_t Pshift;
    int16_t Pgrasp;
    int16_t Plock;
    //			int16_t P8;
    //			int16_t P9;
  };
};

union IOCMD {
  struct {
    uint16_t red : 2;
    uint16_t greed : 2;
    uint16_t yellow : 2;
    uint16_t globalLED : 1;
    uint16_t localLED : 1;
    uint16_t ph : 2;
		uint16_t magnet : 1;
		uint16_t V24 : 1;
    uint16_t V5 :1;
    uint16_t heat : 1;
    uint16_t fan : 1;
    uint16_t reserved : 1; 
    // uint16_t doorconfig:2;
  };
  uint16_t IOU16;
};
union IOFlagsUnion{
  struct {
    uint16_t attach_status_0 : 1;
    uint16_t attach_status_1 : 1;
    uint16_t shift_away_status : 1;
    uint16_t shift_central_status : 1;
    uint16_t gripper_close_status : 1;
    uint16_t gun_status_0 : 1;
    uint16_t gun_status_1 : 1;
    uint16_t reserved : 9;
  };
  uint16_t IOU16;
};
typedef struct FRAME_PC_To_Monosdrive_TAG {
  uint8_t header[2];
	uint8_t src;
	uint8_t payloadBytes;
	uint8_t funcode;

	AXIS_CHOSEN axisChosen;

	AXIS_CONFIG axisConfig;

	int16_t XCmd;
	int16_t YCmd;
	int16_t ZCmd;
	int16_t RCmd;
 
	int16_t PitchCmd;
	int16_t elongateCmd;
	int16_t cableCmd;
	int16_t guideCmd;
	int16_t shiftCmd;
	int16_t graspCmd;
	int16_t lockCmd;

  IOCMD   IOCmd;
  int16_t doorCmd;
  uint16_t crc;

} FRAME_PC_To_Monosdrive;

 
typedef struct FRAME_monosdrive_to_PC_TAG {

  uint8_t header[2];
  uint8_t src;
  uint8_t payloadBytes;
  uint8_t funcode;

  int16_t X;
  int16_t Y;
  int16_t Z;
  int16_t R;
  int16_t Pitch;
  PressureUnion pressures;
  int16_t pSource;
  int16_t pSink;

  IOFlagsUnion IOFlags;
  std::array<int16_t,4> encoders;
  std::array<int16_t,2> lasers;
  std::array<int16_t,3> ultraSonics;
  std::array<uint16_t,8> errorList;

  uint16_t crc;

} FRAME_monosdrive_to_PC;
#elif SLIM_ARM_TYPE==SlimArm_Type_Monos3
enum MOTOR_CONFIG {
  MOTOR_CONFIG_RElATIVE = 0x00,
  MOTOR_CONFIG_ABSOLUTE,
  MOTOR_CONFIG_VELOCITY,
  MOTOR_CONFIG_ACCELERATION,
};
  
enum PressureArrayIndex{
  IndexPressureInnerDown = 0,
  IndexPressureOuterDown = 1,
  IndexPressureInnerUp = 2,
  IndexPressureOuterUp = 3,
  IndexPressureElongate = 4,
  IndexPressureGuide = 5,
  IndexPressureShift = 6,
  IndexPressureGrasp = 7,
  IndexPressureLock = 8
};
 

union AXIS_CHOSEN {
  struct {
		uint16_t reserved : 3;
    uint16_t ph : 1;
    uint16_t rgy : 1;
    uint16_t led : 1;
    uint16_t lock : 1;
    uint16_t grasp : 1;
    uint16_t shift : 1;
    uint16_t guide : 1;
    uint16_t elongate : 1;
    uint16_t Pitch : 1;
    uint16_t Rchosen : 1;
    uint16_t Zchosen : 1;
    uint16_t Ychosen : 1;
    uint16_t Xchosen : 1;
  };
  uint16_t axisChosenU16;
};

union AXIS_CONFIG {
  struct {

    uint16_t reserved : 12;
    uint16_t chamber : 2;
    uint16_t motor : 2;
  };
  uint16_t axisConfigU16;
};


union PressureUnion{
  std::array<int16_t,9> pressureArray;
  struct {
    int16_t Ppitch_inner_Down;
    int16_t Ppitch_outer_Down;
    int16_t Ppitch_inner_Up;
    int16_t Ppitch_outer_Up;
    int16_t Pelongate;
    int16_t Pguide;
    int16_t Pshift;
    int16_t Pgrasp;
    int16_t Plock;
  };
};
 
union IOCMD {
  struct {
    uint16_t red : 2;
    uint16_t greed : 2;
    uint16_t yellow : 2;
    uint16_t globalLED : 1;
    uint16_t localLED : 1;
    uint16_t ph : 2;
		uint16_t magnet : 1;
		uint16_t reserved : 5;
    // uint16_t doorconfig:2;
  };
  uint16_t IOU16;
};
 

union IOFlagsUnion{
  struct {
    uint16_t attach_status_0 : 1;
    uint16_t attach_status_1 : 1;
    uint16_t shift_away_status : 1;
    uint16_t shift_central_status : 1;
    uint16_t gripper_close_status : 1;
    uint16_t gun_status_0 : 1;
    uint16_t gun_status_1 : 1;
    uint16_t fastener     :1;
    uint16_t reserved : 8;
  };
  uint16_t IOU16;
};
typedef struct FRAME_PC_To_Monosdrive_TAG {
  uint8_t header[2];
	uint8_t src;
	uint8_t payloadBytes;
	uint8_t funcode;

	AXIS_CHOSEN axisChosen;

	AXIS_CONFIG axisConfig;

	int16_t XCmd;
	int16_t YCmd;
	int16_t ZCmd;
	int16_t RCmd;
 
	int16_t PitchCmd;
	int16_t elongateCmd; 
	int16_t guideCmd;
	int16_t shiftCmd;
	int16_t graspCmd;
	int16_t lockCmd;

  IOCMD   IOCmd;
  int16_t doorCmd;
  uint16_t crc;

} FRAME_PC_To_Monosdrive;

 
typedef struct FRAME_monosdrive_to_PC_TAG {

  uint8_t header[2];
  uint8_t src;
  uint8_t payloadBytes;
  uint8_t funcode;

  int16_t X;
  int16_t Y;
  int16_t Z;
  int16_t R;
  int16_t Pitch;
  PressureUnion pressures;
  int16_t pSource;
  int16_t pSink;

  IOFlagsUnion IOFlags;
  std::array<int16_t,4> encoders;
  std::array<int16_t,2> lasers;
  std::array<int16_t,3> ultraSonics;
  std::array<uint16_t,8> errorList;

  uint16_t crc;

} FRAME_monosdrive_to_PC;

#endif

enum SLIMSERIAL_FUNCODE_t {
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

#pragma pack() 
