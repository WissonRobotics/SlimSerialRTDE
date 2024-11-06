/**
 * Copyright (c) 2023, WissonRobotics
 * File: SlimArm.cpp
 * Author: Zhiwei Huang (huangzhiwei@wissonrobotics.com)
 * Version 1.0
 * Date: 2023-12-06
 * Brief:
 */
#include "SlimArm.h"
#include <spdlog/fmt/fmt.h>

const std::string AxisNames[6] = {"X","Y","Z","Yaw","Pitch","Roll"}; 

SlimArm::SlimArm():m_actionSingleCommand(false){

  LOG_F(INFO, "[SlimArm] Created");

  //init data
  // arm_port_ = CONFIG_ARM_PARAM.arm_port;
  // baudrate_ = CONFIG_ARM_PARAM.arm_baudrate;
  name_ = "SlimArm";
  if (CONFIG_ARM_PARAM.yaw_zero_offset_enable) {
    m_yaw_zero_offset_ = CONFIG_ARM_PARAM.yaw_zero_offset;
  } else {
    m_yaw_zero_offset_ = 0;
  }
  rtde_client_.setYawOffset(m_yaw_zero_offset_);
  LOG_F(INFO, "[SlimArm] setYawOffset %f ", m_yaw_zero_offset_);
  link_H_ = CONFIG_ARM_PARAM.linkH;
  if(link_H_.size()==0){//maxwell, no link_H.  to avoid error, manually asignn one;
    link_H_ = {0.01154, 0.42953, 0.055, 0.0595, 0.4225}; 
  }
  link_L_ = LinkH2L(0);


  memset((uint8_t *)(&m_arm_data_),0,sizeof(m_arm_data_));
  m_arm_data_.T_current_ = ForwardKinematics(m_arm_data_.q_current_);
  m_arm_data_.T_desired_ = ForwardKinematics(m_arm_data_.q_desired_);
 
  //register callback
  rtde_client_.setUpdateCallback([this](){update();});

#if SLIM_ARM_TYPE == SlimArm_Type_Default
  m_pressure_steady_deadzone.Ppitch_inner = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Ppitch_outer = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pelongate = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pguide = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pshift = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pgrasp = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Plock = 100; // 50HPa deadzone 

  m_pitch_deadzone_0degree=0.0174;
  m_pitch_deadzone=0.082;
  m_pitch_deadzone_default=0.082;
  m_pitch_within_count=0;
  m_pitch_within_count_max=20;

#elif SLIM_ARM_TYPE == SlimArm_Type_Monos3
  m_pressure_steady_deadzone.Ppitch_inner_Down = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Ppitch_outer_Down = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Ppitch_inner_Up = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Ppitch_outer_Up = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pelongate = 100; // 50HPa deadzone
  m_pressure_steady_deadzone.Pguide = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pshift = 150; // 50HPa deadzone
  m_pressure_steady_deadzone.Pgrasp = 200; // 50HPa deadzone
  m_pressure_steady_deadzone.Plock = 150; // 50HPa deadzone
  m_pitch_deadzone_0degree=0.02;
  m_pitch_deadzone=0.004;
  m_pitch_deadzone_default=0.004;
  m_pitch_within_count=0;
  m_pitch_within_count_max=50;
#endif
 
}

SlimArm::~SlimArm(){

}

WS_STATUS SlimArm::Connect() {
  return Connect(arm_port_, baudrate_);
}
 

WS_STATUS SlimArm::Connect(std::string portname,uint32_t baudrate) {
  if(m_using_simulator){

    arm_port_ = "/tmp/ttyV1";
    baudrate_ = baudrate;
    m_arm_simulator.connect("/tmp/ttyV0",baudrate_);
  }
  else{
    arm_port_ = portname;
    baudrate_ = baudrate;
  }

  if (rtde_client_.connect(arm_port_, baudrate_)==WS_OK) {
  LOG_F(INFO, "[SlimArm] connected to port %s at baudrate %d", arm_port_.c_str(), baudrate_);
    m_flagDataUpdated = false;
   
    int tt=0;
    while(!isAlive()){
      if(tt++>2){
        LOG_F(WARNING,"[SlimArm]  port %s  is not responding",arm_port_.c_str());
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    

    if(m_flagDataUpdated){
      m_arm_data_.q_desired_ = m_arm_data_.q_current_;
      m_arm_data_.T_desired_ = m_arm_data_.T_current_;
      LOG_F(1,"Arm data updated");
    }
    else{
      LOG_F(WARNING,"Timeout to update arm data");
    }
  
    SetSpeed(CONFIG_ARM_PARAM.speed_level);

#if SLIM_ARM_TYPE == SlimArm_Type_Default
    setDefaultPitchDeadzone(0.082);
#elif SLIM_ARM_TYPE == SlimArm_Type_Monos3
    setDefaultPitchDeadzone(0.004);
#endif
    setDefaultPitchDeltaDeadzone(0.001);

    // this->SendLightSignalCommand(LightSignalCommand::LightYellowOn);

    return WS_STATUS::WS_OK;
  } else {
    LOG_F(ERROR, "[SlimArm] fail to connect to port %s at baudrate %d", arm_port_.c_str(), baudrate_);
    return WS_STATUS::WS_FAIL;
  }
}

void SlimArm::Disconnect() {
  if (IsConnected()) {
    rtde_client_.disconnect();
  }
}

void SlimArm::enableActions() {
  m_enable_actions = true;
  m_actionMoveJoint.enable();
  m_actionMovePitch.enable();
  m_actionSingleCommand.enable();
  for(auto & action:m_actionMoveChamber){
    action.enable();
  }
}


void SlimArm::disableActions() {
  m_enable_actions = false;
  m_actionMoveJoint.disable();
  m_actionMovePitch.disable();
  m_actionSingleCommand.disable();
  for(auto & action:m_actionMoveChamber){
    action.disable();
  }
}


 



WS_STATUS SlimArm::ValveOn(double timeout,bool async){
    if(async){
      m_actionSingleCommand.actionThread=std::make_unique<std::jthread>([&,timeout](){
        ValveOn(timeout,false);
      });
      return WS_OK;
    }

    return m_actionSingleCommand.act( "ValveOn",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition.
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&SlimArmRTDE::commandStart, &rtde_client_)
                        );

}
 
WS_STATUS SlimArm::ValveOff(double timeout,bool async) {
    if(async){
      m_actionSingleCommand.actionThread=std::make_unique<std::jthread>([&,timeout](){
        ValveOff(timeout,false);
      });
      return WS_OK;
    }

  return m_actionSingleCommand.act( "ValveOff",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&SlimArmRTDE::commandStop, &rtde_client_)
                        );
}

 

WS_STATUS SlimArm::HeatOn(double timeout){
   int16_t command_value = 1;
   return m_actionSingleCommand.act( "HeatOn",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&SlimArmRTDE::commandHeat, &rtde_client_, std::placeholders::_1),
                        command_value
                        );
}
 
WS_STATUS SlimArm::HeatOff(double timeout) {
   int16_t command_value = 0;
   return m_actionSingleCommand.act( "HeatOff",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&SlimArmRTDE::commandHeat, &rtde_client_, std::placeholders::_1),
                        command_value
                        );
}
WS_STATUS SlimArm::FanOn(double timeout){
   int16_t command_value = 1;
   return m_actionSingleCommand.act( "FanOn",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&SlimArmRTDE::commandFan, &rtde_client_, std::placeholders::_1),
                        command_value
                        );
}
 
WS_STATUS SlimArm::FanOff(double timeout) {
   int16_t command_value = 0;
   return m_actionSingleCommand.act( "FanOff",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&SlimArmRTDE::commandFan, &rtde_client_, std::placeholders::_1),
                        command_value
                        );
}
 
 
WS_STATUS SlimArm::SendLightSignalCommand(LightSignalCommand signal_command,double timeout) {
   return m_actionSingleCommand.act(rtde_client_.getSignalLightName(signal_command),
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&SlimArmRTDE::commandSignalLight, &rtde_client_, std::placeholders::_1),
                        signal_command
                        );
}

WS_STATUS SlimArm::LedLightControl(std::string light_name, int light_command,double timeout){
  std::string actionName = "Light_" + light_name + (light_command==0?"_Off":"_On");
   return m_actionSingleCommand.act(actionName,
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&SlimArmRTDE::commandLight, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                        light_name,
                        light_command
                        );  
}
 

bool SlimArm::VerifyPoseIsWithin(std::array<float,6> poses) {
  // TODO
  return true;
}
bool SlimArm::VerifyJointIsWithin(std::array<float,6> joints) {
  if (joints[0] > CONFIG_ARM_PARAM.joint_upper_limit[0] ||
      joints[0] < CONFIG_ARM_PARAM.joint_lower_limit[0] ||
      joints[1] < CONFIG_ARM_PARAM.joint_lower_limit[1] ||
      joints[1] > CONFIG_ARM_PARAM.joint_upper_limit[1] ||
      joints[2] > CONFIG_ARM_PARAM.joint_upper_limit[2] ||
      joints[2] < CONFIG_ARM_PARAM.joint_lower_limit[2] ||
      joints[3] > CONFIG_ARM_PARAM.joint_upper_limit[3] ||
      joints[3] < CONFIG_ARM_PARAM.joint_lower_limit[3] ||
      joints[4] > CONFIG_ARM_PARAM.joint_upper_limit[4] ||
      joints[4] < CONFIG_ARM_PARAM.joint_lower_limit[4])
      return false;
  return true;
}
 




WS_STATUS SlimArm::GunElongateByIOFlags(int pressured,bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&,pressured](){
      GunElongateByIOFlags(pressured,false);
    });
    return WS_OK;
  }
  std::array<uint8_t,2> ioIndexes={0,1};
  std::array<uint8_t,2> ioTargetStates{1,1};
  return chamberMoveAtIOFlags("GunElongate",IndexPressureElongate,pressured,ioIndexes,ioTargetStates,false,CONFIG_TASK_PARAM.attach_gun_insert_timeout);
}
 
//Action Elongate
WS_STATUS SlimArm::GunElongate(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&](){
      GunElongate(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPressure("GunElongate",IndexPressureElongate,CONFIG_ARM_PARAM.gun_elongate_value,CONFIG_TASK_PARAM.attach_gun_insert_timeout);
}
WS_STATUS SlimArm::GunContract(bool async){
    if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&](){
      GunContract(false);
    });
    return WS_OK;
  }
 return chamberMoveAtPressure("GunContract",IndexPressureElongate,CONFIG_ARM_PARAM.gun_contract_value,CONFIG_TASK_PARAM.detach_gun_contract_timeout);
}

#if SLIM_ARM_TYPE == SlimArm_Type_Monos3
WS_STATUS SlimArm::GunElongateByLaser(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&](){
      GunElongateByLaser(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GunElongate",IndexPressureElongate,CONFIG_ARM_PARAM.gun_elongate_value,1,CONFIG_GUN_PARAM.attached_well_distance,true,CONFIG_TASK_PARAM.detach_return_plug_timeout);
  return chamberMoveAtPosition("GunElongate",IndexPressureElongate,CONFIG_ARM_PARAM.gun_elongate_value,1,CONFIG_GUN_PARAM.attached_well_distance,true,CONFIG_TASK_PARAM.detach_return_plug_timeout);
}
#endif
WS_STATUS SlimArm::GunContractByLaser(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&](){
      GunContractByLaser(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GunContract",IndexPressureElongate,CONFIG_ARM_PARAM.gun_contract_value,1,CONFIG_GUN_PARAM.detached_well_distance,false,CONFIG_TASK_PARAM.detach_gun_contract_timeout);
}
WS_STATUS SlimArm::GunElongateByPressure(int pressurecmd,bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&,pressurecmd](){
      GunElongateByPressure(pressurecmd,false);
    });
    return WS_OK;
  }
  return chamberMoveAtPressure("GunElongateByPressure",IndexPressureElongate,pressurecmd,CONFIG_TASK_PARAM.attach_gun_insert_timeout);
}
#if SLIM_ARM_TYPE == SlimArm_Type_Monos3
WS_STATUS SlimArm::GunElongateByLaser(int distanceCmd,bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&,distanceCmd](){
      GunElongateByLaser(distanceCmd,false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GunElongate",IndexPressureElongate,CONFIG_ARM_PARAM.gun_elongate_value,1,distanceCmd,true,CONFIG_TASK_PARAM.detach_return_plug_timeout);
}
#endif
WS_STATUS SlimArm::GunContractByLaser(int distanceCmd,bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&,distanceCmd](){
      GunContractByLaser(distanceCmd,false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GunContract",IndexPressureElongate,CONFIG_ARM_PARAM.gun_contract_value,1,distanceCmd,false,CONFIG_TASK_PARAM.detach_gun_contract_timeout);
}
#if SLIM_ARM_TYPE == SlimArm_Type_Monos3
WS_STATUS SlimArm::GunElongateByPositionPrecisely(int distanceCmd,bool async)
{
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&,distanceCmd](){
      GunElongateByPositionPrecisely(distanceCmd,false);
    });
    return WS_OK;
  }
  return chamberMoveAtPositionPrecisely("GripperElongate",IndexPressureElongate,CONFIG_ARM_PARAM.gun_elongate_value,1,distanceCmd,CONFIG_TASK_PARAM.detach_return_plug_timeout);
}
#endif
WS_STATUS SlimArm::GunZeroPressure(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureElongate].actionThread=std::make_unique<std::jthread>([&](){
      GunZeroPressure(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPressure("GunZeroPressure",IndexPressureElongate,0,CONFIG_TASK_PARAM.attach_gun_insert_timeout);
}


//Action guide
WS_STATUS SlimArm::GuideElongate(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGuide].actionThread=std::make_unique<std::jthread>([&](){
      GuideElongate(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GuideElongate",IndexPressureGuide,CONFIG_ARM_PARAM.guide_elongate_value,0,CONFIG_GUN_PARAM.alignment_well_distance,true,CONFIG_TASK_PARAM.attach_guide_aligned_timeout);
}
WS_STATUS SlimArm::GuideContract(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGuide].actionThread=std::make_unique<std::jthread>([&](){
      GuideContract(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GuideContract",IndexPressureGuide,CONFIG_ARM_PARAM.guide_contract_value,0,CONFIG_GUN_PARAM.alignment_laser_original,false,CONFIG_TASK_PARAM.attach_guide_aligned_timeout);
}
WS_STATUS SlimArm::GuideElongateByPressure(int16_t pressurecmd,bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGuide].actionThread=std::make_unique<std::jthread>([&,pressurecmd](){
      GuideElongateByPressure(pressurecmd,false);
    });
    return WS_OK;
  }
  return chamberMoveAtPressure("GuideElongateByPressure",IndexPressureGuide,pressurecmd,CONFIG_TASK_PARAM.attach_guide_aligned_timeout);
}
WS_STATUS SlimArm::GuideElongateByLaser(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGuide].actionThread=std::make_unique<std::jthread>([&](){
      GuideElongateByLaser(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GuideElongateByLaser",IndexPressureGuide,CONFIG_ARM_PARAM.guide_elongate_value,0,CONFIG_GUN_PARAM.alignment_well_distance ,true,CONFIG_TASK_PARAM.attach_guide_aligned_timeout);
}
WS_STATUS SlimArm::GuideContractByLaser(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGuide].actionThread=std::make_unique<std::jthread>([&](){
      GuideContractByLaser(false);
    });
    return WS_OK;
  }
  return chamberMoveAtPosition("GuideContractByLaser",IndexPressureGuide,CONFIG_ARM_PARAM.guide_contract_value,0,CONFIG_GUN_PARAM.alignment_laser_original,false,CONFIG_TASK_PARAM.attach_guide_aligned_timeout);
}



//Action grasp 
WS_STATUS SlimArm::GripperOpen(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGrasp].actionThread=std::make_unique<std::jthread>([&](){
      GripperOpen(false);
    });
    return WS_OK;
  }

  WS_STATUS ret = chamberMoveAtPressure("GripperOpen",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_open_value,CONFIG_COMPENSATION_PARAM.time_gripper_grasp_delay/1000);
  if( ret == WS_OK ) 
    { LOG_F(INFO,"[Action GripperOpen] [MoveAtPressure - Successful]"); }
  else
    { LOG_F(INFO,"[Action GripperOpen] [MoveAtPressure - Failed]"); }

  return ret;

  // return chamberMoveAtPressure("GripperOpen",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_open_value,CONFIG_COMPENSATION_PARAM.time_gripper_grasp_delay/1000);
}

WS_STATUS SlimArm::GripperClose(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureGrasp].actionThread=std::make_unique<std::jthread>([&](){
      GripperClose(false);
    });
    return WS_OK;
  }

  WS_STATUS ret = chamberMoveAtPressure("GripperClose",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_close_value,CONFIG_COMPENSATION_PARAM.time_gripper_grasp_delay/1000);
  if( ret == WS_OK ) 
    { LOG_F(INFO,"[Action GripperClose] [MoveAtPressure - Successful]"); }
  else
    { LOG_F(INFO,"[Action GripperClose] [MoveAtPressure - Failed]"); }
    // { 
    //   LOG_F(INFO,"[Action GripperClose] [MoveAtPressure - Failed] try it again."); 
    //   WS_STATUS ret_again = chamberMoveAtPressure("GripperClose",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_close_value,CONFIG_COMPENSATION_PARAM.time_gripper_grasp_delay/1000);
    //   if( ret_again == WS_OK ) 
    //     { LOG_F(INFO,"[Action GripperClose] [GripperClose: 2nd - Successful]"); }
    //   else 
    //     { LOG_F(INFO,"[Action GripperClose] [MoveAtPressure - Final failed]"); }
    //   return ret_again;
    // }

  return ret;

  // return chamberMoveAtPressure("GripperClose",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_close_value,CONFIG_COMPENSATION_PARAM.time_gripper_grasp_delay/1000);
}
 
 
//Action shift 
WS_STATUS SlimArm::GripperShiftCentral(bool async){
  // chamberMoveAtPressure("GripperShiftCentral",IndexPressureShift,500,0.5);
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if(async){
    m_actionMoveChamber[IndexPressureShift].actionThread=std::make_unique<std::jthread>([&](){
      GripperShiftCentral(false);
    });
    return WS_OK;
  }
  std::array<uint8_t,2> ioIndexes={2,3};
  std::array<uint8_t,2> ioTargetStates{0,1};
  // return chamberMoveAtIOFlags("GripperShiftCentral",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_central_value,ioIndexes,ioTargetStates,true,5);
  auto ret = chamberMoveAtPressure("GripperShiftCentral",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_central_value,5);

  if( ret == WS_OK ) 
    { LOG_F(INFO,"[Action GripperShiftCentral] [MoveAtPressure - Successful]"); }
  else
    { LOG_F(INFO,"[Action GripperShiftCentral] [MoveAtPressure - Failed]"); }

  ret = ( m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0] && 
          m_arm_data_.io_status[ioIndexes[1]]==ioTargetStates[1] )? WS_OK:WS_FAIL;

  auto func_start_time = TIC();
  double timeout_max = 2.0f;

  while( ret != WS_OK ) {
    LOG_F(INFO,"[Action GripperShiftCentral] [Sensor - Waiting signal]");

    if (TOC(func_start_time) > timeout_max) {
      LOG_F(INFO,"[Action GripperShiftCentral] [Sensor - Waiting timeout]");
      LOG_F(INFO,"[Action GripperShiftCentral] [Sensor - Failed]");
      return ret;
    }

    ret = ( m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0] && 
            m_arm_data_.io_status[ioIndexes[1]]==ioTargetStates[1] )? WS_OK:WS_FAIL;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if(ret==WS_OK){
    LOG_F(INFO,"[Action GripperShiftCentral] [Sensor - Successful]");
  }
  // else{
  //   LOG_F(INFO,"[Action GripperShiftCentral] [Sensor -Failed]");
  // }
  return ret;
}

WS_STATUS SlimArm::GripperShiftCentralByPressure(int pressurecmd,bool async){
  if(async){
    m_actionMoveChamber[IndexPressureShift].actionThread=std::make_unique<std::jthread>([&](){
      GripperShiftCentralByPressure(pressurecmd,false);
    });
    return WS_OK;
  }
  std::array<uint8_t,2> ioIndexes={2,3};
  std::array<uint8_t,2> ioTargetStates{0,1};
  // return chamberMoveAtIOFlags("GripperShiftCentral",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_central_value,ioIndexes,ioTargetStates,true,5);
  auto ret = chamberMoveAtPressure("GripperShiftCentralByPressure",IndexPressureShift,pressurecmd,5);
  // ret = ( m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0] && 
  //         m_arm_data_.io_status[ioIndexes[1]]==ioTargetStates[1] )? WS_OK:WS_FAIL;

  if(ret==WS_OK){
    LOG_F(INFO,"[Action GripperShiftCentralByPressure] [Successful]");
  }
  else{
    LOG_F(INFO,"[Action GripperShiftCentralByPressure] [Failed]");
  }
  return ret;
}

WS_STATUS SlimArm::GripperShiftAway(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureShift].actionThread=std::make_unique<std::jthread>([&](){
      GripperShiftAway(false);
    });
    return WS_OK;
  }
  std::array<uint8_t,2> ioIndexes={2,3};
  std::array<uint8_t,2> ioTargetStates{1,0};
  // return chamberMoveAtIOFlags("GripperShiftAway",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_away_value,ioIndexes,ioTargetStates,true,5);
  auto ret = chamberMoveAtPressure("GripperShiftAway",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_away_value,8);

  if( ret == WS_OK ) 
    { LOG_F(INFO,"[Action GripperShiftAway] [MoveAtPressure - Successful]"); }
  else
    { LOG_F(INFO,"[Action GripperShiftAway] [MoveAtPressure - Failed]"); }

  ret = ( m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0] && 
          m_arm_data_.io_status[ioIndexes[1]]==ioTargetStates[1] )? WS_OK:WS_FAIL;

  auto func_start_time = TIC();
  double timeout_max = 2.0f;

  while( ret != WS_OK ) {
    LOG_F(INFO,"[Action GripperShiftAway] [Sensor - Waiting signal]");

    if (TOC(func_start_time) > timeout_max) {
      LOG_F(WARNING,"[Action GripperShiftAway] [Sensor - Waiting timeout]");
      LOG_F(WARNING,"[Action GripperShiftAway] [Sensor - Failed]");
      return ret;
    }
    
    ret = ( m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0] && 
            m_arm_data_.io_status[ioIndexes[1]]==ioTargetStates[1] )? WS_OK:WS_FAIL;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if(ret==WS_OK){
    LOG_F(INFO,"[Action GripperShiftAway] [Sensor - Successful]");
  }
  // else{
  //   LOG_F(INFO,"[Action GripperShiftAway] [Sensor - Failed]");
  // }
  return ret;
}
 
//Action Lock
WS_STATUS SlimArm::GunLock(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureLock].actionThread=std::make_unique<std::jthread>([&](){
      GunLock(false);
    });
    return WS_OK;
  }
  std::array<uint8_t,1> ioIndexes={7};
  std::array<uint8_t,1> ioTargetStates{0};
  //return chamberMoveAtIOFlags("GunLock",IndexPressureLock,CONFIG_ARM_PARAM.gun_lock_value,ioIndexes,ioTargetStates,timeout);
  WS_STATUS ret = chamberMoveAtPressure("GunLock",IndexPressureLock,CONFIG_ARM_PARAM.gun_lock_value,3);

  if( ret == WS_OK ) 
    { LOG_F(INFO,"[Action GunLock] [MoveAtPressure - Successful]"); }
  else
    { LOG_F(INFO,"[Action GunLock] [MoveAtPressure - Failed]"); }

  ret = (m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0])?WS_OK:WS_FAIL;

  auto func_start_time = TIC();
  double timeout_max = 1.0f;

  while( ret != WS_OK ) {
    LOG_F(INFO,"[Action GunLock] [Sensor - Waiting signal]");

    if (TOC(func_start_time) > timeout_max) {
      LOG_F(WARNING,"[Action GunLock] [Sensor - Waiting timeout]");
      LOG_F(WARNING,"[Action GunLock] [Sensor - Failed]");
      return ret;
    }
    
    ret = (m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0])?WS_OK:WS_FAIL;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if(ret == WS_OK)
    { LOG_F(INFO,"[Action GunLock] [Sensor - Successful]"); }

  // /*Ignore sensor signal just for test*/
  // LOG_F(INFO,"[Action GunLock] Ignore sensor signal...");
  // return WS_STATUS::WS_OK;
  // /**/

  return ret;
}
 
WS_STATUS SlimArm::GunUnlock(bool async){
  if(async){
    m_actionMoveChamber[IndexPressureLock].actionThread=std::make_unique<std::jthread>([&](){
      GunUnlock(false);
    });
    return WS_OK;
  }
  std::array<uint8_t,1> ioIndexes={7};
  std::array<uint8_t,1> ioTargetStates{1};
  // return chamberMoveAtIOFlags("GunUnlock",IndexPressureLock,CONFIG_ARM_PARAM.gun_unlock_value,ioIndexes,ioTargetStates,timeout);
  auto ret = chamberMoveAtPressure("GunUnlock",IndexPressureLock,CONFIG_ARM_PARAM.gun_unlock_value,3);

  if( ret == WS_OK ) 
    { LOG_F(INFO,"[Action GunUnlock] [MoveAtPressure - Successful]"); }
  else
    { LOG_F(WARNING,"[Action GunUnlock] [MoveAtPressure - Failed]"); }

  ret = (m_arm_data_.io_status[ioIndexes[0]] == ioTargetStates[0]) ? WS_OK : WS_FAIL;

  auto func_start_time = TIC();
  double timeout_max = 1.0f;

  while( ret != WS_OK ) {
    LOG_F(INFO,"[Action GunUnlock] [Sensor - Waiting signal]");

    if (TOC(func_start_time) > timeout_max) {
      LOG_F(WARNING,"[Action GunUnlock] [Sensor - Waiting timeout]");
      LOG_F(WARNING,"[Action GunUnlock] [Sensor - Failed]");
      return ret;
    }
    
    ret = (m_arm_data_.io_status[ioIndexes[0]] == ioTargetStates[0]) ? WS_OK : WS_FAIL;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if(ret == WS_OK)
    { LOG_F(INFO,"[Action GunUnlock] [Sensor - Successful]"); }

  // /*Ignore sensor signal just for test*/
  // LOG_F(INFO,"[Action GunUnlock] Ignore sensor signal...");
  // return WS_STATUS::WS_OK;
  // /**/

  return ret;
}



WS_STATUS SlimArm::chamberMoveAtPressure(std::string actionName,int pressureIndex,int targetPressure,double timeout){
  return m_actionMoveChamber[pressureIndex].act( actionName,
                          [&,pressureIndex,targetPressure](ArmActionProgress &progress){
                              if(!isAlive()){
                                progress.message = fmt::format("Monos Arm is not responding");
                                return false;
                              }
                              m_arm_data_.pressureCmd.pressureArray[pressureIndex] = targetPressure;
                              progress.message = fmt::format("Desired pressureCmd:{}",targetPressure);
                              return true;
                              }, //start condition. 
                          [&](ArmActionProgress &progress){return false;}, //stop condition. 
                          [&,pressureIndex,targetPressure](ArmActionProgress &progress){progress.progressIndicator = std::abs(targetPressure-getChamberPressure(pressureIndex));return true;},//progress indication
                          [&,pressureIndex,targetPressure](ArmActionProgress &progress){//complete condition
                              progress.messageFrequencyDivide = 20; //slower message 
                              progress.message = fmt::format("Current pressure:{}",getChamberPressure(pressureIndex));

                              //satisfy distance condition 
                              if(withinPressureDeadzone(pressureIndex) && isSteadyPressure(pressureIndex)){
                                progress.percentage = 100;
                                return true;
                              }
                              return false;
                              },//complete condition.  
                          timeout, 
                          std::bind(&SlimArmRTDE::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                          pressureIndex,
                          targetPressure
                          );
}
  #if SLIM_ARM_TYPE == SlimArm_Type_Monos3
WS_STATUS SlimArm::chamberMoveAtPositionPrecisely(std::string actionName, int pressureIndex, int maxPressure, int laserIndex, int targetPosition, double timeout)
{
  return m_actionMoveChamber[pressureIndex].act( actionName,
                          [&,pressureIndex,maxPressure,targetPosition](ArmActionProgress &progress){
                            if(!isAlive()){
                              progress.message = fmt::format("Monos Arm is not responding");
                              return false;
                            }  
                            m_arm_data_.pressureCmd.pressureArray[pressureIndex] = getChamberPressure(pressureIndex);
                            progress.message = fmt::format("Desired positionCmd:{}",targetPosition);
                            return true;
                            }, //start condition. 
                          [&](ArmActionProgress &progress){return false;}, //stop condition. 
                          [&,pressureIndex,targetPosition,laserIndex](ArmActionProgress &progress){
                            //control at target position
                            int distanceDifference = targetPosition - m_arm_data_.laser_distance_mm[laserIndex];
                            progress.progressIndicator = (distanceDifference != 0) ? std::abs(distanceDifference) : 0;

                            if( progress.progressIndicator != 0 ) 
                              { m_arm_data_.pressureCmd.pressureArray[pressureIndex] += (distanceDifference > 0) ? 1 : -1; }

                            return true;},//progress indication
                          [&,pressureIndex,targetPosition,laserIndex](ArmActionProgress &progress){//complete condition
                            progress.messageFrequencyDivide = 20; //slower message 
                            progress.message = fmt::format("Current Pressure:{}, Current Position:{}, error={}",getChamberPressure(pressureIndex),m_arm_data_.laser_distance_mm[laserIndex], (targetPosition-m_arm_data_.laser_distance_mm[laserIndex]));
                            rtde_client_.commandChamber(pressureIndex,m_arm_data_.pressureCmd.pressureArray[pressureIndex]);

                            //satisfy distance condition 
                            if(progress.progressIndicator==0){
                              progress.percentage = 100;
                              return true;
                            }

                            return false;
                            },//complete condition.  
                          timeout, 
                          std::bind(&SlimArmRTDE::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                          pressureIndex,
                          m_arm_data_.pressureCmd.pressureArray[pressureIndex]
                          );
} 
  #endif
WS_STATUS SlimArm::chamberMoveAtPosition(std::string actionName,int pressureIndex,int targetPressure,int laserIndex,int targetPosition,bool flagLargerThan,double timeout){
  
  return m_actionMoveChamber[pressureIndex].act( actionName,
                          [&,pressureIndex,targetPressure,targetPosition](ArmActionProgress &progress){
                              if(!isAlive()){
                                 progress.message = fmt::format("Monos Arm is not responding");
                                 return false;
                              }  
                              m_arm_data_.pressureCmd.pressureArray[pressureIndex] = targetPressure;
                              progress.message = fmt::format("Desired pressureCmd:{}, desired positionCmd:{}",targetPressure,targetPosition);
                              return true;
                              }, //start condition. 
                          [&](ArmActionProgress &progress){return false;}, //stop condition. 
                          [&,pressureIndex,targetPosition,laserIndex,flagLargerThan](ArmActionProgress &progress){
                            if(flagLargerThan){//elongate until targetPosition 
                               progress.progressIndicator = 
                            (targetPosition-m_arm_data_.laser_distance_mm[laserIndex])>0?std::abs(targetPosition-m_arm_data_.laser_distance_mm[laserIndex]):0;
                            }
                            else{//contract until targetPosition 
                               progress.progressIndicator = 
                            (targetPosition-m_arm_data_.laser_distance_mm[laserIndex])<0?std::abs(targetPosition-m_arm_data_.laser_distance_mm[laserIndex]):0;
                            }
                            return true;},//progress indication
                          [&,pressureIndex,targetPosition,laserIndex](ArmActionProgress &progress){//complete condition
                              progress.messageFrequencyDivide = 20; //slower message 
                              progress.message = fmt::format("Current Pressure:{}, Current Position:{}, error={}",getChamberPressure(pressureIndex),m_arm_data_.laser_distance_mm[laserIndex], (targetPosition-m_arm_data_.laser_distance_mm[laserIndex]));
                              
                              //satisfy distance condition 
                              if(progress.progressIndicator==0){
                                progress.percentage = 100;
                                progress.message = fmt::format("Current Pressure:{}, Current Position:{}, error={}",getChamberPressure(pressureIndex),m_arm_data_.laser_distance_mm[laserIndex], (targetPosition-m_arm_data_.laser_distance_mm[laserIndex]));
                                rtde_client_.commandChamber(pressureIndex,getChamberPressure(pressureIndex));
                                return true;
                              }

                              return false;
                              },//complete condition.  
                          timeout, 
                          std::bind(&SlimArmRTDE::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                          pressureIndex,
                          targetPressure
                          );
} 

template<size_t N>
WS_STATUS SlimArm::chamberMoveAtIOFlags(std::string actionName,int pressureIndex,int targetPressure,std::array<uint8_t,N> &ioIndexes,std::array<uint8_t,N> &ioTargetStates,bool logicAnd, double timeout){
return m_actionMoveChamber[pressureIndex].act( 
                        actionName,
                        [&,pressureIndex,targetPressure,ioIndexes,ioTargetStates](ArmActionProgress &progress){
                            if(!isAlive()){
                                progress.message = fmt::format("Monos Arm is not responding");
                                return false;
                            }  
                            m_arm_data_.pressureCmd.pressureArray[pressureIndex] = targetPressure;
                            progress.message = fmt::format("Desired pressureCmd:{} ,",targetPressure);
                            progress.message.append(fmt::format("Desired/Current "));
                            for(size_t i=0;i<N;i++){
                              progress.message.append(fmt::format("IO_status[{}]={}/{} ",ioIndexes[i],ioTargetStates[i],m_arm_data_.io_status[ioIndexes[i]]));
                            }
                            return true;
                            }, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&,pressureIndex,targetPressure](ArmActionProgress &progress){progress.progressIndicator = std::abs(targetPressure-getChamberPressure(pressureIndex));return true;},//progress indication
                        [&,pressureIndex,targetPressure,ioIndexes,ioTargetStates,logicAnd](ArmActionProgress &progress){//complete condition
                            progress.messageFrequencyDivide = 20; //slower message 
                            progress.message = fmt::format("Current pressure:{} ,",getChamberPressure(pressureIndex));
                            progress.message.append(fmt::format("Desired/Current "));
                            for(size_t i=0;i<N;i++){
                              progress.message.append(fmt::format("IO_status[{}]={}/{} ",ioIndexes[i],ioTargetStates[i],m_arm_data_.io_status[ioIndexes[i]]));
                            }
                            
                            bool ioResult;
                            if(logicAnd){
                              ioResult = true;
                              for(size_t i=0;i<N;i++){ 
                                ioResult &= (m_arm_data_.io_status[ioIndexes[i]]==ioTargetStates[i]);
                              }
                            }
                            else{
                              ioResult = false;
                              for(size_t i=0;i<N;i++){ 
                               if(m_arm_data_.io_status[ioIndexes[i]]==ioTargetStates[i]){
                                  ioResult = true;
                               }
                              }
                            }
                            //satisfy distance condition 
                            if(ioResult){
                              progress.percentage = 100;
                              rtde_client_.commandChamber(pressureIndex,getChamberPressure(pressureIndex));
                              return true;
                            }
                            return false;
                            },//complete condition.  
                        timeout, 
                        std::bind(&SlimArmRTDE::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                        pressureIndex,
                        targetPressure
                        );
}









WS_STATUS SlimArm::MoveJointPrecisely(std::array<float,6> &desired_joint,float pitch_deadzone, double timeout){
  return m_actionMoveJoint.act(
          "MovePre",
          [&](ArmActionProgress &progress){//start condition
            if(!isAlive()){
                progress.message = fmt::format("Monos Arm is not responding");
                return false;
            }
            if(VerifyJointIsWithin(desired_joint)){
              m_arm_data_.q_desired_ = desired_joint;
              progress.message =  fmt::format("q_desired:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_desired_[0], m_arm_data_.q_desired_[1], m_arm_data_.q_desired_[2], m_arm_data_.q_desired_[3], m_arm_data_.q_desired_[4], m_arm_data_.q_desired_[5]);
              return true;
            }
            else{
              progress.message =  fmt::format("q_desired overlimit:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_desired_[0], m_arm_data_.q_desired_[1], m_arm_data_.q_desired_[2], m_arm_data_.q_desired_[3], m_arm_data_.q_desired_[4], m_arm_data_.q_desired_[5]);
              return false;
            }
          },
          [&](ArmActionProgress &progress){return false;},//stop condition
          [&](ArmActionProgress &progress){progress.progressIndicator = vectorDistance(m_arm_data_.q_desired_,m_arm_data_.q_current_);return true;},//progress indication 
          [&](ArmActionProgress &progress){//complete check condition
            auto q_error = m_arm_data_.q_desired_-m_arm_data_.q_current_;

            //enter critical zone
            if(WithinJointDeadZone(q_error,pitch_deadzone)){
              progress.tempDataInt[0]++;//accumulate critical zone time
              progress.messageFrequencyDivide = 10;//faster message 
  
              //enter steady zone
              progress.tempDataInt[1] = getSteadyPitchCount(pitch_deadzone);
              if(progress.tempDataInt[1]>0){
                progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] within:{} steady:{}", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                q_error[4], q_error[5],progress.tempDataInt[0],progress.tempDataInt[1]);

                //complete condition
                if(getSteadyPitchCount(pitch_deadzone)>m_pitch_steady_count_max){
                  progress.percentage = 100;
                  return true;
                }
 
              }
              else{
                progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] within:{}", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                q_error[4], q_error[5],progress.tempDataInt[0]);
              }
 
 
            }
            else{
              progress.tempDataInt[0]=0;//clear critical zone time
              progress.messageFrequencyDivide = 20; //slower message 
              progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                                m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                                q_error[4], q_error[5]);
            }
            return false;
          },
          timeout, 
          std::bind(&SlimArmRTDE::commandJoint, &rtde_client_, std::placeholders::_1),
          desired_joint);
}

void SlimArm::MoveJointPreciselyAsync(std::array<float,6> &desired_joint,float pitch_deadzone, double timeout){
 
  m_actionMoveJoint.actionThread = std::make_unique<std::jthread>([&,timeout](){
      MoveJointPrecisely(desired_joint,pitch_deadzone,timeout);
    }
  );
}
 

WS_STATUS MoveLine(TransFormMatrix &transform_T, double timeout){

  return WS_OK;
}
void MoveLineAsync(TransFormMatrix &transform_T,double timeout = 30 ){

}


WS_STATUS SlimArm::MoveJoint(TransFormMatrix &transform_T, double timeout) {
   
    m_arm_data_.T_desired_ = transform_T;
    auto q_desired = InverseKinematics(m_arm_data_.T_desired_);

    return MoveJoint(q_desired,timeout);
 
}


WS_STATUS SlimArm::MoveJoint(std::array<float,6> &desired_joint, double timeout){

  return m_actionMoveJoint.act(
          "MoveJoint",
          [&](ArmActionProgress &progress){//start condition
            if(!isAlive()){
                progress.message = fmt::format("Monos Arm is not responding");
                return false;
            }
            if(VerifyJointIsWithin(desired_joint)){
              m_arm_data_.q_desired_ = desired_joint;
              progress.message =  fmt::format("q_desired:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_desired_[0], m_arm_data_.q_desired_[1], m_arm_data_.q_desired_[2], m_arm_data_.q_desired_[3], m_arm_data_.q_desired_[4], m_arm_data_.q_desired_[5]);
              return true;
            } else{
              progress.message =  fmt::format("q_desired:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] || q_upper_limit:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] || q_lower_limit:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", 
                                              desired_joint[0], desired_joint[1], desired_joint[2], 
                                              desired_joint[3], desired_joint[4], 
                                              CONFIG_ARM_PARAM.joint_upper_limit[0], CONFIG_ARM_PARAM.joint_upper_limit[1], CONFIG_ARM_PARAM.joint_upper_limit[2],
                                              CONFIG_ARM_PARAM.joint_upper_limit[3], CONFIG_ARM_PARAM.joint_upper_limit[4], 
                                              CONFIG_ARM_PARAM.joint_lower_limit[0], CONFIG_ARM_PARAM.joint_lower_limit[1], CONFIG_ARM_PARAM.joint_lower_limit[2],
                                              CONFIG_ARM_PARAM.joint_lower_limit[3], CONFIG_ARM_PARAM.joint_lower_limit[4]
                                              );
              return false;
            }
          },
          [&](ArmActionProgress &progress){return false;},//stop condition
          [&](ArmActionProgress &progress){progress.progressIndicator = vectorDistance(m_arm_data_.q_desired_,m_arm_data_.q_current_);return true;},//progress indication 
          [&](ArmActionProgress &progress){//complete check condition
            std::array<float,6> q_error = m_arm_data_.q_desired_-m_arm_data_.q_current_;
 
            //enter critical zone
            if(WithinJointDeadZone(q_error)){
              progress.tempDataInt[0]++;//accumulate critical zone time
              progress.messageFrequencyDivide = 10;//faster message 
              progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] within:{}", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                                              m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                                              q_error[4], q_error[5],progress.tempDataInt[0]);
              
              //complete
              if(progress.tempDataInt[0]>20){
                progress.percentage = 100;
                return true;
              }
            }
            else{
              progress.tempDataInt[0]=0;//clear critical zone time
              progress.messageFrequencyDivide = 20; //slower message 
              progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                                m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                                q_error[4], q_error[5]);
            }
            return false;
          },
          timeout, 
          std::bind(&SlimArmRTDE::commandJoint, &rtde_client_, std::placeholders::_1),
          desired_joint);
}

void SlimArm::MoveJointAsync(std::array<float,6> &desired_joint, double timeout){
  m_actionMoveJoint.actionThread = std::make_unique<std::jthread>([&,timeout](){
      MoveJoint(desired_joint,timeout);
    }
  );
}


WS_STATUS SlimArm::MoveJointDelta(std::array<float,6> delta_desired_joint, double timeout){
  std::array<float,6> desired_joint = delta_desired_joint + m_arm_data_.q_current_;
  return MoveJoint(desired_joint,timeout);
}
void SlimArm::MoveJointDeltaAsync(std::array<float,6> delta_desired_joint, double timeout){
  std::array<float,6> desired_joint = delta_desired_joint + m_arm_data_.q_current_;
  m_actionMoveJoint.actionThread = std::make_unique<std::jthread>([&,timeout](){
      MoveJoint(desired_joint,timeout);
    }
  );
}


WS_STATUS SlimArm::stopMoveJoint() {
  LOG_F(INFO, "[SlimArm] Stop MoveJoint");
  std::array<float,6> stop_joint = this->GetArmCurrentJoint();
  if( std::abs(stop_joint[4]) < 0.06 ) 
    { stop_joint[4] = 0.0f; }

  if(MoveJoint(stop_joint, 2.0 )!=WS_OK){
    //shouldn't be here
    m_actionMoveJoint.stopAction();
    return WS_FAIL;
  }
  //m_actionMoveJoint.stopAction();
  return WS_OK;
}


WS_STATUS SlimArm::MoveX(float desired_joint_single,double timeout){
  std::string actionName = "MoveX";
  int axisIndex = 0; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,true,timeout);
}
WS_STATUS SlimArm::MoveY(float desired_joint_single,double timeout){
  std::string actionName = "MoveY";
  int axisIndex = 1; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,true,timeout);  
}
WS_STATUS SlimArm::MoveZ(float desired_joint_single,double timeout){
  std::string actionName = "MoveZ";
  int axisIndex = 2; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,true,timeout);
}
WS_STATUS SlimArm::MoveR(float desired_joint_single,double timeout){
  std::string actionName = "MoveR";
  int axisIndex = 3; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,true,timeout);  
}
WS_STATUS SlimArm::MoveX_Delta(float desired_joint_single,double timeout){
  std::string actionName = "MoveX_delta";
  int axisIndex = 0; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,false,timeout);
}
WS_STATUS SlimArm::MoveY_Delta(float desired_joint_single,double timeout){
  std::string actionName = "MoveY_delta";
  int axisIndex = 1; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,false,timeout);  
}
WS_STATUS SlimArm::MoveZ_Delta(float desired_joint_single,double timeout){
  std::string actionName = "MoveZ_delta";
  int axisIndex = 2; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,false,timeout);
}
WS_STATUS SlimArm::MoveR_Delta(float desired_joint_single,double timeout){
  std::string actionName = "MoveR_delta";
  int axisIndex = 3; 
  return MovejointSingle(actionName,axisIndex,desired_joint_single,false,timeout);
}

WS_STATUS SlimArm::MoveZ_Once_FinishControlPitch(float desired_pitch, float p_coefficient, double timeout){
  std::string actionName = "MoveZ_Once_FinishControlPitch";
  int axisIndex = 2;
  float pitch_change = desired_pitch - m_arm_data_.q_current_[4];
  float z_adjust = p_coefficient * pitch_change; 
  return MovejointSingle(actionName,axisIndex,z_adjust,false,timeout);
}

WS_STATUS SlimArm::MovejointSingle(std::string actionName, int axisIndex, float desired_joint_single, bool isAbsolute,double timeout){
 
  return m_actionMoveJoint.act(
          actionName,
          [&,axisIndex,desired_joint_single,isAbsolute](ArmActionProgress &progress){//start condition
            if(!isAlive()){
                progress.message = fmt::format("Monos Arm is not responding");
                return false;
            }
            if(axisIndex>=0 && axisIndex<=3){

              m_arm_data_.q_desired_  = m_arm_data_.q_current_;
              if(isAbsolute){
                  m_arm_data_.q_desired_[axisIndex] = desired_joint_single;
              }
              else{
                  m_arm_data_.q_desired_[axisIndex] += desired_joint_single;
              }
              
              progress.message =  fmt::format(" Axis {} desired:[{:.4f}]", AxisNames[axisIndex], m_arm_data_.q_desired_[axisIndex]);
              return true;
            }
            else{
              progress.message =  fmt::format(" Axis {} overlimit", AxisNames[axisIndex]);
              return false;
            }
          },
          [&](ArmActionProgress &progress){return false;},//stop condition
          [&,axisIndex](ArmActionProgress &progress){progress.progressIndicator = std::abs(m_arm_data_.q_desired_[axisIndex]-m_arm_data_.q_current_[axisIndex]);return true;},//progress indication 
          [&,axisIndex](ArmActionProgress &progress){//complete check condition
            float q_error = m_arm_data_.q_desired_[axisIndex]-m_arm_data_.q_current_[axisIndex];
 
            //enter critical zone
            if( std::abs(q_error) < CONFIG_ARM_PARAM.joint_dead_zone[axisIndex] ){
              progress.tempDataInt[0]++;//accumulate critical zone time
              progress.messageFrequencyDivide = 10;//faster message 
              progress.message = fmt::format("Axis {} current:[{:.4f}] error:[{:.4f}] within:{}", AxisNames[axisIndex], m_arm_data_.q_current_[axisIndex], q_error ,progress.tempDataInt[0]);
              
              //complete
              if(progress.tempDataInt[0]>20){
                progress.percentage = 100;
                return true;
              }
            }
            else{
              progress.tempDataInt[0]=0;//clear critical zone time
              progress.messageFrequencyDivide = 20; //slower message 
              progress.message = fmt::format("Axis {} current:[{:.4f}] error:[{:.4f}]", AxisNames[axisIndex], m_arm_data_.q_current_[axisIndex], q_error);
              
            }
            return false;
          },
          timeout, 
          std::bind(&SlimArmRTDE::commandJointSingle, &rtde_client_, std::placeholders::_1,std::placeholders::_2),
          axisIndex,
          m_arm_data_.q_desired_[axisIndex]);
}



WS_STATUS SlimArm::SetSpeed(std::string actionName,std::array<int16_t, 6> &speed,double timeout) {
return m_actionSingleCommand.act( actionName,
                        [&](ArmActionProgress &progress){
                          if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}
                          progress.message = fmt::format("target speed:[{} {} {} {} {} {}]",speed[0],speed[1],speed[2],speed[3],speed[4],speed[5]);
                          return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&SlimArmRTDE::commandVelocity, &rtde_client_, std::placeholders::_1),
                        speed
                        );
}

WS_STATUS SlimArm::SetSpeedZ(int16_t z_speed,double timeout) {
  m_arm_data_.joint_speed_[2] = z_speed;
  return SetSpeed("Set Speed Z",m_arm_data_.joint_speed_,timeout);
}

WS_STATUS SlimArm::SetSpeed(std::string speed_level, double timeout) {
  #if SLIM_ARM_TYPE == SlimArm_Type_Default
    m_speed_level = speed_level;
    if (speed_level == "high") {
      std::array<int16_t,6> speedvalue = {1000, 1200, 1200, 125, 0, 0};
      m_arm_data_.joint_speed_ = speedvalue;
    }
    else if (speed_level == "middle") {
      std::array<int16_t,6> speedvalue = {800, 1200, 1200, 125, 0, 0};
      m_arm_data_.joint_speed_ = speedvalue;
    }
    else if (speed_level == "low") {
      std::array<int16_t,6> speedvalue = {800, 1000, 1000, 125, 0, 0};
      m_arm_data_.joint_speed_ = speedvalue;
    }
    return SetSpeed("Set Speed Level "+speed_level, m_arm_data_.joint_speed_,timeout);

  #else if SLIM_ARM_TYPE == SlimArm_Type_Monos3
    m_speed_level = speed_level;
    if (speed_level == "high") {
      std::array<int16_t,6> speedvalue = {500.0, 800.0, 1200.0, 100.0, 0.0, 0.0};
      m_arm_data_.joint_speed_ = speedvalue;
    }
    else if (speed_level == "middle") {
      std::array<int16_t,6> speedvalue = {400.0, 800.0, 1200, 100.0, 0.0, 0.0};
      m_arm_data_.joint_speed_ = speedvalue;
    }
    else if (speed_level == "low") {
      std::array<int16_t,6> speedvalue = {200.0, 800.0, 1000, 100.0, 0.0, 0.0};
      m_arm_data_.joint_speed_ = speedvalue;
    }

    return SetSpeed("Set Speed Level "+speed_level, m_arm_data_.joint_speed_,timeout);

  #endif
   }



 WS_STATUS SlimArm::MovePitch(float target_pitch, double timeout) {
  if(target_pitch==0){
    return MovePitch("MovePitch",target_pitch,m_pitch_deadzone_0degree, m_pitch_delta_deadzone_default,timeout);
  }
  else{
    return MovePitch("MovePitch",target_pitch,m_pitch_deadzone_default, m_pitch_delta_deadzone_default,timeout);
  }
  
}

 

WS_STATUS SlimArm::MovePitch(std::string actionName,float pitch_desired,float pitch_deadzone,float pitch_delta_deadzone,double timeout){
 
  return m_actionMovePitch.act(
          actionName,
          [&,pitch_desired,pitch_deadzone](ArmActionProgress &progress){//start condition
              if(!isAlive()){
                progress.message = fmt::format("Monos Arm is not responding");
                return false;
              }
              m_arm_data_.q_desired_[4] = pitch_desired;
  
              progress.message =  fmt::format("pitch_desired: {:.1f} , pitch_deadzone: {:.4f}", m_arm_data_.q_desired_[4]/M_PI*180,pitch_deadzone/M_PI*180);
              return true;
          },
          [&](ArmActionProgress &progress){return false;},//stop condition
          [&](ArmActionProgress &progress){progress.progressIndicator = std::abs(m_arm_data_.q_desired_[4]-m_arm_data_.q_current_[4]);return true;},//progress indication 
          [&,pitch_deadzone,pitch_delta_deadzone](ArmActionProgress &progress){//complete check condition
            auto q_error = m_arm_data_.q_desired_[4]-m_arm_data_.q_current_[4];
            //enter critical zone 
            int pitch_within_count = getWithinPitchCount(pitch_deadzone);
            int pitch_steady_count = getSteadyPitchCount(pitch_delta_deadzone);
            if(pitch_within_count>0){
              progress.messageFrequencyDivide = 20;//  message   fre
              if(pitch_within_count >m_pitch_within_count_max){

                
                progress.message = fmt::format("q_current: {:.1f}  error: {:.1f}  within:{} steady:{}",m_arm_data_.q_current_[4]/M_PI*180,q_error/M_PI*180 ,pitch_within_count,pitch_steady_count);
                //complete condition
                if(pitch_steady_count>m_pitch_steady_count_max){
                  progress.percentage = 100;
                  return true;
                }
              }
              else{
                progress.message = fmt::format("pitch_current:{:.1f} error:{:.1f} within:{}", m_arm_data_.q_current_[4]/M_PI*180,q_error/M_PI*180, pitch_within_count);
              }
            }
            else{
              progress.messageFrequencyDivide = 20; //slower message 
              progress.message = fmt::format("pitch_current:{:.1f} error:{:.1f}", m_arm_data_.q_current_[4]/M_PI*180,q_error/M_PI*180);
            }
            return false;
          },
          timeout, 
          std::bind(&SlimArmRTDE::commandPitch, &rtde_client_, std::placeholders::_1),
          pitch_desired);
}

void SlimArm::MovePitchAsync(float target_pitch,double timeout){
  LOG_F(INFO, "[SlimArm] MovePitchAsync");
  if(target_pitch==0){
    MovePitchAsync("MovePitchAsync",target_pitch,m_pitch_deadzone_0degree, m_pitch_delta_deadzone_default,timeout);
  }
  else{
    MovePitchAsync("MovePitchAsync",target_pitch,m_pitch_deadzone_default, m_pitch_delta_deadzone_default,timeout);
  }
}
 

void SlimArm::MovePitchAsync(std::string actionName,float pitch_desired,float pitch_deadzone,float pitch_delta_deadzone,double timeout){

  m_actionMovePitch.actionThread = std::make_unique<std::jthread>([&,actionName,pitch_desired,pitch_deadzone,pitch_delta_deadzone,timeout](){
      MovePitch(actionName,pitch_desired,pitch_deadzone,pitch_delta_deadzone,timeout);
    }
  );
}


WS_STATUS SlimArm::ErectUp(double timeout){
  return MovePitch("ErectUp",0,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}
void SlimArm::ErectUpAsync(double timeout){
    MovePitchAsync("ErectUpAsync",0,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}
 

WS_STATUS SlimArm::BendDown(double timeout){
  return MovePitch("BendDown",M_PI_2,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}
void SlimArm::BendDownAsync(double timeout){
     MovePitchAsync("BendDownAsync",M_PI_2,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}


WS_STATUS SlimArm::PitchHold(bool pitchhold,double timeout){
  return m_actionMovePitch.act( "PitchHold",
                        [&,pitchhold](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}progress.message = pitchhold?"True":"False";return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition.
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){ //complete condition. 
                          progress.percentage = 100;
                          is_Pitchhold = pitchhold;
                          return true;
                        }, 
                        timeout, 
                        std::bind(&SlimArmRTDE::commandPitchHold, &rtde_client_,std::placeholders::_1),
                        pitchhold
                        );
}

 void SlimArm::setDefaultPitchDeadzone(float pitch_deadzone){
   m_pitch_deadzone_default = pitch_deadzone;
 }
 void SlimArm::setDefaultPitchDeltaDeadzone(float pitch_delta_deadzone){
   m_pitch_delta_deadzone_default = pitch_delta_deadzone;
 }

WS_STATUS SlimArm::stopMovePitch(){
  LOG_F(INFO, "[SlimArm] Stop MovePitch");
  return PitchHold(true);
}


bool SlimArm::WithinJointDeadZone(std::array<float,6> &joint_error) {

  if (std::abs(joint_error[0]) < CONFIG_ARM_PARAM.joint_dead_zone[0] &&
      std::abs(joint_error[1]) < CONFIG_ARM_PARAM.joint_dead_zone[1] &&
      std::abs(joint_error[2]) < CONFIG_ARM_PARAM.joint_dead_zone[2] &&
      std::abs(joint_error[3]) < CONFIG_ARM_PARAM.joint_dead_zone[3] &&
      getWithinPitchCount(CONFIG_ARM_PARAM.joint_dead_zone[4]) > 0) {
    return true;
  } else {
    return false;
  }
}


bool SlimArm::WithinJointDeadZone(std::array<float,6> &joint_error,float pitchDeadzone) {
 
  if (std::abs(joint_error[0]) < CONFIG_ARM_PARAM.joint_dead_zone[0] &&
      std::abs(joint_error[1]) < CONFIG_ARM_PARAM.joint_dead_zone[1] &&
      std::abs(joint_error[2]) < CONFIG_ARM_PARAM.joint_dead_zone[2] &&
      std::abs(joint_error[3]) < CONFIG_ARM_PARAM.joint_dead_zone[3] &&
      getWithinPitchCount(pitchDeadzone) > 0) {
    return true;
  } else {
    return false;
  }
}

  
 
 
void SlimArm::updatePitchSteady(){
    m_pitch_delta = m_arm_data_.q_current_[4]-m_pitch_last;
    if(std::abs(m_pitch_delta) < m_pitch_delta_deadzone){
        m_pitch_steady_count++;
        if(m_pitch_steady_count>m_pitch_steady_count_max){
          m_pitch_steady_flag = true;
        }
        else{
          m_pitch_steady_flag = false;
        }
    }
    else{
      m_pitch_steady_count=0;
      m_pitch_steady_flag = false;
    }
    m_pitch_last = m_arm_data_.q_current_[4];
}

void SlimArm::updatePitchWithin(){
    float pitch_err = m_arm_data_.q_desired_[4]-m_arm_data_.q_current_[4];
    if(std::abs(pitch_err) < m_pitch_deadzone){
        m_pitch_within_count++;
        if(m_pitch_within_count>m_pitch_within_count_max){
          m_pitch_within_flag = true;
        }
        else{
          m_pitch_within_flag = false;
        }
    }
    else{
      m_pitch_steady_count=0;
      m_pitch_within_count=0;
      m_pitch_within_flag = false;
    }
}



void SlimArm::addLoggingFile(std::string logFileName,std::string logFileLevel){
    rtde_client_.addLoggingFile(logFileName,logFileLevel);
}


void SlimArm::update(){

      //update sensor data from serial RTDE
      m_arm_data_.laser_distance_mm = rtde_client_.sensorData.laserDistance;
      m_arm_data_.distance_guide = rtde_client_.sensorData.laserDistance[0];
      m_arm_data_.distance_gun = rtde_client_.sensorData.laserDistance[1];
      m_arm_data_.gun_guider_distance = m_arm_data_.laser_distance_mm[0] * 0.001;
      m_arm_data_.pressure.pressureArray = rtde_client_.sensorData.pressure;
      m_arm_data_.p_source = rtde_client_.sensorData.pSource;
      m_arm_data_.p_sink = rtde_client_.sensorData.pSink;
       
      m_arm_data_.io_status = rtde_client_.sensorData.IOFlags;
      m_arm_data_.rotation_encoder = rtde_client_.sensorData.rotationEncoder;
      m_arm_data_.ultra_sonic = rtde_client_.sensorData.ultraSonic;
      m_arm_data_.errorList = rtde_client_.sensorData.errorList;
      m_arm_data_.q_current_ = rtde_client_.sensorData.jointStatef;

      //update FK
      m_arm_data_.T_current_ = ForwardKinematics(m_arm_data_.q_current_);

      //update exact FK
      //m_arm_data_.T_current_exact = ForwardKinematicsExact(m_arm_data_.q_current_,m_arm_data_.rotation_encoder);

      //update pitch steady status
      updatePitchWithin();
      updatePitchSteady();

      //update IO steady status
      updateIOSteady();
 
      //update pressure steady status
      updatePressureSteady();

      //update flag
      m_flagDataUpdated = true;

      //log data

}

 
  void SlimArm::updateIOSteady(){
    for(size_t i=0;i<m_arm_data_.io_status.size();i++){
        if(m_arm_data_.io_status[i] == 1){
          if(m_io_steady_count[i]<=m_io_steady_count_max){
            m_io_steady_count[i]++;
          }
        }
        else{
          if(m_io_steady_count[i]>=-m_io_steady_count_max){
            m_io_steady_count[i]--;
          }
        }
    }
  }
 
  
  void SlimArm::updatePressureSteady(){
    for(size_t i=0;i<m_arm_data_.pressure.pressureArray.size();i++){
       if(withinPressureDeadzone(i)){
          if(m_pressure_steady_count.pressureArray[i]<=m_pressure_steady_count_max){
            m_pressure_steady_count.pressureArray[i]++;
          }
       }
       else{
          if(m_pressure_steady_count.pressureArray[i]>=0){
            m_pressure_steady_count.pressureArray[i]--;
          }
       }
    }
  }
 
 
std::vector<float> SlimArm::LinkH2L(float pitch_angle_rad) {
  float H5 = link_H_[3] * (1 + 2 * (cos(pitch_angle_rad / 3.0) + cos(pitch_angle_rad / 6.0)));
  float Lr = H5 / 2.0 / cos(pitch_angle_rad / 2.0);
  std::vector<float> link = {link_H_[0], link_H_[1], link_H_[2] + Lr, link_H_[4] + Lr};
  return link;
}
std::array<float,6> SlimArm::InverseKinematics(TransFormMatrix &transform_T) {
  double epsilon = 1e-6;
  std::array<float,6> joint = {0, 0, 0, 0, 0, 0};
  if (transform_T(2, 2) > 1) {
    transform_T(2, 2) = 1;
  }
  if (transform_T(2, 2) < -1) {
    transform_T(2, 2) = -1;
  }
  // pitch
  float j4pi6 = acos(transform_T(2, 2));
  float s4pi6 = sin(j4pi6);
  float c4pi6 = transform_T(2, 2);
  if (std::abs(j4pi6) < epsilon) {
    j4pi6 = 0;
  }
  joint[4] = j4pi6;
  // INFO("joint 4 {}", joint[4]);
  // pitch could be positive or negative in monos2
  if (j4pi6 != 0) {
    joint[3] = atan2(transform_T(1, 2) / s4pi6, transform_T(0, 2) / s4pi6);
    joint[5] = atan2(transform_T(2, 1) / s4pi6, -transform_T(2, 0) / s4pi6);
  } else {
    float joint35 = atan2(transform_T(1, 0), transform_T(1, 1));
    joint[3] = joint35;
    joint[5] = 0;
  }

  //  link compensation
  link_L_ = LinkH2L(joint[4]);
  float c3 = cos(joint[3]);
  float s3 = sin(joint[3]);
  joint[0] = transform_T(0, 3) - link_L_[0] * c3 - link_L_[2] * c3 / 2 - link_L_[3] * c3 * s4pi6;

  // Y
  joint[1] = transform_T(1, 3) - link_L_[0] * s3 - link_L_[2] * s3 / 2 - link_L_[3] * s3 * s4pi6;

  // Z
  joint[2] = transform_T(2, 3) - link_L_[1] - link_L_[2] * sqrt(3.0) / 2 - link_L_[3] * c4pi6;
  // INFO(" joint solution: [{},{},{},{},{},{}]", joint[0],
  //                joint[1], joint[2], joint[3], joint[4],
  //                joint[5]);
  return joint;
}

std::array<float,6> SlimArm::InverseKinematics_test(TransFormMatrix &transform_T) {
  double epsilon = 1e-6;
  std::array<float,6> joint = {0, 0, 0, 0, 0, 0};
  if (transform_T(2, 2) > 1) {
    transform_T(2, 2) = 1;
  }
  if (transform_T(2, 2) < -1) {
    transform_T(2, 2) = -1;
  }
  joint[4] = acos(transform_T(2, 2));
  if (std::abs(joint[4]) < epsilon) joint[4] = 0;
  joint[4] = std::abs(joint[4]);
  // INFO("joint 4 {}", joint[4]);
  if (joint[4] != 0) {
    joint[3] = atan2(transform_T(1, 2), transform_T(0, 2));
    joint[5] = atan2(transform_T(2, 1), -transform_T(2, 0));
  } else {
    float joint35 = atan2(transform_T(1, 0), transform_T(1, 1));
    joint[3] = joint35;
    joint[5] = 0;
  }

  link_L_ = LinkH2L(joint[4]);
  //  INFO("link_L_ = {} ,{}", link_L_[0], link_L_[1]);
  joint[0] = transform_T(0, 3) - link_L_[1] * cos(joint[3]) * sin(joint[4]);

  // Y
  joint[1] = transform_T(1, 3) - link_L_[1] * sin(joint[3]) * sin(joint[4]);

  // Z
  joint[2] = transform_T(2, 3) - link_L_[0] - link_L_[1] * cos(joint[4]);
  // INFO(" InverseKinematics_test joint solution: [{},{},{},{},{},{}]", joint[0],
  //                joint[1], joint[2], joint[3], joint[4],
  //                joint[5]);
  return joint;
}
TransFormMatrix SlimArm::ForwardKinematics(std::array<float,6> &joint) {
  TransFormMatrix transformT;
  // std::cout<<"transformT "<<transformT<<std::endl;
  transformT.setIdentity();
  float c3 = cos(joint[3]);
  float s3 = sin(joint[3]);
  float c4 = cos(joint[4]);
  float s4 = sin(joint[4]);
  float c5 = cos(joint[5]);
  float s5 = sin(joint[5]);

  float s4pi6 = s4;//sin(joint[4]);
  float c4pi6 = c4;//cos(joint[4]);
  transformT(0, 0) = c5 * c3 * c4pi6 - s3 * s5;
  transformT(0, 1) = -s5 * c3 * c4pi6 - s3 * c5;
  transformT(0, 2) = c3 * s4pi6;

  transformT(1, 0) = c3 * s5 + c4pi6 * c5 * s3;
  transformT(1, 1) = c3 * c5 - c4pi6 * s3 * s5;
  transformT(1, 2) = s3 * s4pi6;

  transformT(2, 0) = -c5 * s4pi6;
  transformT(2, 1) = s5 * s4pi6;
  transformT(2, 2) = c4pi6;

  // TODO:compensated arm link
  link_L_ = LinkH2L(joint[4]);
  // X
  transformT(0, 3) = joint[0] + link_L_[0] * c3 + link_L_[2] * c3 / 2 + link_L_[3] * c3 * s4pi6;
  // Y
  transformT(1, 3) = joint[1] + link_L_[0] * s3 + link_L_[2] * s3 / 2 + link_L_[3] * s3 * s4pi6;
  // Z
  transformT(2, 3) = joint[2] + link_L_[1] + link_L_[2] * sqrt(3.0) / 2 + link_L_[3] * c4pi6;

  return transformT;
}

TransFormMatrix SlimArm::ForwardKinematicsExact(std::array<float,6> &joint, std::vector<float> &r) {
  TransFormMatrix transformT;
  // std::cout<<"transformT "<<transformT<<std::endl;
  transformT.setIdentity();
  float c3 = cos(joint[3]);
  float s3 = sin(joint[3]);
  float c4 = cos(joint[4]);
  float s4 = sin(joint[4]);
  float c5 = cos(joint[5]);
  float s5 = sin(joint[5]);

  float s4pi6 = s4;//sin(joint[4]);
  float c4pi6 = c4;//cos(joint[4]);
  transformT(0, 0) = c5 * c3 * c4pi6 - s3 * s5;
  transformT(0, 1) = -s5 * c3 * c4pi6 - s3 * c5;
  transformT(0, 2) = c3 * s4pi6;

  transformT(1, 0) = c3 * s5 + c4pi6 * c5 * s3;
  transformT(1, 1) = c3 * c5 - c4pi6 * s3 * s5;
  transformT(1, 2) = s3 * s4pi6;

  transformT(2, 0) = -c5 * s4pi6;
  transformT(2, 1) = s5 * s4pi6;
  transformT(2, 2) = c4pi6;

  // TODO:compensated arm link
  float tempL1 = link_H_[0] + link_H_[2] / 2 + link_H_[3] * sin(r[0]) + link_H_[3] * sin(r[0] + r[1]) +
                 link_H_[3] * sin(r[0] + r[1] + r[2]) + link_H_[3] * sin(r[0] + r[1] + r[2] + r[3]) +
                 link_H_[3] * sin(r[0] + r[1] + r[2] + r[3] + r[4]) +
                 link_H_[4] * sin(r[0] + r[1] + r[2] + r[3] + r[4] + r[5]);
  float tempL2 = link_H_[1] + link_H_[2] * sqrt(3) / 2 + link_H_[3] * cos(r[0]) + link_H_[3] * cos(r[0] + r[1]) +
                 link_H_[3] * cos(r[0] + r[1] + r[2]) + link_H_[3] * cos(r[0] + r[1] + r[2] + r[3]) +
                 link_H_[3] * cos(r[0] + r[1] + r[2] + r[3] + r[4]) +
                 link_H_[4] * cos(r[0] + r[1] + r[2] + r[3] + r[4] + r[5]);
  // x
  transformT(0, 3) = joint[0] + c3 * tempL1;
  // Y
  transformT(1, 3) = joint[1] + s3 * tempL1;
  // Z
  transformT(2, 3) = joint[2] + tempL2;

  return transformT;
}


bool SlimArm::save_RX_TX_Data()
{
  std::vector<std::string> stored_TX_Data = rtde_client_.get_Stored_byteStrList_TX();
  std::vector<std::string> stored_RX_Data = rtde_client_.get_Stored_byteStrList_RX();
  if( stored_TX_Data.size() <= 0 || stored_RX_Data.size() <= 0 ) { 
    LOG_F(WARNING,"[Data Log: RX/TX] Stored Data is empty!");
    return false;
  }

  for (const auto& data : stored_TX_Data) 
    { TX_DATA("{}", data); }
  for (const auto& data : stored_RX_Data) 
    { RX_DATA("{}", data); }

  rtde_client_.clear_Stored_byteStrList_RX();
  rtde_client_.clear_Stored_byteStrList_TX();
  
  return true;
}

 
// // int main() {
// // 先初始化日志系统
// // 每天2:30 am 新建一个日志文件
// // auto logger =
// //     spdlog::daily_logger_mt("daily_logger", "../logs/daily.txt", 2, 30);
// // // 遇到warn flush日志，防止丢失
// // logger->flush_on(spdlog::level::info);
// // logger->flush_on(spdlog::level::warn);
// // logger->flush_on(spdlog::level::err);
// // // spdlog 带行号的打印，同时输出console和文件

// // spdlog::flush_every(std::chrono::seconds(1));

// // spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm TaskControl ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");

// // auto data_logger = spdlog::daily_logger_mt("task_data_logger",
// //                                            "../logs/task_data.txt", 2, 30);
// // // 遇到warn flush日志，防止丢失
// // data_logger->flush_on(spdlog::level::info);
// // data_logger->flush_on(spdlog::level::warn);
// // data_logger->flush_on(spdlog::level::err);
// // // spdlog 带行号的打印，同时输出console和文件

// // spdlog::flush_every(std::chrono::seconds(1));

// // spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
// // RECORD_DATA("globalLocation", 0, 0, 2);
// // ConfigCenter::Instance()->InitConfig();
// // SlimArm arm;
// // arm.Connect();
// // std::array<float,6> joint = {4, 50, 30, 1, 9 / 180.0 * M_PI, 0.1};

// // TransFormMatrix transT = arm.ForwardKinematics(joint);
// // std::array<float,6> jointIK = arm.InverseKinematics(transT);

// // INFO("jointIK = [{}  {}  {}  {}  {}  {}]", jointIK[0], jointIK[1], jointIK[2],
// //      jointIK[3], jointIK[4], jointIK[5]);
// // // m_arm_data_.T_current_ = ForwardKinematics(m_arm_data_.q_current_);
// // std::cout << "transT" << transT << std::endl;
// // print("Joint error", joint - jointIK)
// // print("linkL", arm.linkL)
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm Gunlock ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // arm.Gunlock();
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm GunUnlock ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");

// // arm.GunUnlock();
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm BendDown ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");

// // arm.BendDown();
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm ElongateHold ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // arm.ElongateHold();

// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm GunContract ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // arm.GunContract();

// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // INFO(" robot arm MoveJoint ");
// // INFO("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
// // arm.MoveJoint({1, 1, 1, 1, 1, 1}, false, 4, 5, true);
// // arm.MoveJointDelta({1, 1, 1, 1, 1, 1}, false, 4, 5, true);
 
// // std::string port = CONFIG_ARM_PARAM.arm_port;
// // while (true) {

// //     if (arm.IsConnected()) {
// //         if(kk==1){
// //             // arm.Gunlock();
// //             arm.SetSpeed(HIGH);
// //             kk=0;
// //         }
// //         else{
// //             arm.GunUnlock();
// //             kk=1;
// //         }
// //     }
// //     else {
// //         arm.Connect();
// //     }

// //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

// // }
// //   return 0;
// // }
