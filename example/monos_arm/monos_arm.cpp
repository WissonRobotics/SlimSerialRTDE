/**
 * Copyright (c) 2023, WissonRobotics
 * File: monos_arm.cpp
 * Author: Zhiwei Huang (huangzhiwei@wissonrobotics.com)
 * Version 1.0
 * Date: 2023-12-06
 * Brief:
 */
#include "monos_arm.h"
#include <fmt/core.h>
#include <fmt/format.h>
#include "monos_arm_config.h"
MonosArm::MonosArm():m_actionSingleCommand(false){

  LOG_F(INFO, "[MonosArm] Created");

  //init data
  arm_port_ = CONFIG_ARM_PARAM.arm_port;
  baudrate_ = CONFIG_ARM_PARAM.arm_baudrate;
  name_ = "MonosArm";
  if (CONFIG_ARM_PARAM.yaw_zero_offset_enable) {
    m_yaw_zero_offset_ = CONFIG_ARM_PARAM.yaw_zero_offset;
  } else {
    m_yaw_zero_offset_ = 0;
  }
  rtde_client_.setYawOffset(m_yaw_zero_offset_);
  LOG_F(INFO, "[MonosArm] setYawOffset %f ", m_yaw_zero_offset_);
  link_H_ = {0.01154, 0.42953, 0.055, 0.0595, 0.4225};
  link_L_ = LinkH2L(0);


  memset((uint8_t *)(&m_arm_data_),0,sizeof(m_arm_data_));
  m_arm_data_.T_current_ = ForwardKinematics(m_arm_data_.q_current_);
  m_arm_data_.T_desired_ = ForwardKinematics(m_arm_data_.q_desired_);
 
  //register callback
  rtde_client_.setUpdateCallback([this](){update();});
 
  m_pressure_steady_deadzone.Ppitch_inner = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Ppitch_outer = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pelongate = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pguide = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pshift = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Pgrasp = 50; // 50HPa deadzone
  m_pressure_steady_deadzone.Plock = 50; // 50HPa deadzone 
 
}

MonosArm::~MonosArm(){

}
WS_STATUS MonosArm::Connect() {
  return Connect(arm_port_, baudrate_);
}

WS_STATUS MonosArm::Connect(std::string portname,uint32_t baudrate) {
  LOG_F(INFO, "[MonosArm] connect to port %s  baudrate %d", arm_port_.c_str(), baudrate_);
  if (rtde_client_.connect(portname, baudrate)==WS_OK) {
  
    m_flagDataUpdated = false;
   
    int tt=0;
    while(!isAlive()){
      if(tt++>2){
        LOG_F(WARNING,"[MonosArm]  port %s  is not responding",arm_port_.c_str());
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

    setDefaultPitchDeadzone(0.082);
    setDefaultPitchDeltaDeadzone(0.001);

    // this->SendLightSignalCommand(LightSignalCommand::LightYellowOn);

    return WS_STATUS::WS_OK;
  } else {
    return WS_STATUS::WS_FAIL;
  }
}

void MonosArm::Disconnect() {
  if (IsConnected()) {
    rtde_client_.disconnect();
  }
}

void MonosArm::enableActions() {
  m_enable_actions = true;
  m_actionMoveJoint.enable();
  m_actionMovePitch.enable();
  m_actionSingleCommand.enable();
  for(auto & action:m_actionMoveChamber){
    action.enable();
  }
}


void MonosArm::disableActions() {
  m_enable_actions = false;
  m_actionMoveJoint.disable();
  m_actionMovePitch.disable();
  m_actionSingleCommand.disable();
  for(auto & action:m_actionMoveChamber){
    action.disable();
  }
}


 



WS_STATUS MonosArm::ValveOn(double timeout){
   return m_actionSingleCommand.act( "ValveOn",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition.
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandStart, &rtde_client_)
                        );
}
 
WS_STATUS MonosArm::ValveOff(double timeout) {
  return m_actionSingleCommand.act( "ValveOff",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandStop, &rtde_client_)
                        );
}



WS_STATUS MonosArm::MagnetOn(double timeout){
   int16_t command_value = 1;
   return m_actionSingleCommand.act( "MagnetOn",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandMagnet, &rtde_client_, std::placeholders::_1),
                        command_value
                        );
}
 
WS_STATUS MonosArm::MagnetOff(double timeout) {
   int16_t command_value = 0;
   return m_actionSingleCommand.act( "MagnetOff",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandMagnet, &rtde_client_, std::placeholders::_1),
                        command_value
                        );
}

 
 
WS_STATUS MonosArm::SendLightSignalCommand(LightSignalCommand signal_command,double timeout) {
   return m_actionSingleCommand.act(rtde_client_.getSignalLightName(signal_command),
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandSignalLight, &rtde_client_, std::placeholders::_1),
                        signal_command
                        );
}

WS_STATUS MonosArm::LedLightControl(std::string light_name, int light_command,double timeout){
  std::string actionName = "Light_" + light_name + (light_command==0?"_Off":"_On");
   return m_actionSingleCommand.act(actionName,
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandLight, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                        light_name,
                        light_command
                        );  
}
 

bool MonosArm::VerifyPoseIsWithin(std::array<float,6> poses) {
  // TODO
  return true;
}
bool MonosArm::VerifyJointIsWithin(std::array<float,6> joints) {
  // TODO
  return true;
}
 




 
 
//Action Elongate
WS_STATUS MonosArm::GunElongate(double timeout){
  return chamberMoveAtPosition("GunElongate",IndexPressureElongate,CONFIG_ARM_PARAM.gun_elongate_value,0,CONFIG_GUN_PARAM.attached_well_distance,true,timeout);
}
WS_STATUS MonosArm::GunContract(double timeout){
  return chamberMoveAtPosition("GunContract",IndexPressureElongate,CONFIG_ARM_PARAM.gun_contract_value,0,CONFIG_GUN_PARAM.detached_well_distance,false,timeout);
}

//Action guide
WS_STATUS MonosArm::GuideElongate(double timeout){
  return chamberMoveAtPosition("GuideElongate",IndexPressureGuide,CONFIG_ARM_PARAM.guide_elongate_value,0,CONFIG_GUN_PARAM.alignment_well_distance,true,timeout);
}
WS_STATUS MonosArm::GuideContract(double timeout){
  return chamberMoveAtPosition("GuideContract",IndexPressureGuide,CONFIG_ARM_PARAM.guide_contract_value,0,CONFIG_GUN_PARAM.alignment_laser_original,false,timeout);
}

//Action grasp 
WS_STATUS MonosArm::GripperOpen(double timeout){
  return chamberMoveAtPressure("GripperOpen",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_open_value,timeout);
}

WS_STATUS MonosArm::GripperClose(double timeout){
  return chamberMoveAtPressure("GripperClose",IndexPressureGrasp,CONFIG_ARM_PARAM.gripper_close_value,timeout);
}
 
 
//Action shift 
WS_STATUS MonosArm::GripperShiftCentral( double timeout){
    std::array<uint8_t,2> ioIndexes={2,3};
    std::array<uint8_t,2> ioTargetStates{0,1};
    return chamberMoveAtIOFlags("GripperShiftAway",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_central_value,ioIndexes,ioTargetStates,timeout);
}

WS_STATUS MonosArm::GripperShiftAway(double timeout){
  std::array<uint8_t,2> ioIndexes={2,3};
  std::array<uint8_t,2> ioTargetStates{1,0};
  return chamberMoveAtIOFlags("GripperShiftAway",IndexPressureShift,CONFIG_ARM_PARAM.gripper_shift_away_value,ioIndexes,ioTargetStates,timeout);
}
 
//Action Lock
WS_STATUS MonosArm::GunLock(double timeout){
  std::array<uint8_t,1> ioIndexes={0};
  std::array<uint8_t,1> ioTargetStates{0};
  //return chamberMoveAtIOFlags("GunLock",IndexPressureLock,CONFIG_ARM_PARAM.gun_lock_value,ioIndexes,ioTargetStates,timeout);
  WS_STATUS ret = chamberMoveAtPressure("GunLock",IndexPressureLock,CONFIG_ARM_PARAM.gun_lock_value,timeout);
  return (m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0])?WS_OK:WS_FAIL;
 
}
 
WS_STATUS MonosArm::GunUnlock(double timeout){
  std::array<uint8_t,1> ioIndexes={0};
  std::array<uint8_t,1> ioTargetStates{1};
  // return chamberMoveAtIOFlags("GunUnlock",IndexPressureLock,CONFIG_ARM_PARAM.gun_unlock_value,ioIndexes,ioTargetStates,timeout);
  auto ret = chamberMoveAtPressure("GunUnlock",IndexPressureLock,CONFIG_ARM_PARAM.gun_unlock_value,timeout);
  return (m_arm_data_.io_status[ioIndexes[0]]==ioTargetStates[0])?WS_OK:WS_FAIL;
}



WS_STATUS MonosArm::chamberMoveAtPressure(std::string actionName,int pressureIndex,int targetPressure,double timeout){
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
                          std::bind(&Maxwell_SoftArm_SerialClient::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                          pressureIndex,
                          targetPressure
                          );
}

WS_STATUS MonosArm::chamberMoveAtPosition(std::string actionName,int pressureIndex,int targetPressure,int laserIndex,int targetPosition,bool flagLargerThan,double timeout){
  
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
                              progress.messageFrequencyDivide = 10; //slower message 
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
                          std::bind(&Maxwell_SoftArm_SerialClient::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                          pressureIndex,
                          targetPressure
                          );
}  
template<size_t N>
WS_STATUS MonosArm::chamberMoveAtIOFlags(std::string actionName,int pressureIndex,int targetPressure,std::array<uint8_t,N> &ioIndexes,std::array<uint8_t,N> &ioTargetStates,double timeout){
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
                        [&,pressureIndex,targetPressure,ioIndexes,ioTargetStates](ArmActionProgress &progress){//complete condition
                            progress.messageFrequencyDivide = 20; //slower message 
                            progress.message = fmt::format("Current pressure:{} ,",getChamberPressure(pressureIndex));
                            progress.message.append(fmt::format("Desired/Current "));
                            for(size_t i=0;i<N;i++){
                              progress.message.append(fmt::format("IO_status[{}]={}/{} ",ioIndexes[i],ioTargetStates[i],m_arm_data_.io_status[ioIndexes[i]]));
                            }
                            
                            bool ioResult = true; 
                            for(size_t i=0;i<N;i++){ 
                              ioResult &= (m_arm_data_.io_status[ioIndexes[i]]==ioTargetStates[i]);
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
                        std::bind(&Maxwell_SoftArm_SerialClient::commandChamber, &rtde_client_, std::placeholders::_1, std::placeholders::_2),
                        pressureIndex,
                        targetPressure
                        );
}









WS_STATUS MonosArm::MoveJointPrecisely(std::array<float,6> &desired_joint,float pitch_deadzone, double timeout){
  return m_actionMoveJoint.act(
          "MoveJointPrecisely",
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
              progress.messageFrequencyDivide = 1;//faster message 
  
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
              progress.messageFrequencyDivide = 10; //slower message 
              progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                                m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                                q_error[4], q_error[5]);
            }
            return false;
          },
          timeout, 
          std::bind(&Maxwell_SoftArm_SerialClient::commandJoint, &rtde_client_, std::placeholders::_1),
          desired_joint);
}

void MonosArm::MoveJointPreciselyAsync(std::array<float,6> &desired_joint,float pitch_deadzone, double timeout){
 
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


WS_STATUS MonosArm::MoveJoint(TransFormMatrix &transform_T, double timeout) {
   
    m_arm_data_.T_desired_ = transform_T;
    auto q_desired = InverseKinematics(m_arm_data_.T_desired_);

    return MoveJoint(q_desired,timeout);
 
}



WS_STATUS MonosArm::MoveJoint(std::array<float,6> &desired_joint, double timeout){

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
            }
            else{
              progress.message =  fmt::format("q_desired overlimit:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_desired_[0], m_arm_data_.q_desired_[1], m_arm_data_.q_desired_[2], m_arm_data_.q_desired_[3], m_arm_data_.q_desired_[4], m_arm_data_.q_desired_[5]);
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
              progress.messageFrequencyDivide = 1;//faster message 
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
              progress.messageFrequencyDivide = 10; //slower message 
              progress.message = fmt::format("q_current:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}] error:[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]", m_arm_data_.q_current_[0], m_arm_data_.q_current_[1],
                                m_arm_data_.q_current_[2], m_arm_data_.q_current_[3], m_arm_data_.q_current_[4], m_arm_data_.q_current_[5], q_error[0], q_error[1], q_error[2], q_error[3],
                                q_error[4], q_error[5]);
            }
            return false;
          },
          timeout, 
          std::bind(&Maxwell_SoftArm_SerialClient::commandJoint, &rtde_client_, std::placeholders::_1),
          desired_joint);
}

void MonosArm::MoveJointAsync(std::array<float,6> &desired_joint, double timeout){
  m_actionMoveJoint.actionThread = std::make_unique<std::jthread>([&,timeout](){
      MoveJoint(desired_joint,timeout);
    }
  );
}


WS_STATUS MonosArm::MoveJointDelta(std::array<float,6> delta_desired_joint, double timeout){
  std::array<float,6> desired_joint = delta_desired_joint + m_arm_data_.q_current_;
  return MoveJoint(desired_joint,timeout);
}
void MonosArm::MoveJointDeltaAsync(std::array<float,6> delta_desired_joint, double timeout){
  std::array<float,6> desired_joint = delta_desired_joint + m_arm_data_.q_current_;
  m_actionMoveJoint.actionThread = std::make_unique<std::jthread>([&,timeout](){
      MoveJoint(desired_joint,timeout);
    }
  );
}


WS_STATUS MonosArm::stopMoveJoint() {
  LOG_F(INFO, "[MonosArm] Stop MoveJoint");
  if(MoveJoint(this->GetArmCurrentJoint())!=WS_OK){
    //shouldn't be here
    m_actionMoveJoint.stopAction();
    return WS_FAIL;
  }
  else{
    return WS_OK;
  }
}



WS_STATUS MonosArm::SetSpeed(std::string actionName,std::array<int16_t, 6> &speed,double timeout) {
return m_actionSingleCommand.act( actionName,
                        [&](ArmActionProgress &progress){
                          if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}
                          progress.message = fmt::format("target speed:[{} {} {} {} {} {}]",speed[0],speed[1],speed[2],speed[3],speed[4],speed[5]);
                          return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition. 
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition. 
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandVelocity, &rtde_client_, std::placeholders::_1),
                        speed
                        );
}

WS_STATUS MonosArm::SetSpeedZ(int16_t z_speed,double timeout) {
  m_arm_data_.joint_speed_[2] = z_speed;
  return SetSpeed("Set Speed Z",m_arm_data_.joint_speed_,timeout);
}

WS_STATUS MonosArm::SetSpeed(std::string speed_level, double timeout) {
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
}
 

 WS_STATUS MonosArm::MovePitch(float target_pitch, double timeout) {
  if(target_pitch==0){
    return MovePitch("MovePitch",target_pitch,m_pitch_deadzone_0degree, m_pitch_delta_deadzone_default,timeout);
  }
  else{
    return MovePitch("MovePitch",target_pitch,m_pitch_deadzone_default, m_pitch_delta_deadzone_default,timeout);
  }
  
}

 

WS_STATUS MonosArm::MovePitch(std::string actionName,float pitch_desired,float pitch_deadzone,float pitch_delta_deadzone,double timeout){
 
  return m_actionMovePitch.act(
          actionName,
          [&](ArmActionProgress &progress){//start condition
              if(!isAlive()){
                progress.message = fmt::format("Monos Arm is not responding");
                return false;
              }
              m_arm_data_.q_desired_[4] = pitch_desired;
  
              progress.message =  fmt::format("pitch_desired: {:.4f}", m_arm_data_.q_desired_[4]);
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
              progress.messageFrequencyDivide = 10;//  message   fre
              if(pitch_within_count >m_pitch_within_count_max){

                
                progress.message = fmt::format("q_current: {:.4f}  error: {:.4f}  within:{} steady:{}",m_arm_data_.q_current_[4],q_error ,pitch_within_count,pitch_steady_count);
                //complete condition
                if(pitch_steady_count>m_pitch_steady_count_max){
                  progress.percentage = 100;
                  return true;
                }
              }
              else{
                progress.message = fmt::format("pitch_current:{:.4f} error:{:.4f} within:{}", m_arm_data_.q_current_[4],q_error, pitch_within_count);
              }
            }
            else{
              progress.messageFrequencyDivide = 20; //slower message 
              progress.message = fmt::format("pitch_current:{:.4f} error:{:.4f}", m_arm_data_.q_current_[4],q_error);
            }
            return false;
          },
          timeout, 
          std::bind(&Maxwell_SoftArm_SerialClient::commandPitch, &rtde_client_, std::placeholders::_1),
          pitch_desired);
}

void MonosArm::MovePitchAsync(float target_pitch,double timeout){
  LOG_F(INFO, "[MonosArm] MovePitchAsync");
  if(target_pitch==0){
    MovePitchAsync("MovePitchAsync",target_pitch,m_pitch_deadzone_0degree, m_pitch_delta_deadzone_default,timeout);
  }
  else{
    MovePitchAsync("MovePitchAsync",target_pitch,m_pitch_deadzone_default, m_pitch_delta_deadzone_default,timeout);
  }
}
 

void MonosArm::MovePitchAsync(std::string actionName,float pitch_desired,float pitch_deadzone,float pitch_delta_deadzone,double timeout){

  m_actionMovePitch.actionThread = std::make_unique<std::jthread>([&,actionName,pitch_desired,pitch_deadzone,pitch_delta_deadzone,timeout](){
      MovePitch(actionName,pitch_desired,pitch_deadzone,pitch_delta_deadzone,timeout);
    }
  );
}


WS_STATUS MonosArm::ErectUp(double timeout){
  return MovePitch("ErectUp",0,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}
void MonosArm::ErectUpAsync(double timeout){
    MovePitchAsync("ErectUpAsync",0,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}
 

WS_STATUS MonosArm::BendDown(double timeout){
  return MovePitch("BendDown",M_PI_2,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}
void MonosArm::BendDownAsync(double timeout){
     MovePitchAsync("BendDownAsync",M_PI_2,m_pitch_deadzone,m_pitch_delta_deadzone,timeout);
}


WS_STATUS MonosArm::PitchHold(bool pitchhold,double timeout){
  return m_actionMovePitch.act( "PitchHold",
                        [&](ArmActionProgress &progress){if(!isAlive()){progress.message = fmt::format("Monos Arm is not responding");return false;}return true;}, //start condition. 
                        [&](ArmActionProgress &progress){return false;}, //stop condition.
                        [&](ArmActionProgress &progress){return false;}, //progress indication
                        [&](ArmActionProgress &progress){progress.percentage = 100;return true;},//complete condition.  
                        timeout, 
                        std::bind(&Maxwell_SoftArm_SerialClient::commandPitchHold, &rtde_client_,std::placeholders::_1),
                        pitchhold
                        );
}

 void MonosArm::setDefaultPitchDeadzone(float pitch_deadzone){
   m_pitch_deadzone_default = pitch_deadzone;
 }
 void MonosArm::setDefaultPitchDeltaDeadzone(float pitch_delta_deadzone){
   m_pitch_delta_deadzone_default = pitch_delta_deadzone;
 }

WS_STATUS MonosArm::stopMovePitch(){
  LOG_F(INFO, "[MonosArm] Stop MovePitch");
  return PitchHold(true);
}


bool MonosArm::WithinJointDeadZone(std::array<float,6> &joint_error) {

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


bool MonosArm::WithinJointDeadZone(std::array<float,6> &joint_error,float pitchDeadzone) {
 
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

  
 
 
void MonosArm::updatePitchSteady(){
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

void MonosArm::updatePitchWithin(){
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



void MonosArm::addLoggingFile(std::string logFileName,std::string logFileLevel){
    rtde_client_.addLoggingFile(logFileName,logFileLevel);
}


void MonosArm::update(){

      //update sensor data from serial RTDE
      m_arm_data_.laser_distance_mm = rtde_client_.sensorData.laserDistance;
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
}

 
  void MonosArm::updateIOSteady(){
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
 
  
  void MonosArm::updatePressureSteady(){
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
 
 
std::vector<float> MonosArm::LinkH2L(float pitch_angle_rad) {
  float H5 = link_H_[3] * (1 + 2 * (cos(pitch_angle_rad / 3.0) + cos(pitch_angle_rad / 6.0)));
  float Lr = H5 / 2.0 / cos(pitch_angle_rad / 2.0);
  std::vector<float> link = {link_H_[0], link_H_[1], link_H_[2] + Lr, link_H_[4] + Lr};
  return link;
}
std::array<float,6> MonosArm::InverseKinematics(TransFormMatrix &transform_T) {
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

std::array<float,6> MonosArm::InverseKinematics_test(TransFormMatrix &transform_T) {
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
TransFormMatrix MonosArm::ForwardKinematics(std::array<float,6> &joint) {
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

TransFormMatrix MonosArm::ForwardKinematicsExact(std::array<float,6> &joint, std::vector<float> &r) {
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