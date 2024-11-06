/**
 * Copyright (c) 2023, WissonRobotics
 * File: slim_arm.h
 * Author: Zhiwei Huang (huangzhiwei@wissonrobotics.com)
 * Version 1.0
 * Date: 2023-11-21
 * Brief:
 */
#pragma once

#include <math.h>
#include <string.h>
#include <chrono>
#include <cmath>

#include <functional>
#include <iostream>

#include <thread>


#include "slimArmConfig.h"
#include "SlimArmRTDE.h"
#include "SlimArm_Hardware_Simulator.h"
#include <source_location>
#include <spdlog/fmt/fmt.h>
#include <numeric>
#include <complex>
#include <future>
#include "SlimSerialRTDE/SlimAction.h"

typedef Eigen::Matrix4f TransFormMatrix;
 




/*************************************************** SlimArm Begin******************************************************************** */ 
struct ArmSensorData {
  std::array<int16_t,2> laser_distance_mm= {};
  int16_t distance_guide;
  int16_t distance_gun;
  uint8_t gun_guider_distance;
  PressureUnion pressure;
  std::array<int16_t,1> p_source= {};
  std::array<int16_t,1> p_sink= {};
 
  std::array<uint8_t,8> io_status= {};
  std::array<int16_t,4> rotation_encoder = {};
  std::array<int16_t,3> ultra_sonic= {}; 
  std::array<uint16_t,8> errorList= {};

  PressureUnion pressureCmd;

  TransFormMatrix T_desired_;
  std::array<float,6> q_desired_= {};

  TransFormMatrix T_current_;
  std::array<float,6> q_current_= {};
 
  TransFormMatrix  T_current_exact;
  std::array<float,6> q_current_exact= {};

  std::array<int16_t,6> joint_speed_= {};


  std::array<float,6> tcp_current_;
  std::array<float,6>  tcp_desired_;
};
 

class SlimArm {
 public:
  SlimArm();
  ~SlimArm();
  WS_STATUS Connect();
  WS_STATUS Connect(std::string portname,uint32_t baudrate);
  void Disconnect();
  bool IsConnected() { return rtde_client_.isConnected(); }
  bool isAlive() { return rtde_client_.isAlive(); }
  double getNotAliveTime(){ return rtde_client_.getNotAliveTime()*0.001f;};
  //Action Enable toggle
  void disableActions();//A global switch to disable all the actions. Need to call enableActions() to enable all the actions again. 
  void enableActions();
  inline void StopRecover(){enableActions();}
  

  //Action Start/Stop pneumatic system
  WS_STATUS ValveOn(double timeout=1,bool async=false);
  WS_STATUS ValveOff(double timeout=1,bool async=false);
  
  //Action Magnetic
 
  //Action Heat
  WS_STATUS HeatOn(double timeout=1);
  WS_STATUS HeatOff(double timeout=1);

  //Action Fan
  WS_STATUS FanOn(double timeout=1);
  WS_STATUS FanOff(double timeout=1);
 

  //Action Signal Light
  WS_STATUS SendLightSignalCommand(LightSignalCommand signal_command,double timeout=1);

  //Action Flash Ligh
  WS_STATUS LedLightControl(std::string light_name, int command,double timeout=1);






  //Action Elongate
  WS_STATUS GunElongateByIOFlags(int pressured,bool async=false);
  WS_STATUS GunElongate(bool async=false);// gun move by default elongate pressure command
  WS_STATUS GunContract(bool async=false);// gun move by default contract pressure command
  WS_STATUS GunElongateByPressure(int pressurecmd,bool async=false);// gun move by pressure command

  WS_STATUS GunElongateByLaser(int distanceCmd,bool async=false);//gun elongates until laser sensor is larger than distanceCmd
  WS_STATUS GunContractByLaser(int distanceCmd,bool async=false);//gun contracts until laser sensor is smaller than distanceCmd
  WS_STATUS GunElongateByLaser(bool async=false);//gun elongates until laser sensor is larger than default attached_well_distance
  WS_STATUS GunContractByLaser(bool async=false);//gun contracts until laser sensor is smaller than default detached_well_distance

  WS_STATUS GunElongateByPositionPrecisely(int distanceCmd,bool async=false);

  WS_STATUS GunZeroPressure(bool async=false);

  //Action Guide
  WS_STATUS GuideElongate(bool async=false);
  WS_STATUS GuideContract(bool async=false);
  WS_STATUS GuideElongateByPressure(int16_t pressurecmd,bool async=false);

  WS_STATUS GuideElongateByLaser(bool async=false);//Guide elongates until laser sensor is larger than default align_well_distance
  WS_STATUS GuideContractByLaser(bool async=false);//Guide contracts until laser sensor is smaller than default align_well_distance

  //Action Grasp
  WS_STATUS GripperOpen(bool async=false);
  WS_STATUS GripperClose(bool async=false);

  //Action Shift  
  WS_STATUS GripperShiftCentral(bool async=false);
  WS_STATUS GripperShiftAway(bool async=false);
  WS_STATUS GripperShiftCentralByPressure(int pressurecmd,bool async);
  //Action Lock
  WS_STATUS GunLock(bool async=false);
  WS_STATUS GunUnlock(bool async=false);
 

  //chamber Action base
  WS_STATUS chamberMoveAtPressure(std::string actionName,int pressureIndex,int targetPressure,double timeout=30);
  WS_STATUS chamberMoveAtPosition(std::string actionName,int pressureIndex,int targetPressure,int laserIndex,int targetPosition,bool flagLargerThan,double timeout=30);
  #if SLIM_ARM_TYPE == SlimArm_Type_Monos3
  WS_STATUS chamberMoveAtPositionPrecisely(std::string actionName, int pressureIndex, int maxPressure, int laserIndex, int targetPosition, double timeout);
  #endif
  template<size_t N>
  WS_STATUS chamberMoveAtIOFlags(std::string actionName,int pressureIndex,int targetPressure,std::array<uint8_t,N> &ioIndexes,std::array<uint8_t,N> &ioTargetStates,bool logicAnd,double timeout);
 
   

  //Action Move Pitch
  WS_STATUS MovePitch(float target_pitch, double timeout=40); 
  WS_STATUS MovePitch(std::string actionName,float target_pitch,float pitch_deadzone,float pitch_delta_deadzone,double timeout = 40);

  void MovePitchAsync(float target_pitch,double timeout = 40); 
  void MovePitchAsync(std::string actionName,float target_pitch,float pitch_deadzone,float pitch_delta_deadzone,double timeout = 40 );
  
  WS_STATUS ErectUp(double timeout = 30);
  WS_STATUS BendDown(double timeout = 30);

  void ErectUpAsync(double timeout = 30);
  void BendDownAsync(double timeout = 30);
   

  WS_STATUS PitchHold(bool pitchhold,double timeout=30);
  void setDefaultPitchDeadzone(float pitch_deadzone);
  void setDefaultPitchDeltaDeadzone(float pitch_delta_deadzone);
  WS_STATUS stopMovePitch();//simply call pitch hold


  //Action Move Joint in  Joint Space
  WS_STATUS MoveJoint(std::array<float,6> &desired_joint, double timeout = 30);
  void MoveJointAsync(std::array<float,6> &desired_joint, double timeout = 30);

  WS_STATUS MoveJoint(TransFormMatrix &transform_T, double timeout = 30);
  void MoveJointAsync(TransFormMatrix &transform_T,double timeout = 30);



  WS_STATUS MoveJointPrecisely(std::array<float, 6> &desired_joint, float pitch_deadzone, double timeout = 30);
  void MoveJointPreciselyAsync(std::array<float,6> &desired_joint,float pitch_deadzone, double timeout = 30);
 
  WS_STATUS MoveJointDelta(std::array<float,6> delta_desired_joint, double timeout = 30);
  void MoveJointDeltaAsync(std::array<float,6> delta_desired_joint, double timeout = 30);

  //Action Move Line in Cartesian Space
  WS_STATUS MoveLine(TransFormMatrix &transform_T, double timeout = 30);
  void MoveLineAsync(TransFormMatrix &transform_T,double timeout = 30);

  WS_STATUS stopMoveJoint();
  WS_STATUS MovejointSingle(std::string actionName, int axisIndex, float desired_joint_single, bool isAbsolute = true,double timeout = 30);


  WS_STATUS MoveX(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveY(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveZ(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveR(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveX_Delta(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveY_Delta(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveZ_Delta(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveR_Delta(float desired_joint_single,double timeout = 30);
  WS_STATUS MoveZ_Once_FinishControlPitch(float desired_pitch, float p_coefficient, double timeout = 30);





  WS_STATUS SetSpeed(std::string speed_level, double timeout=5);
  WS_STATUS SetSpeedZ(int16_t z_speed,double timeout=5);
  WS_STATUS SetSpeed(std::string actionName,std::array<int16_t, 6> &speed,double timeout=5);
  inline std::array<int16_t,6> GetSpeed() { return m_arm_data_.joint_speed_; };
 
  bool WithinJointDeadZone(std::array<float,6>& joint_error,float pitchDeadzone);
  bool WithinJointDeadZone(std::array<float,6>& joint_error);
  bool VerifyPoseIsWithin(std::array<float,6> poses);
  bool VerifyJointIsWithin(std::array<float,6> joints);
  
  WS_STATUS MoveLineByIK(
      TransFormMatrix transform_T, bool asynic_flag, int command_repeat_max, double timeout, bool fixed_period);
  WS_STATUS MoveJointByIK(
      TransFormMatrix transform_T, bool asynic_flag, int command_repeat_max, double timeout, bool fixed_period);



 
  inline int16_t getChamberPressure(int pressureIndex){return m_arm_data_.pressure.pressureArray[pressureIndex];};

  std::array<float,6> InverseKinematics(TransFormMatrix &transform_T);
  std::array<float,6> InverseKinematics_test(TransFormMatrix &transform_T);
  TransFormMatrix ForwardKinematics(std::array<float,6> &joint);
  TransFormMatrix ForwardKinematicsExact(std::array<float,6> &joint, std::vector<float> &r);
  
  ArmSensorData &GetArmSensorData() { return m_arm_data_; }
  std::array<float,6> &GetArmCurrentJoint() { return m_arm_data_.q_current_; }
  std::array<float,6> &GetArmDesiredJoint() { return m_arm_data_.q_desired_; }
  TransFormMatrix &GetArmCurrentTransformation() { return m_arm_data_.T_current_; }
  
  std::array<float,6> &GetArmCurrentTCP() { return m_arm_data_.tcp_current_; }
  std::array<float,6> &GetArmDesiredTCP() { return m_arm_data_.tcp_desired_; }
  std::vector<float>  GetArmCurrentJointVector() {std::vector<float> jv;for(int i=0;i<6;i++){jv.emplace_back(m_arm_data_.q_current_[i]);}; return jv; }
  SlimArmRTDE rtde_client_;

  bool save_RX_TX_Data();

	void addLoggingFile(std::string logFileName,std::string logFileLevel);
  
  void updatePitchSteady();
  void updatePitchWithin();
  void updateIOSteady();
  void updatePressureSteady();

  inline bool isSteadyIO(int ioIndex,int targetState){
    return  (targetState==1)?(m_io_steady_count[ioIndex]>=m_io_steady_count_max):(m_io_steady_count[ioIndex]<=-m_io_steady_count_max);
  }
  
  inline int32_t getSteadyPitchCount(float pitch_delta_deadzone){
    if(m_pitch_delta_deadzone != pitch_delta_deadzone){
      m_pitch_delta_deadzone = pitch_delta_deadzone;
      m_pitch_steady_count = 0;
    }
    return m_pitch_steady_count;
  }
  inline int32_t getWithinPitchCount(float pitch_deadzone){
    if(m_pitch_deadzone != pitch_deadzone){
      m_pitch_deadzone = pitch_deadzone;
      m_pitch_within_count = 0;
    }
    return m_pitch_within_count;
  }
  inline bool withinPressureDeadzone(int pressureindex){

    if(pressureindex==6 || pressureindex==7 || pressureindex==8){
      return ((m_arm_data_.pressure.pressureArray[pressureindex]>=m_arm_data_.pressureCmd.pressureArray[pressureindex] && m_arm_data_.pressureCmd.pressureArray[pressureindex]>=0) ||
             (m_arm_data_.pressure.pressureArray[pressureindex]<=m_arm_data_.pressureCmd.pressureArray[pressureindex] && m_arm_data_.pressureCmd.pressureArray[pressureindex]<=0));
    }
    else{
     return std::abs(m_arm_data_.pressure.pressureArray[pressureindex] -m_arm_data_.pressureCmd.pressureArray[pressureindex]) <= m_pressure_steady_deadzone.pressureArray[pressureindex];
    }
  
  }
 
  inline bool isSteadyPressure(int pressureIndex){
    return  (m_pressure_steady_count.pressureArray[pressureIndex]>=m_pressure_steady_count_max)?1:0;
  }
  void configSimulation(bool use_simulation){
    m_using_simulator = use_simulation;
    //make virtual serial ports
     int returnCode = system("socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1 &");
     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // checking if the command was executed successfully
    if (returnCode != 0) {
       LOG_F(ERROR,"[Monos Arm Simulator] [Failed] to create virtual ports");
 
    }
    else{

      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
      LOG_F(INFO,"[Monos Arm Simulator] [Success] to create virtual port /tmp/ttyV0 and /tmp/ttyV1");
    }
  }
  ArmSensorData m_arm_data_;

  bool is_Pitchhold = false;
  bool history_Pitchhold = false;
  std::string arm_port_;
  unsigned int baudrate_;
 
  private:
 

  ArmAction m_actionMoveJoint;
  ArmAction m_actionMovePitch;
  ArmAction m_actionSingleCommand;
  #if SLIM_ARM_TYPE == SlimArm_Type_Default
  std::array<ArmAction,8> m_actionMoveChamber;
  #elif SLIM_ARM_TYPE == SlimArm_Type_Monos3
  std::array<ArmAction,9> m_actionMoveChamber;
  #endif


  void update();
  std::vector<float> LinkH2L(float pitch_angle_rad);

  std::string name_;


  bool m_flagDataUpdated=false;

 

  std::string m_speed_level;
  float m_yaw_zero_offset_;
 
  std::vector<float> link_L_;
  std::vector<float> link_H_;
    // int guide_laser_distance_desired_;
  // int gun_laser_distande_desired_;
  bool m_enable_actions = true;


  //pressure steady check
  PressureUnion m_pressure_steady_deadzone={};
  PressureUnion m_pressure_steady_count={};
  int m_pressure_steady_count_max=20;

  //pitch within check
  bool m_pitch_within_flag=false;
  float m_pitch_deadzone_0degree=0.02;
  float m_pitch_deadzone=0.004;
  float m_pitch_deadzone_default=0.004;
  float m_pitch_within_count=0;
  float m_pitch_within_count_max=50;

  //pitch steady check
  float m_pitch_last=0;
  bool m_pitch_steady_flag=false;
  float m_pitch_delta=0;
  float m_pitch_delta_deadzone=0.001;
  float m_pitch_delta_deadzone_default=0.001;
  int32_t m_pitch_steady_count=0;
  int32_t m_pitch_steady_count_max=100;
 
  //pin and limit steady check

  std::array<int8_t,8> m_io_steady_count={};
  int m_io_steady_count_max = 20;

  bool m_using_simulator=false;
  SlimArm_Hardware_Simulator m_arm_simulator;
};
  