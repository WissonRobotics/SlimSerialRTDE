/**
 * Copyright (c) 2023, WissonRobotics
 * File: monos_arm.h
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
#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>
#include <queue>
#include <thread>
#include <typeinfo>
#include <fmt/format.h>
 #include <numeric>
 #include <complex>
#include <future>
#include "eigen3/Eigen/Eigen"
 
#include "Maxwell_SoftArm_SerialClient.h"
#include <source_location>
#include "loguru.hpp" 
typedef Eigen::Matrix4f TransFormMatrix;
 

inline std::chrono::_V2::steady_clock::time_point TIC(){
    return std::chrono::steady_clock::now();
}
inline double TOC(std::chrono::_V2::steady_clock::time_point start_time_point){
    return (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time_point)).count();
}


/*************************************************** Function repeat Begin******************************************************************** */
template <typename Func, typename... Args>
WS_STATUS command_repeat(WS_STATUS expect_return_value, double timeout_max, int repeat_max, Func&& func, Args&&... args) {
 
  auto func_start_time = TIC();
  int repeat_times=0;
 
  while (true) {

    repeat_times++;
     
    //auto result = func(std::forward<Args>(args)...);
    auto result = std::invoke(std::forward<Func>(func),std::forward<Args>(args)...);
    auto nextTick = TIC() + std::chrono::milliseconds(defaultCommandTimeoutPeriodMs);
 
    if (result == expect_return_value) {
      return WS_OK;
    }  
 
    if (repeat_max != 0 && repeat_times >= repeat_max) {
      LOG_F(ERROR,"[Command_repeat Timeout] %s exceed %d times ",std::source_location::current().function_name(),repeat_max);
      return WS_TIMEOUT;
    }

    if (timeout_max != 0) {
      if (TOC(func_start_time) > timeout_max) {
        LOG_F(ERROR,"[Command_repeat Timeout] %s timeout for %f ms",std::source_location::current().function_name(),timeout_max);
        return WS_TIMEOUT;
      }
    }

    std::this_thread::sleep_until(nextTick);
  }
}



/*************************************************** Arm Action Begin******************************************************************** */
struct ArmActionResult{
  WS_STATUS action_result;
  double action_time_cost;
};
struct ArmActionProgress{
  int conditionCheckNumber;
  float progressIndicator;
  float progressIndicatorOriginal;
  int percentage;
  std::string  message;
  int  messageFrequencyDivide;
  int commandTimes;
  double  timeCost;
  int tempDataInt[5];
  double tempDataDouble[3];
 
};

class ArmAction{
public:
  ArmAction(bool preemptible=true)
  :m_preemptible(preemptible),
  m_timeout(60),
  m_command_repeat_max(50){
    reset();
  }

  void setActionName(std::string _name){
    m_actionName = _name;
  }
 
  void setActionCompleteCallback(std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback){
    if(actionCompleteCallback){
      m_actionCompleteCallback =  std::forward<std::function<void(WS_STATUS &actionResult)>>(actionCompleteCallback);
    }
  }
 
  template <typename Func, typename... Args>
  WS_STATUS act(std::string actionName,std::function<bool(ArmActionProgress &progress)> &&actionStartCondition,std::function<bool(ArmActionProgress &progress)> &&actionStopCondition,std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition,double timeout_max, Func&& actionFunc, Args&&... actionArgs){
    LOG_F(INFO, "Enter Action");
    std::unique_lock lock_(m_actionMtx);
    if(m_preemptible){
      lock_.unlock();
    }
    LOG_F(INFO, "[Action %s] [Init]",actionName.c_str());

    if(!isStopped()){

      stopAction();
      LOG_F(WARNING, "[Action %s] [Preempt] A previous action %s is still running, stopping it now...",actionName.c_str(),m_actionName.c_str());
      
      int timeoutSeconds = 5;
      while(spinWait([&](){return isStopped();},1,10)!=WS_OK){

        stopAction();
        LOG_F(WARNING, "[Action %s] [Preempt] A previous action %s is still running, stopping it now...",actionName.c_str(),m_actionName.c_str());
        if(timeoutSeconds--<=0){
          break;
        }
      }

      if(isStopped()){
        LOG_F(1, "[Action %s] [Preempt] Successfully stopped previous action %s, going to perform current action anyway",actionName.c_str(),m_actionName.c_str());
      }
      else{
        LOG_F(WARNING, "[Action %s] [Preempt] Fail to stop previous action %s, going to perform current action anyway",actionName.c_str(),m_actionName.c_str());
      }
    }

    reset();

    m_actionName = actionName; 
  
    m_idleFlag = false;

    m_timeout = timeout_max;
    //optional store the functions
    // m_function = std::forward<Func>(actionFunc);
    // m_args = std::forward<Args>(actionArgs)...;
    m_actionStartCondition =  std::forward<std::function<bool(ArmActionProgress &progress)>>(actionStartCondition);
    m_actionStopCondition =  std::forward<std::function<bool(ArmActionProgress &progress)>>(actionStopCondition);
    m_actionProgressIndication = std::forward<std::function<bool(ArmActionProgress &progress)>>(actionProgressIndication);
    m_actionCompleteCondition =  std::forward<std::function<bool(ArmActionProgress &progress)>>(actionCompleteCondition);

 
    //fast fail  
    if(!m_actionEnabled){
      LOG_F(ERROR, "[Action %s] [Abort] Action is disabled",m_actionName.c_str());
      m_idleFlag = true;
      m_actionResult = WS_STATUS::WS_FAIL;
      if(m_actionCompleteCallback){
        m_actionCompleteCallback(m_actionResult);
      }
      return m_actionResult;
    }


    //fast fail for start pre condition
    if (!m_actionStartCondition(m_progress)){
      LOG_F(ERROR, "[Action %s] [Abort] %s",m_actionName.c_str(),m_progress.message.c_str());
      m_idleFlag = true;
      m_actionResult = WS_STATUS::WS_FAIL;
      if(m_actionCompleteCallback){
        m_actionCompleteCallback(m_actionResult);
      }
      return m_actionResult;
    }
    else{
      LOG_F(INFO, "[Action %s] [Start] %s",m_actionName.c_str(),m_progress.message.c_str());
    }


    auto action_start_time = TIC();
    
    WS_STATUS func_ret = command_repeat(WS_STATUS::WS_OK, 0, m_command_repeat_max, std::forward<Func>(actionFunc), std::forward<Args>(actionArgs)...);
 
    if (func_ret != WS_STATUS::WS_OK) {
      m_progress.timeCost = TOC(action_start_time);
      LOG_F(ERROR, "[Action %s] [Failed] [%f s] commanding timeout over %d times ",m_actionName.c_str(),m_progress.timeCost,m_command_repeat_max);
      m_actionResult = WS_STATUS::WS_FAIL;
    }
    else{
      LOG_F(2, "[Action %s] [Command] [OK]",m_actionName.c_str());
      while(true){
        auto nextTick = TIC() + std::chrono::milliseconds(defaultFrameTxRxPeriodMs);
        m_progress.timeCost = TOC(action_start_time);
        
        //timeout
        if (m_progress.timeCost > m_timeout) {
          LOG_F(ERROR, "[Action %s] [Timeout] [%f s]",m_actionName.c_str(),m_progress.timeCost);
          m_actionResult = WS_STATUS::WS_TIMEOUT;
          break;
        }
 
        //external stop condition
        if (m_actionStopCondition(m_progress)) {
          LOG_F(WARNING, "[Action %s] [Stopped] [%f s] %s",m_actionName.c_str(),m_progress.timeCost,m_progress.message.c_str());
          m_actionResult = WS_STATUS::WS_ABORT;
          break;
        }

        //internal stop request
        if (m_stopRequest || (!m_actionEnabled)) {
          LOG_F(WARNING, "[Action %s] [Stopped] [%f s] Stop command is received",m_actionName.c_str(),m_progress.timeCost);
          m_stopRequest=false;
          m_actionResult = WS_STATUS::WS_ABORT;
          break;  
        }

        //progress indication
        if(m_actionProgressIndication(m_progress)){
          if(m_progress.conditionCheckNumber==0){
            m_progress.progressIndicatorOriginal = m_progress.progressIndicator;
            if(m_progress.progressIndicatorOriginal==0){
              m_progress.progressIndicatorOriginal=0.000000001;
            }
          }
          m_progress.percentage = (int)(100*(1-m_progress.progressIndicator/m_progress.progressIndicatorOriginal));
        }

        if(m_progress.conditionCheckNumber % m_progress.messageFrequencyDivide  ==0){
          LOG_F(INFO, "[Action %s] [Progress %d] [%f s] %s",m_actionName.c_str(),m_progress.percentage,m_progress.timeCost,m_progress.message.c_str());
        }

        //complete condition
        if (m_actionCompleteCondition(m_progress)){
          LOG_F(INFO, "[Action %s] [Progress %d] [%f s] %s",m_actionName.c_str(),m_progress.percentage,m_progress.timeCost,m_progress.message.c_str());
          LOG_F(INFO, "[Action %s] [Successful]",m_actionName.c_str());
          m_actionResult = WS_STATUS::WS_OK;
          break;
        }

        

        m_progress.conditionCheckNumber++;
        std::this_thread::sleep_until(nextTick);
      }
    } 
    m_idleFlag = true;
    if(m_actionCompleteCallback){
      m_actionCompleteCallback(m_actionResult);
    }
    return m_actionResult;
  }

  void reset(){
    m_progress.commandTimes = 0;
    m_progress.conditionCheckNumber = 0;
    m_progress.message = "";
    m_progress.messageFrequencyDivide = 1;
    m_progress.percentage = 0;
    m_progress.timeCost = 0;
    for(auto &item:m_progress.tempDataInt){
      item = 0;
    }
    for(auto &item:m_progress.tempDataDouble){
      item = 0;
    }
  }

  void disable(){
    m_actionEnabled = false;
  }

  void enable(){
    m_actionEnabled = true;
  }

  void stopAction(){
    m_stopRequest = true;
  }

  bool isStopped(){
    return m_idleFlag;
  }
 

 WS_STATUS spinWait(std::function<bool()> &&waitCondition,int timeoutMS,int intervalMS=0) {
	auto timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMS);
	while(true){
		if(waitCondition()){
			return WS_OK;
		}
		if(std::chrono::steady_clock::now()>timeoutPoint){
			return WS_TIMEOUT;
		}
		if(intervalMS>0){
			std::this_thread::sleep_for(std::chrono::milliseconds(intervalMS));
		}
	}
}

  void setPreemptible(bool preempt){
    m_preemptible = preempt;
  }
 
  std::unique_ptr<std::jthread> actionThread;
private:
  bool m_preemptible = true;
  double m_timeout;
  int m_command_repeat_max;  

  std::string m_actionName;
  // std::function<std::invoke_result_t<Func, Args...>> m_function;
  // std::tuple<Args...> m_args;
  std::function<bool(ArmActionProgress &progress)> m_actionStartCondition;
  std::function<bool(ArmActionProgress &progress)> m_actionStopCondition;
  std::function<bool(ArmActionProgress &progress)> m_actionCompleteCondition;
  std::function<bool(ArmActionProgress &progress)> m_actionProgressIndication;
  std::function<void(WS_STATUS &actionResult)> m_actionCompleteCallback;
  ArmActionProgress m_progress;
  WS_STATUS m_actionResult;
 


  bool m_stopRequest=false;
  bool m_idleFlag=true;
  
  bool m_actionEnabled=true;
  std::mutex m_actionMtx;
  
};



/*************************************************** MonosArm Begin******************************************************************** */ 
struct ArmSensorData {
  std::array<int16_t,2> laser_distance_mm= {};
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

};
 

class MonosArm {
 public:
  MonosArm();
  ~MonosArm();
  WS_STATUS Connect();
  WS_STATUS Connect(std::string portname,uint32_t baudrate);
  void Disconnect();
  inline bool IsConnected() { return rtde_client_.isConnected(); }

  //Action Enable toggle
  void disableActions();//A global switch to disable all the actions. Need to call enableActions() to enable all the actions again. 
  void enableActions();
  inline void StopRecover(){enableActions();}
  

  //Action Start/Stop pneumatic system
  WS_STATUS ValveOn(double timeout=1);
  WS_STATUS ValveOff(double timeout=1);
  
  //Action Magnetic
  WS_STATUS MagnetOn(double timeout=1);
  WS_STATUS MagnetOff(double timeout=1);

  //Action Signal Light
  WS_STATUS SendLightSignalCommand(LightSignalCommand signal_command,double timeout=1);

  //Action Flash Ligh
  WS_STATUS LedLightControl(std::string light_name, int command,double timeout=1);



  //Action Elongate
  WS_STATUS GunElongate(double timeout=30);
  WS_STATUS GunContract(double timeout=30);
 
  //Action Guide
  WS_STATUS GuideElongate(double timeout=30);
  WS_STATUS GuideContract(double timeout=30);
 
  //Action Grasp
  WS_STATUS GripperOpen(double timeout = 10);
  WS_STATUS GripperClose(double timeout = 10);

  //Action Shift  
  WS_STATUS GripperShiftCentral(double timeout=20);
  WS_STATUS GripperShiftAway(double timeout=20);

  //Action Lock
  WS_STATUS GunLock(double timeout = 5);
  WS_STATUS GunUnlock(double timeout = 5);
 

  //chamber Action base
  WS_STATUS chamberMoveAtPressure(std::string actionName,int pressureIndex,int targetPressure,double timeout=30);
  WS_STATUS chamberMoveAtPosition(std::string actionName,int pressureIndex,int targetPressure,int laserIndex,int targetPosition,bool flagLargerThan,double timeout=30);
  template<size_t N>
  WS_STATUS chamberMoveAtIOFlags(std::string actionName,int pressureIndex,int targetPressure,std::array<uint8_t,N> &ioIndexes,std::array<uint8_t,N> &ioTargetStates,double timeout);
  

 
  // ArmFuncReturn ElongateHold();

  // ArmFuncReturn OpenDoor();
  // ArmFuncReturn CloseDoor();


  //Action Move Pitch

  WS_STATUS MovePitch(float target_pitch, double timeout=30);
  void MovePitchAsync(float target_pitch,double timeout = 30);

  WS_STATUS MovePitch(std::string actionName,float target_pitch,float pitch_deadzone,float pitch_delta_deadzone,double timeout = 30);
  void MovePitchAsync(std::string actionName,float target_pitch,float pitch_deadzone,float pitch_delta_deadzone,double timeout = 30 );
  
  WS_STATUS ErectUp(double timeout = 30);
  WS_STATUS BendDown(double timeout = 30);

  void ErectUpAsync(double timeout = 30);
  void BendDownAsync(double timeout = 30);
   

  WS_STATUS PitchHold(bool pitchhold,double timeout=30);
  void setPitchDeadzone(float pitch_deadzone);
  void setPitchDeltaDeadzone(float pitch_delta_deadzone);
  WS_STATUS stopMovePitch();//simply call pitch hold


  //Action Move Joint in  Joint Space
  WS_STATUS MoveJoint(std::array<float,6> &desired_joint, double timeout = 30);
  void MoveJointAsync(std::array<float,6> &desired_joint, double timeout = 30,std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback=NULL);

  WS_STATUS MoveJoint(TransFormMatrix &transform_T, double timeout);
  void MoveJointAsync(TransFormMatrix &transform_T,double timeout = 30,std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback=NULL);



  WS_STATUS MoveJointPrecisely(std::array<float, 6> &desired_joint, float pitch_deadzone, double timeout = 30);
  void MoveJointPreciselyAsync(std::array<float,6> &desired_joint,float pitch_deadzone, double timeout = 30,std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback=NULL);
 
  WS_STATUS MoveJointDelta(std::array<float,6> &desired_joint, double timeout = 30);
  void MoveJointDeltaAsync(std::array<float,6> &desired_joint, double timeout = 30,std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback=NULL);

  //Action Move Line in Cartesian Space
  WS_STATUS MoveLine(TransFormMatrix &transform_T, double timeout);
  void MoveLineAsync(TransFormMatrix &transform_T,double timeout = 30,std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback=NULL);

  WS_STATUS stopMoveJoint();

  WS_STATUS SetSpeed(std::string speed_level, double timeout=5);

  WS_STATUS SetSpeedZ(int16_t z_speed, double timeout = 5);
  WS_STATUS SetSpeed(std::string actionName,std::array<int16_t, 6> &speed,double timeout=5);
  inline std::array<int16_t,6> GetSpeed() { return m_arm_data_.joint_speed_; };
  

  bool WithinJointDeadZone(std::array<float,6>&,float pitchDeadzone=-1);
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
  
  Maxwell_SoftArm_SerialClient rtde_client_;

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
  inline int32_t geWithinPitchCount(float pitch_deadzone){
    if(m_pitch_deadzone != pitch_deadzone){
      m_pitch_deadzone = pitch_deadzone;
      m_pitch_within_count = 0;
    }
    return m_pitch_within_count;
  }
  inline bool isSteadyPressure(int pressureIndex){
    return  (m_pressure_steady_count.pressureArray[pressureIndex]>=m_pressure_steady_count_max)?1:0;
  }
 
  private:
 

  ArmAction m_actionMoveJoint;
  ArmAction m_actionMovePitch;
  ArmAction m_actionSingleCommand;
  std::array<ArmAction,8> m_actionMoveChamber;


  void update();
  std::vector<float> LinkH2L(float pitch_angle_rad);

  std::string name_;

  std::string arm_port_;
  unsigned int baudrate_;
 
  bool m_flagDataUpdated=false;
  ArmSensorData m_arm_data_;
 
  std::string m_speed_level_;

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
  float m_pitch_deadzone=0.005;
  float m_pitch_within_count=0;
  float m_pitch_within_count_max=20;

  //pitch steady check
  float m_pitch_last=0;
  bool m_pitch_steady_flag=false;
  float m_pitch_delta=0;
  float m_pitch_delta_deadzone=0.003;
  int32_t m_pitch_steady_count=0;
  int32_t m_pitch_steady_count_max=50;
 
  //pin and limit steady check

  std::array<int8_t,8> m_io_steady_count={};
  int m_io_steady_count_max = 20;
};
  