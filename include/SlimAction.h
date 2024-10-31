 
#include <string.h>
#include <chrono>
#include <cmath> 
#include <functional>
 
#include <queue>
#include <thread>
#include <typeinfo>
 
#include "log/monos_log.h"
 
#include <source_location>
#include <spdlog/fmt/fmt.h>
#include "SlimSerialRTDE.h"

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
  m_command_repeat_max(20){
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


  // template <typename Func, typename... Args>
  // WS_STATUS actAsync(std::string actionName,std::function<bool(ArmActionProgress &progress)> &&actionStartCondition,std::function<bool(ArmActionProgress &progress)> &&actionStopCondition,std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition,double timeout_max, Func&& actionFunc, Args&&... actionArgs){
   
  //   actionThread = std::make_unique<std::jthread>([&](){
  //     act(actionName,);
  //   }
  // );
  // }

  template <typename Func, typename... Args>
  WS_STATUS act(std::string actionName,std::function<bool(ArmActionProgress &progress)> &&actionStartCondition,std::function<bool(ArmActionProgress &progress)> &&actionStopCondition,std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition,double timeout_max, Func&& actionFunc, Args&&... actionArgs){
    std::unique_lock lock_(m_actionMtx);
    if(m_preemptible){
      lock_.unlock();
    }
    LOG_F(INFO, "[Act %s] [Init]",actionName.c_str());

    if(!isStopped()){

      stopAction();
      LOG_F(WARNING, "[Act %s] [Preempt] A previous Act %s is running, stopping it now...",actionName.c_str(),m_actionName.c_str());
      
      int timeoutSeconds = 5;
      while(spinWait([&](){return isStopped();},1,200)!=WS_OK){

       // stopAction();
        LOG_F(WARNING, "[Act %s] [Preempt] A previous Act %s is still running, stopping it now...",actionName.c_str(),m_actionName.c_str());
        if(timeoutSeconds--<=0){
          break;
        }
      }

      if(isStopped()){
        LOG_F(1, "[Act %s] [Preempt] Successfully stopped previous Act %s, going to perform current action anyway",actionName.c_str(),m_actionName.c_str());
      }
      else{
        LOG_F(WARNING, "[Act %s] [Preempt] Fail to stop previous Act %s, going to perform current action anyway",actionName.c_str(),m_actionName.c_str());
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
      LOG_F(ERROR, "[Act %s] [Abort] Action is disabled",m_actionName.c_str());
      m_idleFlag = true;
      m_actionResult = WS_STATUS::WS_FAIL;
      if(m_actionCompleteCallback){
        m_actionCompleteCallback(m_actionResult);
      }
      return m_actionResult;
    }


    //fast fail for start pre condition
    if (!m_actionStartCondition(m_progress)){
      LOG_F(ERROR, "[Act %s] [Abort] %s",m_actionName.c_str(),m_progress.message.c_str());
      m_idleFlag = true;
      m_actionResult = WS_STATUS::WS_ERROR;
      if(m_actionCompleteCallback){
        m_actionCompleteCallback(m_actionResult);
      }
      return m_actionResult;
    }
    else{
      LOG_F(INFO, "[Act %s] [Start] %s",m_actionName.c_str(),m_progress.message.c_str());
    }


    auto action_start_time = TIC();
    
    WS_STATUS func_ret = command_repeat(WS_STATUS::WS_OK, 0, m_command_repeat_max, std::forward<Func>(actionFunc), std::forward<Args>(actionArgs)...);
 
    if (func_ret != WS_STATUS::WS_OK) {
      m_progress.timeCost = TOC(action_start_time);
      LOG_F(ERROR, "[Act %s] [Failed] [%.1f s] commanding timeout over %d times ",m_actionName.c_str(),m_progress.timeCost,m_command_repeat_max);
      m_actionResult = WS_STATUS::WS_FAIL;
    }
    else{
      LOG_F(2, "[Act %s] [Command] [OK]",m_actionName.c_str());
      auto command_start_time = TIC();
      while(true){
        auto nextTick = TIC() + std::chrono::milliseconds(defaultFrameTxRxPeriodMs);
        m_progress.timeCost = TOC(action_start_time);

        // 
        if(TOC(command_start_time) >= 1.0) {
          command_repeat(WS_STATUS::WS_OK, 0, 2, std::forward<Func>(actionFunc), std::forward<Args>(actionArgs)...);
          LOG_F(INFO, "[Act %s] [Command repeat] [%.1f s]", m_actionName.c_str(), m_progress.timeCost);
          command_start_time = TIC();
        }
        
        //timeout
        if (m_progress.timeCost > m_timeout) {
          LOG_F(ERROR, "[Act %s] [Timeout] [%.1f s]",m_actionName.c_str(),m_progress.timeCost);
          m_actionResult = WS_STATUS::WS_TIMEOUT;
          break;
        }
 
        //external stop condition
        if (m_actionStopCondition(m_progress)) {
          LOG_F(WARNING, "[Act %s] [Stopped] [%.1f s] %s",m_actionName.c_str(),m_progress.timeCost,m_progress.message.c_str());
          m_actionResult = WS_STATUS::WS_ABORT;
          break;
        }

        //internal stop request
        if (m_stopRequest || (!m_actionEnabled)) {
          LOG_F(WARNING, "[Act %s] [Stopped] [%.1f s] Stop command is received",m_actionName.c_str(),m_progress.timeCost);
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

        if(m_progress.conditionCheckNumber % m_progress.messageFrequencyDivide == 0 ){
          LOG_F(INFO, "[Act %s] [%.1f s] %s",m_actionName.c_str(),m_progress.timeCost,m_progress.message.c_str());
          //LOG_F(INFO, "[Act %s] [Progress %d] [%.1f s] %s",m_actionName.c_str(),m_progress.percentage,m_progress.timeCost,m_progress.message.c_str());
        }

        //complete condition
        if (m_actionCompleteCondition(m_progress)){
          LOG_F(INFO, "[Act %s] [%.1f s] %s",m_actionName.c_str(),m_progress.timeCost,m_progress.message.c_str());
          //LOG_F(INFO, "[Act %s] [Progress %d%%] [%.1f s] %s",m_actionName.c_str(),m_progress.percentage,m_progress.timeCost,m_progress.message.c_str());
          LOG_F(INFO, "[Act %s] [Successful]",m_actionName.c_str());
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
  // If this Act can be interrupted?
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