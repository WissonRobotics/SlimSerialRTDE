#pragma once
#include <string.h>
#include <chrono>
#include <cmath>
#include <functional>

#include <queue>
#include <thread>
#include <typeinfo>

#include <source_location>
#include <spdlog/fmt/fmt.h>

#include "SlimSerialRTDE/SlimSerialRTDE.h"

#ifdef USE_LOGURU
#include <loguru/loguru.hpp>
#endif
inline std::chrono::_V2::steady_clock::time_point TIC()
{
  return std::chrono::steady_clock::now();
}
inline double TOC(std::chrono::_V2::steady_clock::time_point start_time_point)
{
  return (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time_point)).count();
}

/*************************************************** Arm Action Begin******************************************************************** */
struct ArmActionResult
{
  WS_STATUS action_result;
  double action_time_cost;
};
struct ArmActionProgress
{
  int conditionCheckNumber;
  float progressIndicator;
  float progressIndicatorOriginal;
  int percentage;
  std::string message;
  int messageFrequencyDivide;
  int commandTimes;
  double timeCost;
  int tempDataInt[5];
  double tempDataDouble[3];
};

class ArmAction
{
public:
  ArmAction();
  ArmAction(bool preemptible);

  ArmAction(std::string _name_prefix, bool preemptible);

  void setActionNamePrefix(std::string _name_prefix = "");
  void setActionCompleteCallback(std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback);

  void reset();
  void disable();
  void enable();
  void stopAction();
  bool isStopped();

  WS_STATUS spinWait(std::function<bool()> &&waitCondition, int timeoutMS, int intervalMS = 0);
  void setPreemptible(bool preempt);

  void enableLogger(std::shared_ptr<spdlog::logger> ext_logger = spdlog::default_logger());
  void disableLogger();
  std::unique_ptr<std::jthread> actionThread;
  std::shared_ptr<spdlog::logger> m_logger = spdlog::default_logger();

  /*************************************************** Function repeat Begin******************************************************************** */
  template <typename Func, typename... Args>
  WS_STATUS command_repeat(WS_STATUS expect_return_value, double timeout_max, int repeat_max, Func &&func, Args &&...args)
  {

    auto func_start_time = TIC();
    int repeat_times = 0;

    while (true)
    {

      repeat_times++;

      // auto result = func(std::forward<Args>(args)...);
      auto result = std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
      auto nextTick = TIC() + std::chrono::milliseconds(100);

      if (result == expect_return_value)
      {
        return WS_OK;
      }

      if (repeat_max != 0 && repeat_times >= repeat_max)
      {
#ifdef USE_LOGURU
        LOG_F(ERROR, "[Command_repeat Timeout] %s exceed %d times ", std::source_location::current().function_name(), repeat_max);
#else
        SPDLOG_ERROR("[Command_repeat Timeout] {} exceed {} times ", std::source_location::current().function_name(), repeat_max);
#endif
        return WS_TIMEOUT;
      }

      if (timeout_max != 0)
      {
        if (TOC(func_start_time) > timeout_max)
        {
#ifdef USE_LOGURU
          LOG_F(ERROR, "[Command_repeat Timeout] %s timeout for %f ms", std::source_location::current().function_name(), timeout_max);
#else
          SPDLOG_ERROR("[Command_repeat Timeout] {} timeout for {} ms", std::source_location::current().function_name(), timeout_max);
#endif
          return WS_TIMEOUT;
        }
      }

      std::this_thread::sleep_until(nextTick);
    }
  }

  template <typename Func, typename... Args>
  WS_STATUS act(std::string actionName, std::function<bool(ArmActionProgress &progress)> &&actionStartCondition, std::function<bool(ArmActionProgress &progress)> &&actionStopCondition, std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition, double timeout_max, Func &&actionFunc, Args &&...actionArgs)
  {
    std::unique_lock lock_(m_actionMtx);
    std::string m_actionName_last = m_actionName;
    m_actionName = m_actionName_prefix + actionName;
    if (m_preemptible)
    {
      lock_.unlock();
    }

#ifdef USE_LOGURU
    LOG_F(INFO, "[Act %s] [Init]", m_actionName.c_str());
#else
    SPDLOG_INFO("[Act {}] [Init]", m_actionName);
#endif

    if (!isStopped())
    {

      stopAction();

#ifdef USE_LOGURU
      LOG_F(WARNING, "[Act %s] [Preempt] A previous Act %s is running, stopping it now...", m_actionName.c_str(), m_actionName_last.c_str());
#else
      SPDLOG_WARN("[Act {}] [Preempt] A previous Act {} is running, stopping it now...", m_actionName, m_actionName_last);
#endif

      int timeoutSeconds = 5;
      while (spinWait([&]()
                      { return isStopped(); }, 1, 200) != WS_OK)
      {

        // stopAction();

#ifdef USE_LOGURU
        LOG_F(WARNING, "[Act %s] [Preempt] A previous Act %s is still running, stopping it now...", m_actionName.c_str(), m_actionName_last.c_str());
#else
        SPDLOG_WARN("[Act {}] [Preempt] A previous Act {} is still running, stopping it now...", m_actionName, m_actionName_last);
#endif

        if (timeoutSeconds-- <= 0)
        {
          break;
        }
      }

      if (isStopped())
      {
#ifdef USE_LOGURU
        LOG_F(INFO, "[Act %s] [Preempt] Successfully stopped previous Act %s, going to perform current action anyway", m_actionName.c_str(), m_actionName_last.c_str());
#else
        SPDLOG_DEBUG("[Act {}] [Preempt] Successfully stopped previous Act {}, going to perform current action anyway", m_actionName, m_actionName_last);
#endif
      }
      else
      {
#ifdef USE_LOGURU
        LOG_F(WARNING, "[Act %s] [Preempt] Fail to stop previous Act %s, going to perform current action anyway", m_actionName.c_str(), m_actionName_last.c_str());
#else
        SPDLOG_WARN("[Act {}] [Preempt] Fail to stop previous Act {}, going to perform current action anyway", m_actionName, m_actionName_last);
#endif
      }
    }

    reset();

    m_idleFlag = false;

    m_timeout = timeout_max;
    // optional store the functions
    //  m_function = std::forward<Func>(actionFunc);
    //  m_args = std::forward<Args>(actionArgs)...;
    m_actionStartCondition = std::forward<std::function<bool(ArmActionProgress & progress)>>(actionStartCondition);
    m_actionStopCondition = std::forward<std::function<bool(ArmActionProgress & progress)>>(actionStopCondition);
    m_actionProgressIndication = std::forward<std::function<bool(ArmActionProgress & progress)>>(actionProgressIndication);
    m_actionCompleteCondition = std::forward<std::function<bool(ArmActionProgress & progress)>>(actionCompleteCondition);

    // fast fail
    if (!m_actionEnabled)
    {
#ifdef USE_LOGURU
      LOG_F(ERROR, "[Act %s] [Abort] Action is disabled", m_actionName.c_str());
#else
      SPDLOG_ERROR("[Act {}] [Abort] Action is disabled", m_actionName);
#endif
      m_idleFlag = true;
      m_actionResult = WS_STATUS::WS_FAIL;
      if (m_actionCompleteCallback)
      {
        m_actionCompleteCallback(m_actionResult);
      }
      return m_actionResult;
    }

    // fast fail for start pre condition
    if (!m_actionStartCondition(m_progress))
    {
#ifdef USE_LOGURU
      LOG_F(ERROR, "[Act %s] [Abort] %s", m_actionName.c_str(), m_progress.message.c_str());
#else
      SPDLOG_ERROR("[Act {}] [Abort] {}", m_actionName, m_progress.message);
#endif
      m_idleFlag = true;
      m_actionResult = WS_STATUS::WS_ERROR;
      if (m_actionCompleteCallback)
      {
        m_actionCompleteCallback(m_actionResult);
      }
      return m_actionResult;
    }
    else
    {
#ifdef USE_LOGURU
      LOG_F(INFO, "[Act %s] [Start] %s", m_actionName.c_str(), m_progress.message.c_str());
#else
      SPDLOG_INFO("[Act {}] [Start] {}", m_actionName, m_progress.message);
#endif
    }

    auto action_start_time = TIC();

    WS_STATUS func_ret = command_repeat(WS_STATUS::WS_OK, 0, m_command_repeat_max, std::forward<Func>(actionFunc), std::forward<Args>(actionArgs)...);

    if (func_ret != WS_STATUS::WS_OK)
    {
      m_progress.timeCost = TOC(action_start_time);
#ifdef USE_LOGURU
      LOG_F(ERROR, "[Act %s] [Failed] [%.1f s] commanding timeout over %d times ", m_actionName.c_str(), m_progress.timeCost, m_command_repeat_max);
#else
      SPDLOG_ERROR("[Act {}] [Failed] [{:1f} s] commanding timeout over {} times ", m_actionName, m_progress.timeCost, m_command_repeat_max);
#endif
      m_actionResult = WS_STATUS::WS_FAIL;
    }
    else
    {
#ifdef USE_LOGURU
      LOG_F(INFO, "[Act %s] [Command] [OK]", m_actionName.c_str());
#else
      SPDLOG_TRACE("[Act {}] [Command] [OK]", m_actionName);
#endif
      action_start_time = TIC();
      while (true)
      {
        auto nextTick = TIC() + std::chrono::milliseconds(10);
        m_progress.timeCost = TOC(action_start_time);

        //
        // if(TOC(command_start_time) >= 1.0) {
        //   command_repeat(WS_STATUS::WS_OK, 0, 2, std::forward<Func>(actionFunc), std::forward<Args>(actionArgs)...);
        //   SPDLOG_INFO( "[Act {}] [Command repeat] [{:1f} s]", m_actionName, m_progress.timeCost);
        //   command_start_time = TIC();
        // }

        // timeout
        if (m_progress.timeCost > m_timeout)
        {
#ifdef USE_LOGURU
          LOG_F(ERROR, "[Act %s] [Timeout] [%.1f s]", m_actionName.c_str(), m_progress.timeCost);
#else
          SPDLOG_ERROR("[Act {}] [Timeout] [{:1f} s]", m_actionName, m_progress.timeCost);
#endif
          m_actionResult = WS_STATUS::WS_TIMEOUT;
          break;
        }

        // external stop condition
        if (m_actionStopCondition(m_progress))
        {
#ifdef USE_LOGURU
          LOG_F(WARNING, "[Act %s] [Stopped] [%.1f s] %s", m_actionName.c_str(), m_progress.timeCost, m_progress.message.c_str());
#else
          SPDLOG_WARN("[Act {}] [Stopped] [{:1f} s] {}", m_actionName, m_progress.timeCost, m_progress.message);
#endif
          m_actionResult = WS_STATUS::WS_ABORT;
          break;
        }

        // internal stop request
        if (m_stopRequest || (!m_actionEnabled))
        {
#ifdef USE_LOGURU
          LOG_F(WARNING, "[Act %s] [Stopped] [%.1f s] Stop command is received", m_actionName.c_str(), m_progress.timeCost);
#else
          SPDLOG_WARN("[Act {}] [Stopped] [{:1f} s] Stop command is received", m_actionName, m_progress.timeCost);
#endif
          m_stopRequest = false;
          m_actionResult = WS_STATUS::WS_ABORT;
          break;
        }

        // complete condition
        if (m_actionCompleteCondition(m_progress))
        {
#ifdef USE_LOGURU
          LOG_F(INFO, "[Act %s] [Successful] [%.1f s] %s", m_actionName.c_str(), m_progress.timeCost, m_progress.message.c_str());
#else
          SPDLOG_INFO("[Act {}] [{:1f} s] {}", m_actionName, m_progress.timeCost, m_progress.message);
          // SPDLOG_INFO( "[Act {}] [Progress {}%%] [{:1f} s] {}",m_actionName,m_progress.percentage,m_progress.timeCost,m_progress.message);
          SPDLOG_INFO("[Act {}] [Successful]", m_actionName);
#endif
          m_actionResult = WS_STATUS::WS_OK;
          break;
        }

        // progress indication
        if (m_actionProgressIndication(m_progress))
        {
          if (m_progress.conditionCheckNumber == 0)
          {
            m_progress.progressIndicatorOriginal = m_progress.progressIndicator;
            if (m_progress.progressIndicatorOriginal == 0)
            {
              m_progress.progressIndicatorOriginal = 0.000000001;
            }
          }
          m_progress.percentage = (int)(100 * (1 - m_progress.progressIndicator / m_progress.progressIndicatorOriginal));
        }

        if (m_progress.conditionCheckNumber % m_progress.messageFrequencyDivide == 0)
        {
#ifdef USE_LOGURU
          LOG_F(INFO, "[Act %s] [%.1f s] %s", m_actionName.c_str(), m_progress.timeCost, m_progress.message.c_str());
// SPDLOG_INFO( "[Act {}] [Progress {}] [{:1f} s] {}",m_actionName,m_progress.percentage,m_progress.timeCost,m_progress.message);
#else
          SPDLOG_INFO("[Act {}] [{:1f} s] {}", m_actionName, m_progress.timeCost, m_progress.message);
#endif
        }

        m_progress.conditionCheckNumber++;
        std::this_thread::sleep_until(nextTick);
      }
    }
    m_idleFlag = true;
    if (m_actionCompleteCallback)
    {
      m_actionCompleteCallback(m_actionResult);
    }
    return m_actionResult;
  }

private:
  // If this Act can be interrupted?
  bool m_preemptible = true;
  double m_timeout;
  int m_command_repeat_max;
  std::string m_actionName = "";
  std::string m_actionName_prefix = "";
  // std::function<std::invoke_result_t<Func, Args...>> m_function;
  // std::tuple<Args...> m_args;
  std::function<bool(ArmActionProgress &progress)> m_actionStartCondition;
  std::function<bool(ArmActionProgress &progress)> m_actionStopCondition;
  std::function<bool(ArmActionProgress &progress)> m_actionCompleteCondition;
  std::function<bool(ArmActionProgress &progress)> m_actionProgressIndication;
  std::function<void(WS_STATUS &actionResult)> m_actionCompleteCallback;
  ArmActionProgress m_progress;
  WS_STATUS m_actionResult;

  bool m_stopRequest = false;
  bool m_idleFlag = true;

  bool m_actionEnabled = true;
  std::mutex m_actionMtx;
};