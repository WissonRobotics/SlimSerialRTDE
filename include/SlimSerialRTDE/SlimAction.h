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

  /*************************************************** Function repeat Begin******************************************************************** */
  template <typename Func, typename... Args>
  WS_STATUS command_repeat(WS_STATUS expect_return_value, double timeout_max, int repeat_max, Func &&func, Args &&...args);

  template <typename Func, typename... Args>
  WS_STATUS act(std::string actionName, std::function<bool(ArmActionProgress &progress)> &&actionStartCondition, std::function<bool(ArmActionProgress &progress)> &&actionStopCondition, std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition, double timeout_max, Func &&actionFunc, Args &&...actionArgs);

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