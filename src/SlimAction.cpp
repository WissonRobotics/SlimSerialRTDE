#include "SlimSerialRTDE/SlimAction.h"

ArmAction::ArmAction() : m_preemptible(true),
                         m_timeout(60),
                         m_command_repeat_max(20),
                         m_actionName_prefix("")
{
    reset();
};

ArmAction::ArmAction(bool preemptible) : m_preemptible(preemptible),
                                         m_timeout(60),
                                         m_command_repeat_max(20),
                                         m_actionName_prefix("")
{
    reset();
};

ArmAction::ArmAction(std::string _name_prefix, bool preemptible)
    : m_preemptible(preemptible),
      m_timeout(60),
      m_command_repeat_max(20),
      m_actionName_prefix(_name_prefix)
{
    reset();
}

void ArmAction::setActionNamePrefix(std::string _name_prefix)
{
    m_actionName_prefix = _name_prefix;
}

void ArmAction::setActionCompleteCallback(std::function<void(WS_STATUS &actionResutl)> &&actionCompleteCallback)
{
    if (actionCompleteCallback)
    {
        m_actionCompleteCallback = std::forward<std::function<void(WS_STATUS & actionResult)>>(actionCompleteCallback);
    }
}

// template <typename Func, typename... Args>
// WS_STATUS actAsync(std::string actionName,std::function<bool(ArmActionProgress &progress)> &&actionStartCondition,std::function<bool(ArmActionProgress &progress)> &&actionStopCondition,std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition,double timeout_max, Func&& actionFunc, Args&&... actionArgs){

//   actionThread = std::make_unique<std::jthread>([&](){
//     act(actionName,);
//   }
// );
// }


void ArmAction::reset()
{
    m_progress.commandTimes = 0;
    m_progress.conditionCheckNumber = 0;
    m_progress.message = "";
    m_progress.messageFrequencyDivide = 1;
    m_progress.percentage = 0;
    m_progress.timeCost = 0;
    for (auto &item : m_progress.tempDataInt)
    {
        item = 0;
    }
    for (auto &item : m_progress.tempDataDouble)
    {
        item = 0;
    }
}

void ArmAction::disable()
{
    m_actionEnabled = false;
}

void ArmAction::enable()
{
    m_actionEnabled = true;
}

void ArmAction::stopAction()
{
    m_stopRequest = true;
}

bool ArmAction::isStopped()
{
    return m_idleFlag;
}

WS_STATUS ArmAction::spinWait(std::function<bool()> &&waitCondition, int timeoutMS, int intervalMS)
{
    auto timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMS);
    while (true)
    {
        if (waitCondition())
        {
            return WS_OK;
        }
        if (std::chrono::steady_clock::now() > timeoutPoint)
        {
            return WS_TIMEOUT;
        }
        if (intervalMS > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMS));
        }
    }
}

void ArmAction::setPreemptible(bool preempt)
{
    m_preemptible = preempt;
}

void ArmAction::enableLogger(std::shared_ptr<spdlog::logger> ext_logger)
{
    m_logger = ext_logger;
}

void ArmAction::disableLogger()
{
    if (spdlog::get("disabledLogger"))
    {
        m_logger = spdlog::get("disabledLogger");
    }
    else
    {
        m_logger = spdlog::stdout_color_mt("disabledLogger");
    }
    m_logger->set_level(spdlog::level::off);
}
