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

/*************************************************** Function repeat Begin******************************************************************** */
template <typename Func, typename... Args>
WS_STATUS ArmAction::command_repeat(WS_STATUS expect_return_value, double timeout_max, int repeat_max, Func &&func, Args &&...args)
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
WS_STATUS ArmAction::act(std::string actionName, std::function<bool(ArmActionProgress &progress)> &&actionStartCondition, std::function<bool(ArmActionProgress &progress)> &&actionStopCondition, std::function<bool(ArmActionProgress &progress)> &&actionProgressIndication, std::function<bool(ArmActionProgress &progress)> &&actionCompleteCondition, double timeout_max, Func &&actionFunc, Args &&...actionArgs)
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
