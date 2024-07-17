#ifndef __MONOS_LOG_H
#define __MONOS_LOG_H
 
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
//#define DEBUG(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_DEBUG(spdlog::get("daily_logger"), __VA_ARGS__)
#define INFO(...)  
//#define WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_WARN(spdlog::get("daily_logger"), __VA_ARGS__)
//#define ERROR(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_ERROR(spdlog::get("daily_logger"), __VA_ARGS__)

#define TASK_DATA(...)  
#define RECORD_DATA(stage_name,result,start_time,duration,repeat_times)\
    do { \
        TASK_DATA("|  {0: ^{5}} | duration:{1: <{6}} | repeat_times:{2: <{7}} | result:{3: <{7}} |",stage_name,duration,repeat_times,result,start_time,45,20,10);\
    } while(0)
#endif

#define RX_DATA(...)  
#define TX_DATA(...)  
// std::string time_to_string(std::chrono::steady_clock::time_point tp)
// {
//   // auto time = to_time_t(start_time);
//   auto time = std::chrono::system_clock::to_time_t(
//       std::chrono::system_clock::now() +
//       (tp - std::chrono::steady_clock::now()));
//   std::stringstream ss;
//   ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
//   std::string str = ss.str();
//   return str;
// }

