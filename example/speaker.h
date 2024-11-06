#pragma once
#include <cstring>
#include <functional>
#include <thread>
#include "SlimSerialRTDE/SlimSerialRTDE.h"
#include <algorithm>
#include <deque>
#include <mutex>
#include <optional>
#include <condition_variable> 
namespace SPEAKER {

template <typename T>
  class Thread_Safe_Blocking_Queue {
    public:

      Thread_Safe_Blocking_Queue() = default;


      void push_back(T&& value) {
          {

              std::unique_lock<std::mutex> lock(_sync);
              data_.push_back(std::forward<T>(value));
          }
          _cvCanPop.notify_one();
      }

      void push_front(T&& value) {
          {

          std::unique_lock<std::mutex> lock(_sync);
          data_.push_front(std::forward<T>(value));
          }
          _cvCanPop.notify_one();
      }

      bool empty() const {

          std::unique_lock<std::mutex> lock(_sync);
          return data_.empty();
      }

      std::optional<T> pop_front(int timeoutMs=0) {

          std::unique_lock<std::mutex> lock(_sync);
          
          auto timeExp = std::chrono::system_clock::now() + std::chrono::milliseconds(timeoutMs);
          while(data_.empty()) {
              if(_cvCanPop.wait_until(lock,timeExp)==std::cv_status::timeout){
                  break;
              }
          }

          if (data_.empty()) return std::nullopt;

          auto front = std::move(data_.front());
          data_.pop_front();
          return front;
      }

      std::optional<T> pop_back(int timeoutMs=0) {
          // std::scoped_lock lock(mutex_);
          std::unique_lock<std::mutex> lock(_sync);

          auto timeExp = std::chrono::system_clock::now() + std::chrono::milliseconds(timeoutMs);
          while(data_.empty()) {
              if(_cvCanPop.wait_until(lock,timeExp)==std::cv_status::timeout){
                  break;
              }
          }

          if (data_.empty()) return std::nullopt;

          auto back = std::move(data_.back());
          data_.pop_back();
          return back;
      }


    private:
      std::deque<T> data_{};
      std::mutex _sync;
      std::condition_variable _cvCanPop;
  };

 
enum MsgType {
  MsgStartWork = 1,
  MsgGunInserted,
  MsgStartCharging,
  MsgChargingFinished,
  MsgMoving,
  MsgCarDismatch,
  MsgCheckCarPose,
  MsgCheckChargerPort,
  MsgGunUnplugFailed,
  MsgRobotError,
  MsgArmOverLimit,
  MsgCarDistanceSmall,
  MsgCheckLine,
  MsgRetryTask,
  MsgSolftArmMoveFailed,
  MsgCameraLocateError,
  MsgPeopleInvadeOverTimes,
  MsgPeopleInvading,
  MsgRightCarPoseInvalid,
  MsgLeftCarPoseInvalid
};

struct SpeakMsgMeta{
  int msg;
  int min_speak_duration_ms;
};

class SPEAKER {
 public:
  SPEAKER(std::string portname = "", int baudrate = 9600);
  ~SPEAKER();
  WS_STATUS connect(std::string portname, int baudrate = 9600);

  bool isConnected();

  WS_STATUS speak(int messageIndex, int sleepTimeMs = 0);

  void speakCyclially(int messageIndex, int cyclePeriodMs){
    m_cycleMessage = messageIndex;
    m_cyclePeriod = cyclePeriodMs;
    if(cyclePeriodMs<0){
      stopSpeakCyclially();
    }
    else
    {
      m_cycleFlag = true;
    }
  }

  void speakCyclially(){
    speakCyclially(m_cycleMessage,m_cyclePeriod);
  }

  void stopSpeakCyclially(){
    m_cycleFlag = false;
    m_cyclePeriod = 1000;
  }

  WS_STATUS checkStatus();

 
  WS_STATUS speakDirect(int messageIndex, int sleepTimeMs = 0);


 private:
  uint8_t calCRCU8(uint8_t *pdata,int datasize);
  void frameCallback(uint8_t *pdata,uint32_t databytes);
  std::function<void(uint8_t *pdata,uint32_t databytes)> frameCallbackFunc;

  std::string m_portname;
  int m_baudrate;
  int m_cycleMessage;
  int m_cyclePeriod;
  bool m_cycleFlag;
  SlimSerialRTDE slimSerial;
  std::array<uint8_t,16> txFrame;
  std::array<uint8_t,4> queryFrame={0xAA,0x01,0x00,0xAB};
  std::unique_ptr<std::jthread> speakThread;
  Thread_Safe_Blocking_Queue<SpeakMsgMeta> msgQueue;

};

}  // namespace SPEAKER