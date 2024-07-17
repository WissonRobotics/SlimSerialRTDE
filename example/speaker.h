#pragma once
#include "SlimDriveRTDE.h"
#include <cstring>
#include <functional>
 

#include "thread_safe_queue.h"
namespace SPEAKER {

enum COM_STATUS {
  COM_OK = 0,
  COM_ERROR = 1,
};

enum MsgType {
  MsgStartWork = 1,
  MsgGunInserted,
  MsgStartCharging,
  MsgChargingFinished,
  MsgMoving,
  MsgCarDismatch,
  MsgCheckCarPose,
  MsgCheckChargerPort
};

struct SpeakMsgMeta{
  int msg;
  int min_speak_duration_ms;
};

class SPEAKER {
 public:
  SPEAKER(std::string portname = "", int baudrate = 9600);
  ~SPEAKER();
  COM_STATUS connect(std::string portname, int baudrate = 9600);

  bool isConnected();

  COM_STATUS speak(int messageIndex, int sleepTimeMs = 0);

  void speakCyclially(int messageIndex, int cyclePeriodMs){
    m_cycleMessage = messageIndex;
    m_cyclePeriod = cyclePeriodMs;
    if(cyclePeriodMs<0){
      stopSpeakCyclially();
    }
  }

  void speakCyclially(){
    speakCyclially(m_cycleMessage,m_cyclePeriod);
  }

  void stopSpeakCyclially(){
    m_cycleFlag = false;
    m_cyclePeriod = 1000;
  }

  COM_STATUS checkStatus();

 
  COM_STATUS speakDirect(int messageIndex, int sleepTimeMs = 0);


 private:
  uint8_t calCRC(std::vector<uint8_t>& frame);
  void frameCallback(std::vector<uint8_t>& rxFrame);
  std::function<void(std::vector<uint8_t>& rxFrame)> frameCallbackFunc;

  std::string m_portname;
  int m_baudrate;
  int m_cycleMessage;
  int m_cyclePeriod;
  bool m_cycleFlag;
  SlimDriveRTDE slimSerial;
  std::unique_ptr<std::jthread> speakThread;
  dp::thread_safe_queue<SpeakMsgMeta> msgQueue;

};

}  // namespace SPEAKER