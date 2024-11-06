 

#include "speaker.h"
#include <stdlib.h>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <bits/stdc++.h>
#include <future>
namespace SPEAKER {

SPEAKER::SPEAKER(std::string portname, int baudrate) : slimSerial() {
  frameCallbackFunc = std::bind(&SPEAKER::frameCallback, this, std::placeholders::_1,std::placeholders::_2);
  slimSerial.addRxFrameCallback(frameCallbackFunc);

  if (portname.size() > 0) {
    connect(portname, baudrate);
  }
  m_cyclePeriod=5000;
  m_cycleMessage= -1; 

  queryFrame[0] = 0xAA;
  queryFrame[1] = 0x01;
  queryFrame[2] = 0x00;
  queryFrame[3] = 0xAB;
 
   speakThread = std::make_unique<std::jthread>(
      [this](std::stop_token stop_token)
      {
          while (!stop_token.stop_requested())
          {  
              auto result = msgQueue.pop_back(m_cyclePeriod);
 
              if(result!=std::nullopt){
                 SpeakMsgMeta msgMeta = result.value();
                 speakDirect(msgMeta.msg,msgMeta.min_speak_duration_ms);
              }
              else{
                 if(m_cycleFlag){
                  speakDirect(m_cycleMessage,0);
                 }
              }
               
          }; 
      }
  ); 


}

SPEAKER::~SPEAKER(){
  speakThread->request_stop();
	speakThread->join();
}


bool SPEAKER::isConnected() { return slimSerial.isConnected(); }

WS_STATUS SPEAKER::connect(std::string portname, int baudrate) {
  m_portname = portname;
  m_baudrate = baudrate;
  if ((slimSerial.connect(m_portname, m_baudrate)) == WS_OK) {
    // printf("serial port connectted to %s at %d\r\n", m_portname.c_str(), 9600);
    slimSerial.setFrameType(0);
    return WS_OK;
  } else {
    // printf("serial port  %s connection error\r\n", m_portname.c_str());
    return WS_ERROR;
  }
}

void SPEAKER::frameCallback(uint8_t *pdata,uint32_t databytes) {
  // do nothing
}


WS_STATUS SPEAKER::speak(int messageIndex, int speakDurationMs) {
  SpeakMsgMeta msgMeta;
  msgMeta.msg= messageIndex;
  msgMeta.min_speak_duration_ms = speakDurationMs;
  msgQueue.push_back(std::move(msgMeta));
  return WS_OK;
}


WS_STATUS SPEAKER::speakDirect(int messageIndex, int sleepTimeMs) {
  if(messageIndex<0)
    return WS_OK;

  auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(sleepTimeMs);

  std::string messageIndexStr = std::to_string(messageIndex);
  int zeroDigits = 5 - (int)(messageIndexStr.size());
  if (zeroDigits <= 0) {
 
    return WS_ERROR;
  }
  messageIndexStr.insert(0, zeroDigits, '0');

  // std::vector<uint8_t> txFrame;
  uint8_t *p=&txFrame[0];
  *p++=0xAA;
  *p++=0x08;
  *p++=0x0B;
  *p++=0x02;
  *p++=0x2F;
  for (int i = 0; i < 5; i++) {
    *p++=messageIndexStr[i];
  }
  *p++=0x2A;
  *p++=0x3F;
  *p++=0x3F;
  *p++=0x3F;
  *p++=calCRCU8(&txFrame[0],14);

  // send frame
  slimSerial.transmitFrame(txFrame);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // previous command has no return, so need to send another frame to check for connection status
  if (WS_OK == checkStatus()) {
    // sleep for demanded time
    std::this_thread::sleep_until(timeoutPoint);
    return WS_OK;
  } else {
    return WS_ERROR;
  }
 
}




WS_STATUS SPEAKER::checkStatus() {
 
  return slimSerial.transmitReceiveFrame(queryFrame,200);
}

uint8_t SPEAKER::calCRCU8(uint8_t *pdata,int datasize) {
  uint32_t s=0;
  for(int i=0;i<datasize;i++){
    s+=pdata[i];
  }
  uint8_t crc= (uint8_t)(s&0xFF);
  return crc;
}

}  // namespace SPEAKER
 