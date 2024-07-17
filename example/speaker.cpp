 

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

SPEAKER::SPEAKER(std::string portname, int baudrate) : slimSerial("Speaker") {
  frameCallbackFunc = std::bind(&SPEAKER::frameCallback, this, std::placeholders::_1);
  slimSerial.addRxFrameCallback(frameCallbackFunc);

  if (portname.size() > 0) {
    connect(portname, baudrate);
  }
  m_cyclePeriod=5000;
  m_cycleMessage= -1; 

   speakThread = std::make_unique<std::jthread>(
      [this](std::stop_token stop_token)
      {
          while (!stop_token.stop_requested())
          {
              auto future = std::async(std::launch::async,[this](){return msgQueue.pop_back(m_cyclePeriod);});
			  auto result = future.get();
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

COM_STATUS SPEAKER::connect(std::string portname, int baudrate) {
  m_portname = portname;
  m_baudrate = baudrate;
  if ((slimSerial.connect(m_portname, m_baudrate)) == true) {
    printf("serial port connectted to %s at %d\r\n", m_portname.c_str(), 9600);
    slimSerial.setLoggerLevel("info");
    slimSerial.setFrameType(0);
    return COM_OK;
  } else {
    printf("serial port  %s connection error\r\n", m_portname.c_str());
    return COM_ERROR;
  }
}

void SPEAKER::frameCallback(std::vector<uint8_t>& rxFrame) {
  // do nothing
}


COM_STATUS SPEAKER::speak(int messageIndex, int speakDurationMs) {
  SpeakMsgMeta msgMeta;
  msgMeta.msg= messageIndex;
  msgMeta.min_speak_duration_ms = speakDurationMs;
  msgQueue.push_back(std::move(msgMeta));
  return COM_OK;
}


COM_STATUS SPEAKER::speakDirect(int messageIndex, int sleepTimeMs) {
  if(messageIndex<0)
    return COM_OK;

  auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(sleepTimeMs);

  std::string messageIndexStr = std::to_string(messageIndex);
  int zeroDigits = 5 - (int)(messageIndexStr.size());
  if (zeroDigits <= 0) {
    printf("speaker error: too large message index\r\n");
    return COM_ERROR;
  }
  messageIndexStr.insert(0, zeroDigits, '0');

  std::vector<uint8_t> txFrame;
  txFrame.emplace_back(0xAA);
  txFrame.emplace_back(0x08);
  txFrame.emplace_back(0x0B);
  txFrame.emplace_back(0x02);
  txFrame.emplace_back(0x2F);
  for (int i = 0; i < 5; i++) {
    txFrame.emplace_back(messageIndexStr[i]);
  }
  txFrame.emplace_back(0x2A);
  txFrame.emplace_back(0x3F);
  txFrame.emplace_back(0x3F);
  txFrame.emplace_back(0x3F);
  txFrame.emplace_back(calCRC(txFrame));

  // send frame
  slimSerial.transmitFrame(txFrame);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // previous command has no return, so need to send another frame to check for connection status
  if (COM_OK == checkStatus()) {
    // sleep for demanded time
    std::this_thread::sleep_until(timeoutPoint);
    return COM_OK;
  } else {
    return COM_ERROR;
  }
 
}




COM_STATUS SPEAKER::checkStatus() {
  std::vector<uint8_t> txFrame;
  // AA 01 00 AB
  txFrame.emplace_back(0xAA);
  txFrame.emplace_back(0x01);
  txFrame.emplace_back(0x00);
  txFrame.emplace_back(0xAB);
  std::vector<uint8_t> rxFrame = slimSerial.transmitReceiveFrame(txFrame, 200);
  if (rxFrame.size() > 0) {
    return COM_OK;
  } else {
    printf("speaker communication error\r\n");
    return COM_ERROR;
  }
}

uint8_t SPEAKER::calCRC(std::vector<uint8_t>& frame) {
  return (uint8_t)(std::accumulate(frame.begin(), frame.end(), 0) & 0xFF);
}

}  // namespace SPEAKER

/************************  example *************************/

// int main(int argc, char** argv) {

//  	SPEAKER::SPEAKER speaker;

// 	if (speaker.connect("/dev/ttyUSB1") == SPEAKER::COM_OK){

// 		if ( speaker.speak(1)== SPEAKER::COM_OK){
// 			//speak 00001xxx.mp3
// 		}
// 		else if ( speaker.speak(2,1500)== SPEAKER::COM_OK){
// 			//speak 00002xxx.mp3  and wait until 1.5s
// 		}
// 		else{

// 		}

// 	}
// 	else{
// 		printf("connection failed\r\n");
// 		return 1;
// 	}
// 	return 0;

// }

/************************  example *************************/