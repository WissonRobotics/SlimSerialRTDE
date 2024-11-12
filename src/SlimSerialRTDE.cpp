#include "SlimSerialRTDE/SlimSerialRTDE.h"
#include "SlimSerialRTDE/AsyncSerial.h"
#include <spdlog/fmt/bin_to_hex.h>
#include <spdlog/fmt/fmt.h>
#include "spdlog/sinks/stdout_color_sinks.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <iterator>
#include <functional>
#include <cmath> 

class SlimSerialRTDE::SlimSerialRTDEImpl : public AsyncSerial {
 public:
  SlimSerialRTDEImpl();
  ~SlimSerialRTDEImpl();

  WS_STATUS transmitReceiveFrame(std::vector<uint8_t> const& txframe, uint32_t timeoutMS = 20);
  WS_STATUS transmitReceiveFrame(uint8_t* pData, uint16_t datasize, uint32_t timeoutMS = 20);

  uint32_t readBuffer(uint8_t* pDes, uint32_t nBytes = 1, uint32_t timeoutMS = 10000);

  uint32_t clearRxBuffer();
 
  inline uint8_t getFrameType();
  void setFrameType(uint8_t ftype);

  void addCRC(uint8_t* pData, uint16_t bytesIncludingHeaders);
  uint16_t calculateCRC(uint8_t* data, uint32_t datasize);

  bool applyFuncodeFilter(uint8_t funcodeIn);
  bool applyAddressFilter(uint8_t addressIn);

  void printRxBuffer();

  std::vector<uint8_t> assembleTxFrameWithAddress(uint8_t des, uint8_t fcode, std::vector<uint8_t> const& payload);
  std::vector<uint8_t> assembleTxFrameWithAddress(uint8_t des, uint8_t fcode, uint8_t* payload, uint16_t payloadsize);

  std::vector<uint8_t> getRxFrame();

  WS_STATUS spinWait(std::function<bool()>&& waitCondition, int timeoutMS, int intervalMS = 0);

  bool dataFlag;
  void rxDataCallback();
  WS_STATUS frameParser();
  bool frameParsed;
  std::mutex frameParserMtx;
  std::condition_variable frameParserCV;
//   std::unique_ptr<std::jthread> frameParserThread;

  bool frameCompleteFlag;
  std::mutex frameCompleteMtx;
  std::condition_variable frameCompleteCV;
  std::function<void(uint8_t* rxFrame, uint32_t databytes)> frameCallback;

  std::mutex readBufferMtx;
  std::condition_variable readBufferCV;

  std::array<uint8_t, 512> _inFrame;
  uint32_t _inFrameBytes;

  uint8_t default_headers[2];
  uint8_t default_address;

  uint8_t _frameType;

  uint8_t addressFilter[ADDRESS_FILTER_MAX_LEN];
  bool addressFilterOn;
  uint8_t addressFilter_num;

  uint8_t funcodeFilter[FUNCODE_FILTER_MAX_LEN];
  bool funcodeFilterOn;
  uint8_t funcodeFilter_num;

  bool lengthFilterOn;

  uint16_t lengthFilterMax = 2048;

  int m_totalRxFrames = 0;
  uint64_t m_lastTxRxTime = 0;

  uint64_t m_tic_rxDataCallback = 0;
  uint64_t m_tic_frameParser = 0;
  uint64_t m_tic_frameCallback = 0;
  uint64_t m_tic_frameComplete = 0;
  uint64_t m_tic_frameStart = 0;

  uint64_t m_tic_frameParserEnd = 0;

  bool m_isServer = true;
};

SlimSerialRTDE::SlimSerialRTDEImpl::SlimSerialRTDEImpl(): AsyncSerial() {
  addRxDataCallback([this]() { rxDataCallback(); });
  // frameParserThread = std::make_unique<std::jthread>(
  // 	[this](std::stop_token stop_token)
  // 	{
  // 		while (!stop_token.stop_requested())
  // 		{
  // 			{
  // 				std::unique_lock<std::mutex> lk_ack(frameParserMtx);
  // 				dataFlag = false;
  // 				frameParserCV.wait(lk_ack, [&]() {return (dataFlag || stop_token.stop_requested()); });
  // 			}

  // 			if (!stop_token.stop_requested() && dataFlag) {

  // 				frameParser();

  // 			}

  // 		};
  // 	}
  // );
  memset(&_inFrame[0], 0, sizeof(_inFrame));
  _inFrameBytes = 0;
 
  dataFlag = false;

  addressFilter_num = 0;
  funcodeFilter_num = 0;
  addressFilterOn = false;
  funcodeFilterOn = false;

  m_totalRxFrames = 0;
  setFrameType(SLIMSERIAL_FRAME_TYPE_1_NUM);
}

SlimSerialRTDE::SlimSerialRTDEImpl::~SlimSerialRTDEImpl() {
  try {
    // if (frameParserThread) {
    //   {
    //     frameParserThread->request_stop();
    //     frameParserCV.notify_one();
    //   }
    //   frameParserThread->join();
    // }

    {
      std::unique_lock<std::mutex> lock_(frameParserMtx);
      frameCompleteFlag = false;
      frameCompleteCV.notify_one();
    }
  } catch (...) {
  }
}

void SlimSerialRTDE::SlimSerialRTDEImpl::rxDataCallback() {
  // call parser thread to begin parse; if the parsing is still on going, missng this notificaion wouldn't matter
  // std::unique_lock<std::mutex> lk(frameParserMtx);
  dataFlag = true;
  m_tic_rxDataCallback = getTimeUTC();
  if (m_isServer) {
    if (frameParser() == WS_OK) {
      frameParsed = true;
    }
  }
  // frameParserCV.notify_one();
}

 

bool SlimSerialRTDE::SlimSerialRTDEImpl::applyAddressFilter(uint8_t addressIn) {
  if (!addressFilterOn) return true;

  /*add custom address filter here*/

  // internal address whitelist
  for (int i = 0; i < addressFilter_num; i++) {
    if (addressIn == addressFilter[i]) return true;
  }

  return false;
}

bool SlimSerialRTDE::SlimSerialRTDEImpl::applyFuncodeFilter(uint8_t funcodeIn) {
  if (!funcodeFilterOn) return true;

  /*add custom address filter here*/

  // internal address whitelist
  for (int i = 0; i < funcodeFilter_num; i++) {
    if (funcodeIn == funcodeFilter[i]) return true;
  }

  return false;
}
inline uint8_t SlimSerialRTDE::SlimSerialRTDEImpl::getFrameType() { return _frameType; }
void SlimSerialRTDE::SlimSerialRTDEImpl::setFrameType(uint8_t ftype) {
  _frameType = ftype;
 
  if (_frameType == SLIMSERIAL_FRAME_TYPE_2_NUM) {
    default_headers[0] = 0xFF;
    default_headers[1] = 0xFF;
  } else {
    default_headers[0] = 0x5A;
    default_headers[1] = 0xA5;
  }
}

WS_STATUS SlimSerialRTDE::SlimSerialRTDEImpl::transmitReceiveFrame(
    std::vector<uint8_t> const& txframe, uint32_t timeoutMS) {
  return transmitReceiveFrame((uint8_t*)txframe.data(), txframe.size(), timeoutMS);
}

WS_STATUS SlimSerialRTDE::SlimSerialRTDEImpl::spinWait(
    std::function<bool()>&& waitCondition, int timeoutMS, int intervalMS) {
  auto timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMS);
  while (true) {
    if (waitCondition()) {
      return WS_OK;
    }
    if (std::chrono::steady_clock::now() > timeoutPoint) {
      return WS_TIMEOUT;
    }
    if (intervalMS > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(intervalMS));
    }
  }
}

WS_STATUS SlimSerialRTDE::SlimSerialRTDEImpl::transmitReceiveFrame(
    uint8_t* pData, uint16_t datasize, uint32_t timeoutMS) {
  WS_STATUS ret = WS_FAIL;

  // aync wait for ack response, spurious wake-up is handled by predate function
  // upon receiving the frameCompleteCV,  the registered framecallback function should have already been excecuted.
  std::unique_lock<std::mutex> lock_(frameCompleteMtx);
  m_logger->debug("[TransmitReceive] [Start]");
  frameCompleteFlag = false;

  m_tic_frameStart = getTimeUTC();
  // block transmit
  if (datasize > 0) {
    std::size_t txedsize = transmit(pData, datasize);
    if (txedsize != datasize) {
      m_logger->warn("[TransmitReceive] [Abort] unmatched transmitted bytes {} out of expected {}", txedsize, datasize);
      frameCompleteFlag = true;
      m_tic_frameComplete = getTimeUTC();
      m_lastTxRxTime = getTimeUTC() - m_tic_frameStart;
      return WS_FAIL;
    }
  }

  // read with timeout
  frameParsed = false;
  if (m_isServer) {
    ret = spinWait([&]() { return frameParsed; }, timeoutMS);
  } else {
    ret = spinWait([&]() { return frameParser(); }, timeoutMS);
  }

  // auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMS);
  // frameParsed = false;
  // while(true){
  // 	if(m_isServer){
  // 		if(frameParsed){
  // 			ret = WS_OK;
  // 			break;
  // 		}
  // 	}
  // 	else if(frameParser()==WS_OK){
  // 		ret = WS_OK;
  // 		break;
  // 	}

  // 	if(std::chrono::steady_clock::now()>timeoutPoint){

  // 		ret = WS_TIMEOUT;
  // 		break;
  // 	}
  // 	//std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // }

  m_tic_frameComplete = getTimeUTC();
  m_lastTxRxTime = getTimeUTC() - m_tic_frameStart;
  frameCompleteFlag = true;

  if (ret == WS_OK) {
    m_logger->debug("[TransmitReceive] [Success]");
  } else {
    m_logger->debug("[TransmitReceive] [Timeout]");
  }

  return ret;

  // if (frameCompleteCV.wait_until(lock_, timeoutPoint, [&]() {return frameCompleteFlag; })) {
  // 	m_logger->trace("Got a frame");
  // 	m_tic_frameComplete = getTimeUTC();
  // 	m_lastTxRxTime = getTimeUTC() - m_tic_frameStart;
  // 	return WS_OK;
  // }
  // else {//timeout occurred
  // 	m_logger->warn("Timeout to receive a frame");
  // 	_inFrame.resize(0);
  // 	m_tic_frameComplete = getTimeUTC();
  // 	m_lastTxRxTime = getTimeUTC() - m_tic_frameStart;
  // 	return WS_TIMEOUT;
  // }
}

uint32_t SlimSerialRTDE::SlimSerialRTDEImpl::readBuffer(uint8_t* pDes, uint32_t nBytes, uint32_t timeoutMS) {
  uint32_t readN = circularBuffer.out(pDes, nBytes);

  if (readN < nBytes) {
    uint32_t remainingBytes = nBytes - readN;
    std::unique_lock<std::mutex> lock_(readBufferMtx);

    auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMS);
    if (readBufferCV.wait_until(
            lock_, timeoutPoint, [&]() { return circularBuffer.availableBytes() >= remainingBytes; })) {
      readN += circularBuffer.out(pDes, remainingBytes);

      m_logger->debug("Reading buffer successfully {} nBytes", readN);
    } else {
      readN += circularBuffer.out(pDes, remainingBytes);

      m_logger->debug("Reading buffer Failed, requesting {} but only got {} nBytes ", nBytes, readN);
    }
  }

  return readN;
}

std::vector<uint8_t> SlimSerialRTDE::SlimSerialRTDEImpl::getRxFrame() {
  std::vector<uint8_t> rframe;
  for (uint32_t i = 0; i < _inFrameBytes; i++) {
    rframe.emplace_back(_inFrame[i]);
  };
  return rframe;
}

uint32_t SlimSerialRTDE::SlimSerialRTDEImpl::clearRxBuffer() {
  return circularBuffer.discardN(circularBuffer.availableBytes());
}

void SlimSerialRTDE::SlimSerialRTDEImpl::printRxBuffer() {
  std::array<uint8_t,4096> buf;
  uint32_t readlen = circularBuffer.peek(&buf[0], circularBuffer.availableBytes()); ;
  m_logger->trace("Rxbuffer has {} bytes: {}",readlen,spdlog::to_hex(std::begin(buf), std::begin(buf) + readlen));
}

// this function will decoding all the available bytes in the rx buffer, calling the frame callback in sequence, until
// available bytes less than a frame is left.
WS_STATUS SlimSerialRTDE::SlimSerialRTDEImpl::frameParser() {
  m_tic_frameParser = getTimeUTC();
  uint32_t remainingBytes = circularBuffer.availableBytes();
  WS_STATUS parserResult = WS_FAIL;
  bool waitingForMore = false;
  while (true) {
    if (remainingBytes == 0) return WS_FAIL;

    m_logger->debug("[frame parser] [Start] Rx has {} bytes", remainingBytes);
    printRxBuffer();

    // type 0 is treating any data to be a valid frame. so simply read out all the data and call framecallback
    if (_frameType == SLIMSERIAL_FRAME_TYPE_0_NUM) {
      _inFrameBytes = circularBuffer.availableBytes();

      circularBuffer.out((uint8_t*)(&_inFrame[0]), _inFrameBytes);
      remainingBytes -= _inFrameBytes;

      parserResult = WS_OK;
      m_logger->debug("[frame parser] [Success] decoded a frame type 0 with {} bytes", _inFrameBytes);
      m_logger->trace("frame content: {}", spdlog::to_hex(std::begin(_inFrame), std::begin(_inFrame) + _inFrameBytes));
   
      // dealing frame
      if (frameCallback) {
        frameCallback(&_inFrame[0], _inFrameBytes);
      }

      m_totalRxFrames++;

      // notificy rx frame handle complete
      //  std::unique_lock<std::mutex> lk(frameCompleteMtx);
      //  frameCompleteFlag = true;
      //  frameCompleteCV.notify_one();

    } else if (_frameType == SLIMSERIAL_FRAME_TYPE_1_NUM) {
      // 5+N+2
      // Header1(5A) + Header2(A5) + Src + dataBytes +  Funcode  + data + crc16
      //  dataBytes =   sizeof(data)
      while ((remainingBytes = circularBuffer.availableBytes()) >= 7) {
        // check header
        uint8_t headersIn[2] = {0, 0};
        headersIn[0] = circularBuffer.peekAt(0);
        headersIn[1] = circularBuffer.peekAt(1);
        if (headersIn[0] == default_headers[0] && headersIn[1] == default_headers[1]) {
          // check address
          uint8_t addressIn = circularBuffer.peekAt(2);
          if (applyAddressFilter(addressIn)) {
            // check funcode
            uint8_t funcodeIn = circularBuffer.peekAt(4);
            if (applyFuncodeFilter(funcodeIn)) {
              // check length
              uint16_t expectedFrameBytes = circularBuffer.peekAt(3) + 7;
              if (expectedFrameBytes <= lengthFilterMax) {
                // got enough rx bytes
                if (remainingBytes >= expectedFrameBytes) {
                  // check crc
                  if (circularBuffer.calculateCRC(expectedFrameBytes - 2) ==
                      circularBuffer.peekAt_U16(expectedFrameBytes - 2)) {
                    // store frame
                    _inFrameBytes = expectedFrameBytes;
                    circularBuffer.out((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                    remainingBytes -= _inFrameBytes;
                    parserResult = WS_OK;
                    m_logger->debug("[frame parser] [Success] Decoded frame type 1 with {} bytes", _inFrameBytes);
                    //m_logger->trace("frame content: {}", spdlog::to_hex(std::begin(_inFrame), std::begin(_inFrame) + _inFrameBytes));
                    m_totalRxFrames++;

                    // dealing frame
                    if (frameCallback) {
                      m_tic_frameCallback = getTimeUTC();
                      frameCallback((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                    }

                    // notificy rx frame handle complete

                    // std::unique_lock<std::mutex> lk(frameCompleteMtx);
                    // frameCompleteFlag = true;
                    // frameCompleteCV.notify_one();
                    continue;
                  } else {  // bad crc

                    m_logger->warn("Bad CRC. calculated 0x{:2X}, recevied 0x{:2X}.",
                        circularBuffer.calculateCRC(expectedFrameBytes - 2),
                        circularBuffer.peekAt_U16(expectedFrameBytes - 2));
                    std::array<uint8_t,256> badframe_buf;
                    circularBuffer.peek(&badframe_buf[0], expectedFrameBytes);
                    m_logger->warn("rx frame content: {}", spdlog::to_hex(std::begin(badframe_buf), std::begin(badframe_buf) + expectedFrameBytes));

                    int discardN = circularBuffer.discardUntilNext(default_headers[0]);
                    remainingBytes -= discardN;
                    m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
                    printRxBuffer();
                    continue;
                  }
                } else {  // not rx finished
                  m_logger->debug("incomplete frame, continue receving");
                  waitingForMore = true;
                  break;
                }
              } else {  // illegal length
                m_logger->warn("illegal length 0x{:2X}", expectedFrameBytes);
                int discardN = circularBuffer.discardUntilNext(default_headers[0]);
                remainingBytes -= discardN;
                m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
                printRxBuffer();

                continue;
              }
            } else {
              // illegal funcode
              m_logger->warn("illegal funcode 0x{:2X}", funcodeIn);

              int discardN = circularBuffer.discardUntilNext(default_headers[0]);
              remainingBytes -= discardN;
              m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
              printRxBuffer();
              continue;
            }
          } else {  // illegal address
            m_logger->warn("illegal address 0x{:2X}", addressIn);
            int discardN = circularBuffer.discardUntilNext(default_headers[0]);
            remainingBytes -= discardN;
            m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
            printRxBuffer();
            continue;
          }
        } else {  // illegal header
          m_logger->warn("illegal header 0x{:2X} 0x{:2X}", headersIn[0], headersIn[1]);
          int discardN = circularBuffer.discardUntilNext(default_headers[0]);
          remainingBytes -= discardN;
          m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
          printRxBuffer();
          continue;
        }
      }
    } else if (_frameType == SLIMSERIAL_FRAME_TYPE_2_NUM) {
      // 2+N+2
      // Header1(FF) + Header2(FF) + databytes + data + crc16
      // databytes = 1 + sizeof(data)
      while ((remainingBytes = circularBuffer.availableBytes()) >= 4) {
        // check header
        uint8_t headersIn[2] = {0, 0};
        headersIn[0] = circularBuffer.peekAt(0);
        headersIn[1] = circularBuffer.peekAt(1);
        if (headersIn[0] == default_headers[0] && headersIn[1] == default_headers[1]) {
          // check funcode
          uint8_t funcodeIn = circularBuffer.peekAt(3);
          if (applyFuncodeFilter(funcodeIn)) {
            // check length
            uint16_t expectedFrameBytes = circularBuffer.peekAt(2) + 4;
            if (expectedFrameBytes <= lengthFilterMax) {
              // got enough rx bytes
              if (remainingBytes >= expectedFrameBytes) {
                // check crc
                if (circularBuffer.calculateCRC(expectedFrameBytes - 2) ==
                    circularBuffer.peekAt_U16(expectedFrameBytes - 2)) {
                  // store frame
                  _inFrameBytes = expectedFrameBytes;
                  circularBuffer.out((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                  remainingBytes -= _inFrameBytes;
                  parserResult = WS_OK;
                  m_logger->debug("[frame parser] [Success] Decoded frame type 2 with {} bytes", _inFrameBytes);
                  //m_logger->trace("frame content: {}", spdlog::to_hex(std::begin(_inFrame), std::begin(_inFrame) + _inFrameBytes));
                  m_totalRxFrames++;

                  // dealing frame
                  if (frameCallback) {
                    m_tic_frameCallback = getTimeUTC();
                    frameCallback((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                  }

                  // notificy rx frame handle complete

                  // std::unique_lock<std::mutex> lk(frameCompleteMtx);
                  // frameCompleteFlag = true;
                  // frameCompleteCV.notify_one();
                  continue;
                } else {  // bad crc

                  m_logger->warn("Bad CRC. calculated 0x{:2X}, recevied 0x{:2X}.",
                      circularBuffer.calculateCRC(expectedFrameBytes - 2),
                      circularBuffer.peekAt_U16(expectedFrameBytes - 2));
                  int discardN = circularBuffer.discardUntilNext(default_headers[0]);
                  remainingBytes -= discardN;
                  m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
                  printRxBuffer();

                  continue;
                }
              } else {  // not rx finished
                m_logger->debug("incomplete frame, continue receving");
                waitingForMore = true;
                break;
              }
            } else {  // illegal length

              m_logger->warn("illegal length 0x{:2X}", expectedFrameBytes);

              int discardN = circularBuffer.discardUntilNext(default_headers[0]);
              remainingBytes -= discardN;
              m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
              printRxBuffer();

              continue;
            }
          } else {
            // illegal funcode

            m_logger->warn("illegal funcode 0x{:2X}", funcodeIn);
            int discardN = circularBuffer.discardUntilNext(default_headers[0]);
            remainingBytes -= discardN;
            m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
            printRxBuffer();

            continue;
          }
        } else {  // illegal header

          m_logger->warn("illegal header 0x{:2X} 0x{:2X}", headersIn[0], headersIn[1]);
          int discardN = circularBuffer.discardUntilNext(default_headers[0]);
          remainingBytes -= discardN;
          m_logger->warn("Discarding {} bytes to find 0x{:2X}", discardN, default_headers[0]);
          printRxBuffer();

          continue;
        }
      }

    } else if (_frameType == SLIMSERIAL_FRAME_TYPE_MODBUS_SERVER_NUM) {
      // src + funcode + data + crc16
      while ((remainingBytes = circularBuffer.availableBytes()) >= 8) {
        // check address. disabled by default
        uint8_t addressIn = circularBuffer.peekAt(0);
        if (applyAddressFilter(addressIn)) {
          // check funcode
          uint16_t funcodeIn = circularBuffer.peekAt(1);
          if (funcodeIn == 0x03 || funcodeIn == 0x06 || funcodeIn == 0x10) {
            
            uint16_t expectedFrameBytes=8;
            if (funcodeIn == 0x03) {
              expectedFrameBytes  = 8;
            }
            else if(funcodeIn == 0x06){
              expectedFrameBytes  =  circularBuffer.availableBytes();//special treatment for 0x06
            }
            else if (funcodeIn == 0x10) {
              expectedFrameBytes  =  (uint16_t)(circularBuffer.peekAt(6)) + 9;
            }
 

            // got enough rx bytes
            if (remainingBytes >= expectedFrameBytes) {
              // check crc
              if (circularBuffer.calculateCRC(expectedFrameBytes - 2) ==
                  circularBuffer.peekAt_U16(expectedFrameBytes - 2)) {
                _inFrameBytes = expectedFrameBytes;
                circularBuffer.out((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                remainingBytes -= _inFrameBytes;
                parserResult = WS_OK;
                m_logger->debug("[frame parser] [Success] Decoded frame type 3 MODBUS with {} bytes", _inFrameBytes);
                //m_logger->trace("frame content: {}", spdlog::to_hex(std::begin(_inFrame), std::begin(_inFrame) + _inFrameBytes));
                m_totalRxFrames++;

                // dealing frame
                if (frameCallback) {
                  m_tic_frameCallback = getTimeUTC();
                  frameCallback((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                }

                // notificy rx frame handle complete

                // std::unique_lock<std::mutex> lk(frameCompleteMtx);
                // frameCompleteFlag = true;
                // frameCompleteCV.notify_one();
                continue;
              } else {  // bad crc
                if(funcodeIn == 0x06 && circularBuffer.availableBytes()<256){//special treatment for 0x06
                  m_logger->debug("writing maybe incomplete, continue receving");
                  waitingForMore = true;
                  break;
                }
                else{
                  m_logger->warn("Bad CRC. calculated 0x{:2X}, recevied 0x{:2X}.",
                      circularBuffer.calculateCRC(expectedFrameBytes - 2),
                      circularBuffer.peekAt_U16(expectedFrameBytes - 2));
                  int discardN = circularBuffer.discardUntilNext(default_address);
                  remainingBytes -= discardN;
                  m_logger->warn("Discarding {} bytes to find 0x{:X}", discardN, default_address);
                  continue;
                }
              }
            } else {  // not rx finished
              m_logger->debug("incomplete frame, continue receving");
              waitingForMore = true;
              break;
            }
          } else {
            // illegal funcode

            m_logger->warn("illegal funcode 0x{:2X}", funcodeIn);
            int discardN = circularBuffer.discardUntilNext(default_address);
            remainingBytes -= discardN;
            m_logger->warn("Discarding {} bytes to find 0x{:X}", discardN, default_address);
            continue;
          }
        } else {
          // illegal address
          m_logger->warn("illegal address 0x{:2X}", addressIn);
          int discardN = circularBuffer.discardUntilNext(default_address);
          remainingBytes -= discardN;
          m_logger->warn("Discarding {} bytes to find 0x{:X}", discardN, default_address);
          printRxBuffer();
          continue;
        }
      }
    }else if (_frameType == SLIMSERIAL_FRAME_TYPE_MODBUS_CLIENT_NUM) {
      // src + funcode + data + crc16
      while ((remainingBytes = circularBuffer.availableBytes()) >= 7) {
        // check address. disabled by default
        uint8_t addressIn = circularBuffer.peekAt(0);
        if (applyAddressFilter(addressIn)) {
          // check funcode
          uint16_t funcodeIn = circularBuffer.peekAt(1);
          if (funcodeIn == 0x03 || funcodeIn == 0x06 || funcodeIn == 0x10) {


            uint16_t expectedFrameBytes=0;
            if (funcodeIn == 0x03) {
              expectedFrameBytes  = (uint8_t)(circularBuffer.peekAt(2)) + 5;
            }
            else if(funcodeIn == 0x06){
              expectedFrameBytes  =  circularBuffer.availableBytes();//special treatment for 0x06
            }
            else if (funcodeIn == 0x10) {
              expectedFrameBytes  =  8;
            }

            // got enough rx bytes
            if (remainingBytes >= expectedFrameBytes) {
              // check crc
              if (circularBuffer.calculateCRC(expectedFrameBytes - 2) ==
                  circularBuffer.peekAt_U16(expectedFrameBytes - 2)) {
                _inFrameBytes = expectedFrameBytes;
                circularBuffer.out((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                remainingBytes -= _inFrameBytes;
                parserResult = WS_OK;
                m_logger->debug("[frame parser] [Success] Decoded frame type 3 MODBUS with {} bytes", _inFrameBytes);
                //m_logger->trace("frame content: {}", spdlog::to_hex(std::begin(_inFrame), std::begin(_inFrame) + _inFrameBytes));
                m_totalRxFrames++;

                // dealing frame
                if (frameCallback) {
                  m_tic_frameCallback = getTimeUTC();
                  frameCallback((uint8_t*)(&_inFrame[0]), _inFrameBytes);
                }

                // notificy rx frame handle complete

                // std::unique_lock<std::mutex> lk(frameCompleteMtx);
                // frameCompleteFlag = true;
                // frameCompleteCV.notify_one();
                continue;
              } else {  // bad crc
                if(funcodeIn == 0x06 && circularBuffer.availableBytes()<256){//special treatment for 0x06
                  m_logger->debug("writing reply maybe incomplete, continue receving");
                  waitingForMore = true;
                  break;
                }
                else{
                  m_logger->warn("Bad CRC. calculated 0x{:2X}, recevied 0x{:2X}.",
                      circularBuffer.calculateCRC(expectedFrameBytes - 2),
                      circularBuffer.peekAt_U16(expectedFrameBytes - 2));
                  int discardN = circularBuffer.discardUntilNext(default_address);
                  remainingBytes -= discardN;
                  m_logger->warn("Discarding {} bytes to find 0x{:X}", discardN, default_address);
                  continue;
                }
              }
            } else {  // not rx finished
              m_logger->debug("incomplete frame, continue receving");
              waitingForMore = true;
              break;
            }
          } else {
            // illegal funcode

            m_logger->warn("illegal funcode 0x{:2X}", funcodeIn);
            int discardN = circularBuffer.discardUntilNext(default_address);
            remainingBytes -= discardN;
            m_logger->warn("Discarding {} bytes to find 0x{:X}", discardN, default_address);
            continue;
          }
        } else {
          // illegal address
          m_logger->warn("illegal address 0x{:2X}", addressIn);
          int discardN = circularBuffer.discardUntilNext(default_address);
          remainingBytes -= discardN;
          m_logger->warn("Discarding {} bytes to find 0x{:X}", discardN, default_address);
          printRxBuffer();
          continue;
        }
      }
    }  
    
    else if (_frameType == SLIMSERIAL_FRAME_TYPE_NONE_NUM) {
      // do nothing for rx
      std::unique_lock<std::mutex> lock_(readBufferMtx);
      readBufferCV.notify_one();
      parserResult = WS_OK;
      break;
    }

    // check available bytes again
    remainingBytes = circularBuffer.availableBytes();
    if (((_frameType == SLIMSERIAL_FRAME_TYPE_1_NUM) && remainingBytes < 7) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_1_NUM) && waitingForMore) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_2_NUM) && remainingBytes < 4) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_2_NUM) && waitingForMore) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_MODBUS_SERVER_NUM)) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_MODBUS_CLIENT_NUM)) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_0_NUM) && remainingBytes == 0) ||
        ((_frameType == SLIMSERIAL_FRAME_TYPE_NONE_NUM))) {
      break;
    }
  }

  m_tic_frameParserEnd = getTimeUTC();
  m_logger->debug("[frame parser] [Finished] remaining {} bytes", circularBuffer.availableBytes());
  return parserResult;
}

std::vector<uint8_t> SlimSerialRTDE::SlimSerialRTDEImpl::assembleTxFrameWithAddress(
    uint8_t des, uint8_t fcode, std::vector<uint8_t> const& payload) {
  if (payload.size() > 0) {
    return assembleTxFrameWithAddress(des, fcode, (uint8_t*)payload.data(), payload.size());
  } else {
    return assembleTxFrameWithAddress(des, fcode, NULL, 0);
  }
}
std::vector<uint8_t> SlimSerialRTDE::SlimSerialRTDEImpl::assembleTxFrameWithAddress(
    uint8_t des, uint8_t fcode, uint8_t* pData, uint16_t datasize) {
  std::vector<uint8_t> txframe;

  txframe.reserve(datasize + 7);

  txframe.emplace_back((uint8_t)(default_headers[0]));
  txframe.emplace_back((uint8_t)(default_headers[1]));
  if (_frameType == SLIMSERIAL_FRAME_TYPE_2_NUM) {
    txframe.emplace_back((uint8_t)(datasize + 2));
  } else {
    txframe.emplace_back(des);
    txframe.emplace_back((uint8_t)datasize);
  }
  txframe.emplace_back((uint8_t)fcode);
  for (int i = 0; i < datasize; i++) {
    txframe.emplace_back(*(pData + i));
  }

  uint16_t crc = calculateCRC((uint8_t*)(txframe.data()), txframe.size());
  txframe.emplace_back((uint8_t)(crc & 0xFF));
  txframe.emplace_back((uint8_t)((crc >> 8) & 0xFF));

  return txframe;
}

void SlimSerialRTDE::SlimSerialRTDEImpl::addCRC(uint8_t* pData, uint16_t bytesIncludingHeaders) {
  uint16_t crc = SLIM_CURCULAR_BUFFER::calculateCRC((uint8_t*)pData, bytesIncludingHeaders);
  *(pData + bytesIncludingHeaders) = (uint8_t)(crc & 0xFF);
  *(pData + bytesIncludingHeaders + 1) = (uint8_t)((crc >> 8) & 0xFF);
}

uint16_t SlimSerialRTDE::SlimSerialRTDEImpl::calculateCRC(uint8_t* buffer, uint32_t datasize) {
  return SLIM_CURCULAR_BUFFER::calculateCRC(buffer, datasize);
}

/***************************************************** SlimSerialRTDEImpl  END
 * ***********************************************************************/

/***************************************************** SlimSerialRTDE  Begin
 * ***********************************************************************/

SlimSerialRTDE::SlimSerialRTDE()
    : pimpl_(std::make_unique<SlimSerialRTDEImpl>()),m_logger(spdlog::default_logger()) {
 
    }

SlimSerialRTDE::~SlimSerialRTDE() = default;
SlimSerialRTDE::SlimSerialRTDE(SlimSerialRTDE&& rhs) = default;
SlimSerialRTDE& SlimSerialRTDE::operator=(SlimSerialRTDE&& rhs) = default;

WS_STATUS SlimSerialRTDE::connect(std::string dname, unsigned int baud_, bool autoConnect) {
  
  if (isConnected()) {
     m_logger->warn("Serial port {} is already opened, disconnect and reconnect", pimpl_->m_portname);
     disconnect();
  }  
	try {
	boost::system::error_code err = pimpl_->open(dname, baud_, autoConnect);
	if (!err) {
	return WS_OK;
	} else {
	m_logger->error("Failed to open {}", dname);
	return WS_ERROR;
	}
  }

  catch (...) {
    return WS_ERROR;
  }
 
}
void SlimSerialRTDE::disconnect() {
  pimpl_->close();
  m_logger->info("Serial port {} disconnected", pimpl_->m_portname);
}
bool SlimSerialRTDE::isConnected() { return pimpl_->isOpen(); }

void SlimSerialRTDE::setBaudrate(uint32_t baud) {
  pimpl_->setBaudrate(baud);
  m_logger->info("baudrate changed to {}", baud);
}

void SlimSerialRTDE::enableLogger(std::shared_ptr<spdlog::logger> ext_logger) {
  pimpl_->enableLogger(ext_logger);
}
void SlimSerialRTDE::disableLogger() {
  pimpl_->disableLogger();
}

void SlimSerialRTDE::addRxFrameCallback(std::function<void(uint8_t*, uint32_t)> frameCallbackFunc) {
  pimpl_->frameCallback = frameCallbackFunc;
}

std::vector<uint8_t> SlimSerialRTDE::assembleTxFrameWithAddress(
    uint8_t des, uint8_t fcode, std::vector<uint8_t> const& payload) {
  return pimpl_->assembleTxFrameWithAddress(des, fcode, payload);
}
std::vector<uint8_t> SlimSerialRTDE::assembleTxFrameWithAddress(
    uint8_t des, uint8_t fcode, uint8_t* pData, uint16_t datasize) {
  return pimpl_->assembleTxFrameWithAddress(des, fcode, pData, datasize);
}

std::size_t SlimSerialRTDE::transmitFrame(const std::vector<uint8_t>& txData) { return pimpl_->transmit(txData); }
void SlimSerialRTDE::transmitFrameAsync(const std::vector<uint8_t>& txData) { pimpl_->transmitAsync(txData); }
WS_STATUS SlimSerialRTDE::transmitReceiveFrame(std::vector<uint8_t> const& txData, uint32_t timeoutMS) {
  return pimpl_->transmitReceiveFrame(txData, timeoutMS);
}

std::size_t SlimSerialRTDE::transmitFrame(uint8_t* pData, uint16_t datasize) {
  return pimpl_->transmit(pData, datasize);
}
void SlimSerialRTDE::transmitFrameAsync(uint8_t* pData, uint16_t datasize) { pimpl_->transmit(pData, datasize); }
WS_STATUS SlimSerialRTDE::transmitReceiveFrame(uint8_t* pData, uint16_t datasize, uint32_t timeoutMS) {
  return pimpl_->transmitReceiveFrame(pData, datasize, timeoutMS);
}

uint32_t SlimSerialRTDE::readBuffer(uint8_t* pDes, int nBytes, uint32_t timeoutMS) {
  return pimpl_->readBuffer(pDes, nBytes, timeoutMS);
}
uint32_t SlimSerialRTDE::clearRxBuffer() { return pimpl_->clearRxBuffer(); }
void SlimSerialRTDE::printRxBuffer() { pimpl_->printRxBuffer(); }
std::vector<uint8_t> SlimSerialRTDE::getRxFrame() { return pimpl_->getRxFrame(); }
WS_STATUS SlimSerialRTDE::spinWait(std::function<bool()>&& waitCondition, int timeoutMS, int intervalMS) {
  return pimpl_->spinWait(std::forward<std::function<bool()>>(waitCondition), timeoutMS, intervalMS);
}

void SlimSerialRTDE::addAddressFilter(uint8_t legalAddress) {
  pimpl_->addressFilter[pimpl_->addressFilter_num] = legalAddress;
  if (pimpl_->addressFilter_num < ADDRESS_FILTER_MAX_LEN - 1) {
    pimpl_->addressFilter_num++;
  }
}

void SlimSerialRTDE::setAddressFilter(std::vector<uint8_t> legalAddresses) {
  pimpl_->addressFilter_num =
      legalAddresses.size() > ADDRESS_FILTER_MAX_LEN ? ADDRESS_FILTER_MAX_LEN : legalAddresses.size();
  for (int i = 0; i < pimpl_->addressFilter_num; i++) {
    pimpl_->addressFilter[i] = legalAddresses[i];
  }
}

void SlimSerialRTDE::toggleAddressFilter(bool addressFilterOn) { pimpl_->addressFilterOn = addressFilterOn; }

void SlimSerialRTDE::addFuncodeFilter(uint8_t legalFuncode) {
  pimpl_->funcodeFilter[pimpl_->funcodeFilter_num] = legalFuncode;
  if (pimpl_->funcodeFilter_num < FUNCODE_FILTER_MAX_LEN - 1) {
    pimpl_->funcodeFilter_num++;
  }
}

void SlimSerialRTDE::setFrameLengthFilter(uint16_t _lengthFilterMax) { pimpl_->lengthFilterMax = _lengthFilterMax; }

void SlimSerialRTDE::toggleFuncodeFilter(bool filterOn) { pimpl_->funcodeFilterOn = filterOn; }

void SlimSerialRTDE::setHeader(uint8_t h1, uint8_t h2) {
  pimpl_->default_headers[0] = h1;
  pimpl_->default_headers[1] = h2;
  m_logger->debug("set frame header to {:c} {:c}", h1, h2);
}

uint8_t SlimSerialRTDE::getHeader(uint8_t index) { return pimpl_->default_headers[index]; }
void SlimSerialRTDE::setTxAddress(uint8_t srcAddress) { pimpl_->default_address = srcAddress; }
uint8_t SlimSerialRTDE::getTxAddress() { return pimpl_->default_address; }
uint8_t SlimSerialRTDE::getFrameType() { return pimpl_->getFrameType(); }
void SlimSerialRTDE::setFrameType(uint8_t ftype) { pimpl_->setFrameType(ftype); }

void SlimSerialRTDE::addCRC(uint8_t* pData, uint16_t bytesIncludingHeaders) {
  pimpl_->addCRC(pData, bytesIncludingHeaders);
}
uint16_t SlimSerialRTDE::calculateCRC(uint8_t* data, uint16_t datasize) { return pimpl_->calculateCRC(data, datasize); }

std::string SlimSerialRTDE::toHexString(std::vector<uint8_t>& data) { return pimpl_->toHexString(data); }
std::string SlimSerialRTDE::toHexString(uint8_t* pdata, uint32_t databytes) {
  return pimpl_->toHexString(pdata, databytes);
}

uint32_t SlimSerialRTDE::availableBytes() { return pimpl_->circularBuffer.availableBytes(); }

int SlimSerialRTDE::getTotalTxFrames() { return pimpl_->m_totalTxFrames; }

int SlimSerialRTDE::getTotalRxFrames() { return pimpl_->m_totalRxFrames; }

int SlimSerialRTDE::getTotalTxBytes() { return pimpl_->m_totalTxBytes; }

int SlimSerialRTDE::getTotalRxBytes() { return pimpl_->m_totalRxBytes; }

uint64_t SlimSerialRTDE::getLastTxRxFrameTime() { return pimpl_->m_lastTxRxTime; }

uint64_t SlimSerialRTDE::getTimeUTC() { return pimpl_->getTimeUTC(); }

uint64_t SlimSerialRTDE::getTicm_tic_frameStart() { return pimpl_->m_tic_frameStart; }

uint64_t SlimSerialRTDE::getTicm_tic_rxDataCallback() { return pimpl_->m_tic_rxDataCallback; }
uint64_t SlimSerialRTDE::getTicm_tic_frameParser() { return pimpl_->m_tic_frameParser; }
uint64_t SlimSerialRTDE::getTicm_tic_frameComplete() { return pimpl_->m_tic_frameComplete; }
uint64_t SlimSerialRTDE::getTicm_tic_frameParserEnd() { return pimpl_->m_tic_frameParserEnd; }

void SlimSerialRTDE::setServer(bool isServer) { pimpl_->m_isServer = isServer; }