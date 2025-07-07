#pragma once

#include <string>
#include <functional>
#include <vector>
#include <memory>
#include <spdlog/spdlog.h>
#include "spdlog/sinks/stdout_color_sinks.h"
enum WS_STATUS {
	WS_OK,
	WS_TIMEOUT,
	WS_ABORT,
	WS_FAIL,
	WS_ERROR,
	WS_QUIT,
	WS_NONE
};


#define INTERNAL_MAX_FRAME_SIZE 4096
#pragma pack(1)
typedef struct SLIMSERIAL_FRAME_TYPE_0_TAG {
	union {
		uint8_t data8[INTERNAL_MAX_FRAME_SIZE];
		int16_t data16[INTERNAL_MAX_FRAME_SIZE/2];
		uint16_t dataU16[INTERNAL_MAX_FRAME_SIZE/2];
	} payload;

}SLIMSERIAL_FRAME_TYPE_0;

typedef struct SLIMSERIAL_FRAME_TYPE_1_TAG {

	uint8_t header[2];
	uint8_t src;
	uint8_t payloadBytes;
	uint8_t funcode;
	union {
		uint8_t data8[INTERNAL_MAX_FRAME_SIZE];
		int16_t data16[INTERNAL_MAX_FRAME_SIZE/2];
		uint16_t dataU16[INTERNAL_MAX_FRAME_SIZE/2];
	} payload;

}SLIMSERIAL_FRAME_TYPE_1;


typedef struct SLIMSERIAL_FRAME_TYPE_2_TAG {

	uint8_t header[2];
	uint8_t payloadBytes;
	uint8_t funcode;
	union {
		uint8_t data8[INTERNAL_MAX_FRAME_SIZE];
		int16_t data16[INTERNAL_MAX_FRAME_SIZE/2];
		uint16_t dataU16[INTERNAL_MAX_FRAME_SIZE/2];
	} payload;

}SLIMSERIAL_FRAME_TYPE_2;

typedef struct SLIMSERIAL_FRAME_TYPE_3_MODBUS_TAG {

	uint8_t address;
	uint8_t funcode;
	union {
		uint8_t data8[INTERNAL_MAX_FRAME_SIZE];
		int16_t data16[INTERNAL_MAX_FRAME_SIZE/2];
		uint16_t dataU16[INTERNAL_MAX_FRAME_SIZE/2];
	} payload;

}SLIMSERIAL_FRAME_TYPE_MODBUS;
#pragma pack()

typedef enum
{
  SLIMSERIAL_FRAME_TYPE_0_NUM				= 0,
  SLIMSERIAL_FRAME_TYPE_1_NUM				= 1,
  SLIMSERIAL_FRAME_TYPE_2_NUM				= 2,
  SLIMSERIAL_FRAME_TYPE_MODBUS_SERVER_NUM 	= 3,
  SLIMSERIAL_FRAME_TYPE_MODBUS_CLIENT_NUM 	= 4,

  SLIMSERIAL_FRAME_TYPE_NONE_NUM        	= 99

}SLIMSERIAL_FRAME_TYPE_NUM;
;
 
#define ADDRESS_FILTER_MAX_LEN 10
#define FUNCODE_FILTER_MAX_LEN 20


class SlimSerialRTDE {
public:
	SlimSerialRTDE();
	~SlimSerialRTDE();
	SlimSerialRTDE(SlimSerialRTDE&& rhs);
	SlimSerialRTDE& operator=(SlimSerialRTDE&& rhs);

	WS_STATUS connect(std::string dname,
		unsigned int baud_ = 1000000,bool autoConnect=true);
	void disconnect();
	bool isConnected(); 
 

	void setBaudrate(uint32_t baud = 1000000);

	void addRxFrameCallback(std::function<void(uint8_t *,uint32_t)> frameCallbackFunc);
 

	std::vector<uint8_t> assembleTxFrameWithAddress(uint8_t des, uint8_t fcode, std::vector<uint8_t> const& payload = {});
	std::vector<uint8_t> assembleTxFrameWithAddress(uint8_t des, uint8_t fcode, uint8_t *pData,uint16_t datasize);

	



	std::size_t transmitFrame(std::vector<uint8_t> const& txData);
 	std::size_t transmitFrame(uint8_t *pData,uint16_t datasize);

	void transmitFrameAsync(std::vector<uint8_t> const& txData);
	void transmitFrameAsync(uint8_t *pData,uint16_t datasize);
	
	WS_STATUS transmitReceiveFrame(std::vector<uint8_t> const& txData, uint32_t timeoutMS = 20);
	WS_STATUS transmitReceiveFrame(uint8_t *pData,uint16_t datasize, uint32_t timeoutMS = 20);
 
 	template <size_t N>
	std::size_t transmitFrame(std::array<uint8_t,N> &txArray){
		return transmitFrame(&txArray[0],txArray.size());
	}

	template <size_t N>
	void transmitFrameAsync(std::array<uint8_t,N> &txArray){
		 transmitFrameAsync(&txArray[0],txArray.size());
	}

	template <size_t N>
	WS_STATUS transmitReceiveFrame(std::array<uint8_t,N> &txArray, uint32_t timeoutMS){
		return transmitReceiveFrame(&txArray[0],txArray.size(), timeoutMS);
	}


	uint32_t readBuffer(uint8_t* pDes, int nBytes = 1, uint32_t timeoutMS = 360000);
	

	uint32_t clearRxBuffer();
	void printRxBuffer();
	std::vector<uint8_t> getRxFrame();

	void addAddressFilter(uint8_t legalAddress);

	void setAddressFilter(std::vector<uint8_t> legalAddresses);
 
	void toggleAddressFilter(bool addressFilterOn);

	void addFuncodeFilter(uint8_t legalFuncode);
 
	void toggleFuncodeFilter(bool funcodeIn);

	void setServer(bool );

	void enableLogger(std::shared_ptr<spdlog::logger> ext_logger=spdlog::default_logger());
	void disableLogger();
 
	void setHeader(uint8_t h1, uint8_t h2);
	uint8_t getHeader(uint8_t index);
	void setTxAddress(uint8_t srcAddress);
	uint8_t getTxAddress();
	void setFrameLengthFilter(uint16_t _lengthFilterMax);
	uint8_t getFrameType();
	void setFrameType(uint8_t ftype);
 	WS_STATUS  spinWait(std::function<bool()> &&waitCondition,int timeoutMS,int intervalMS=0);
	
	void addCRC(uint8_t* pData, uint16_t bytesIncludingHeaders);
	uint16_t calculateCRC(uint8_t* data, uint16_t datasize);

	std::string toHexString(std::vector<uint8_t>& data);
	std::string toHexString(uint8_t *pdata,uint32_t databytes);
	uint32_t availableBytes();


	int getTotalTxFrames();
	int getTotalRxFrames();
	int getTotalTxBytes();
	int getTotalRxBytes();
 
	uint64_t getLastTxRxFrameTime();
	uint64_t getTimeUTC();
	uint64_t getTicm_tic_frameStart();
	uint64_t getTicm_tic_rxDataCallback();
	uint64_t getTicm_tic_frameParser();
	uint64_t getTicm_tic_frameComplete();
	uint64_t getTicm_tic_frameParserEnd();

	std::shared_ptr<spdlog::logger> m_logger; 
private:
	
	class SlimSerialRTDEImpl;
	std::unique_ptr<SlimSerialRTDEImpl> pimpl_;
};
 