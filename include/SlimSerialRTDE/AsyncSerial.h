#pragma once
#include <functional>
#include <thread>
#include <array>
#include <future>
#include <chrono>
#include "boost/asio.hpp"

#include <string.h>
#include "stdio.h" 
#include "SlimSerialRTDE/slimCircularBuffer.h"

#include <spdlog/spdlog.h>
#include "spdlog/sinks/stdout_color_sinks.h"

class AsyncSerial
{
	 
	static constexpr int DEFAULT_CIRCULAR_BUF_SIZE = 4096;


public:
	typedef boost::asio::serial_port_base::flow_control::type flowControlType;
	typedef boost::asio::serial_port_base::parity::type parityType;
	typedef boost::asio::serial_port_base::stop_bits::type stopBitsType;
	typedef boost::system::errc::errc_t errorCode;


public:

	AsyncSerial();
	~AsyncSerial();
	boost::system::error_code open(std::string, unsigned int = 115200,bool autoConnect=true);
	bool isOpen() const;
	void close();
	void setBaudrate(uint32_t baud);
 
	void setAutoConnectPeriod(int autoReconnectTimeMs=1000);

	void enableLogger(std::shared_ptr<spdlog::logger> ext_logger=spdlog::default_logger());
	void disableLogger();

	//uint32_t readBuffer(uint8_t* pDes, int nBytes = 1);

	//std::future_status receive(uint8_t* pDes, int nBytes, unsigned int timeout);

	std::size_t	transmit(uint8_t *pData,uint16_t datasize);
	std::size_t transmit(const std::vector<uint8_t>& v);

	void transmitAsync(uint8_t *pData,uint16_t datasize);
	void transmitAsync(const std::vector<uint8_t>& v);

	void addRxDataCallback(std::function<void(void)> rxIdleCallbackExt);


	void resetTxRxCount();

	std::string toHexString(const std::vector<uint8_t>& data);
	std::string toHexString(uint8_t *pdata,uint32_t databytes);


	inline uint64_t getTimeUTC() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	};
	int m_totalRxBytes=0;
	int m_totalTxBytes=0; 
	int m_totalTxFrames=0;
	SLIM_CURCULAR_BUFFER circularBuffer;
	std::string m_portname ="";
	unsigned int m_baudrate=115200;
	std::shared_ptr<spdlog::logger> m_logger=spdlog::default_logger(); 
private:
	void stopIOContextThread();

	

	std::array<uint8_t,DEFAULT_CIRCULAR_BUF_SIZE> m_txBuffer;

	boost::asio::io_context io_context;
	boost::asio::serial_port serial_port;
	boost::asio::io_context::work serial_work;
	std::array<uint8_t, DEFAULT_CIRCULAR_BUF_SIZE> single_buf;
	uint8_t cbBuf[DEFAULT_CIRCULAR_BUF_SIZE];

	int m_autoReconnectTimeMs=1000;
	std::unique_ptr<std::jthread> ioContextThread;



	mutable std::mutex readBufferMtx;
	mutable std::mutex errMtx;

	int error_value=0;
	unsigned int timeoutVal = 60000;
	mutable std::mutex writeMtx;
	std::condition_variable writeCv;
	bool writeLocked = false;

	//reconnect
	bool needToReconnectFlag=false;
	std::unique_ptr<std::jthread> reconnectThread;
	std::mutex reconnectMutex;
	std::condition_variable reconnectCV;

	 
	std::function<void(void)> rxDataCallback;
	//dp::thread_safe_queue<SpeakMsgMeta> msgQueue;

	void startAutoConnect(int autoReconnectTimeMs = 1000);

	void stopAutoConnect();

	void triggerReconnect();

	boost::system::error_code doOpen(std::string, unsigned int = 115200, flowControlType = flowControlType::none, unsigned int = 8, parityType = parityType::none, stopBitsType = stopBitsType::one);

	void doClose();
	void setError(const int error_value);
	int getError() const;
	void asyncReadHandler(boost::system::error_code const& error, size_t bytes_transferred);

	void asyncWriteHandler(const boost::system::error_code& error, std::size_t bytes_transferred);

	bool m_closing_state=false;
};
