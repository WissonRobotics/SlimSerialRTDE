// #include "includes.h"
// #define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
  
#include "SlimSerialRTDE/SlimSerialRTDE.h"
#include <iostream>
#include <string>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <thread> 
#include <spdlog/spdlog.h>

class SlimSerialServerTest :public SlimSerialRTDE {
public:
	SlimSerialServerTest() {
		addRxFrameCallback(std::bind(&SlimSerialServerTest::frameCallback, this, std::placeholders::_1,std::placeholders::_2));

		addAddressFilter(0x10);
		toggleAddressFilter(true);

		addFuncodeFilter(0xA0);
		toggleFuncodeFilter(true);

		setFrameLengthFilter(100);

		std::vector<uint8_t>payload;
		for (int i = 0; i < 60; i++) {
			payload.emplace_back(i);
		}
		m_txframe = assembleTxFrameWithAddress(0x12, 0xA2, payload);

		resetFrameCount();

		setServer(true);
	};

	void frameCallback(uint8_t *pdata,uint32_t databytes) {
 
		//printf("Handling Rx frame with funcode %x\n", pdata[4]);

		switch (pdata[4]) {
		case 0xA0: {
 			transmitFrameGood();
			//transmitFrameRandom();
		}
		}
	}

	void transmitFrameRandom() {
		if (rand() % 4) {
			transmitFrameGood();
		}
		else {
			transmitFrameBad();
		}
	}

	void transmitFrameBad() {
		int n = rand() % 5;
		if (n==0) {
			transmitFrameBadHeader();
		}
		else if (n == 1) {
			transmitFrameBadAddress();
		}
		else if (n == 2) {
			transmitFrameBadFuncode();
		}
		else if (n == 3) {
			transmitFrameBadLength();
		}
		else if (n == 4) {
			transmitFrameBadCRC();
		}
	}

	void transmitFrameGood() {
		totalTxFrameGood++;
		transmitFrame(m_txframe);
	}

	void transmitFrameBadHeader() {
		transmitFrametempered(m_txframe, 1, 0xEE);
	}

	void transmitFrameBadAddress() {
		transmitFrametempered(m_txframe, 2, 0xEE);
	}

	void transmitFrameBadFuncode() {
		transmitFrametempered(m_txframe, 3, 0xEE);
	}
	void transmitFrameBadLength() {
		transmitFrametempered(m_txframe, 4, 0xEE);
	}
	void transmitFrameBadCRC() {
		transmitFrametempered(m_txframe, m_txframe.size()-2, m_txframe[m_txframe.size()-2]+1);
	}

	void transmitFrametempered(std::vector<uint8_t>& txframe, uint8_t badInd, uint8_t badData) {
		uint8_t temp = txframe[badInd];
		txframe[badInd] = badData;
		transmitFrame(m_txframe);
		totalTxFrameBad++;
		txframe[badInd] = temp;
	}
	void resetFrameCount() {
		totalTxFrameGood = 0;
		totalTxFrameBad = 0;
	};
 
	int totalTxFrameGood;
	int totalTxFrameBad;
	std::vector<uint8_t>  m_txframe;
};


class SlimSerialClientTest :public SlimSerialRTDE {
public:
	uint32_t txrxtimeout = 8;
	SlimSerialClientTest() {
		addRxFrameCallback(std::bind(&SlimSerialClientTest::frameCallback, this, std::placeholders::_1,std::placeholders::_2));

		addAddressFilter(0x12);
		toggleAddressFilter(true);

		addFuncodeFilter(0xA2);
		toggleFuncodeFilter(true);

		setFrameLengthFilter(200);

		std::vector<uint8_t>payload;
		for (int i = 0; i < 30; i++) {
			payload.emplace_back(i);
		}
		m_txframe = assembleTxFrameWithAddress(0x10, 0xA0, payload);

		resetFrameCount();
	};

	void frameCallback(uint8_t *pdata,uint32_t databytes) {
		//printf("Handling Rx frame with funcode %x\n", rxFrame[4]);

		// switch (pdata[4]) {
		// 	case 0x04: {

		// 		if (rand() % 4) {
		// 			transmitFrameGood();
		// 		}
		// 		else {
		// 			transmitFrameBad();
		// 		}
		// 		break;
		// 	}
		// }
	}

	void transmitFrameRandom() {
		if (rand() % 4) {
			transmitFrameGood();
		}
		else {
			transmitFrameBad();
		}
	}

	void transmitFrameBad() {
		int n = rand() % 5;
		if (n == 0) {
			transmitFrameBadHeader();
		}
		else if (n == 1) {
			transmitFrameBadAddress();
		}
		else if (n == 2) {
			transmitFrameBadFuncode();
		}
		else if (n == 3) {
			transmitFrameBadLength();
		}
		else if (n == 4) {
			transmitFrameBadCRC();
		}
	}

	void transmitFrameGood() {
		totalTxFrameGood++;
		auto ret = transmitReceiveFrame(m_txframe,txrxtimeout);
		if(ret!=WS_OK){
			// spdlog::warn("timeout rxrx");
		}
	}

	void transmitFrameBadHeader() {
		transmitFrametempered(m_txframe, 1, 0xEE);
	}

	void transmitFrameBadAddress() {
		transmitFrametempered(m_txframe, 2, 0xEE);
	}

	void transmitFrameBadFuncode() {
		transmitFrametempered(m_txframe, 3, 0xEE);
	}
	void transmitFrameBadLength() {
		transmitFrametempered(m_txframe, 4, 0xEE);
	}
	void transmitFrameBadCRC() {
		transmitFrametempered(m_txframe, m_txframe.size() - 2, m_txframe[m_txframe.size() - 2] + 1);
	}

	void transmitFrametempered(std::vector<uint8_t>& txframe, uint8_t badInd, uint8_t badData) {
		uint8_t temp = txframe[badInd];
		txframe[badInd] = badData;
		transmitReceiveFrame(m_txframe,txrxtimeout);
		totalTxFrameBad++;
		txframe[badInd] = temp;
	}

	void resetFrameCount() {

		totalTxFrameGood = 0;
		totalTxFrameBad = 0;
	};
	int totalTxFrameGood=0;
	int totalTxFrameBad=0;
	std::vector<uint8_t>  m_txframe;
};


std::array<int,8> testCommunications(int testTimeMs=1000) {
	std::array<int,4> expectRes = {0,0,0,0};
	

	SlimSerialServerTest server;
	SlimSerialClientTest client;
	
	std::unique_ptr<std::jthread> virtualPortThread = std::make_unique<std::jthread>([](){
		//create virtual ports
		int returnCode = system("socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1 &");
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		if (returnCode != 0) {
			spdlog::error("[Monos Arm Simulator] [Failed] to create virtual ports");
		}
		else{
			spdlog::info("[Monos Arm Simulator] [Success] to create virtual port /tmp/ttyV0 and /tmp/ttyV1");
		}
	}) ;
 

	//server connect to virtual ports
	if (server.connect("/tmp/ttyV0", 1000000,true) == WS_OK) {
		spdlog::info("serial port connectted to /tmp/ttyV0 at 1000000\r\n");
		server.setFrameType(1);
	}
	else {
		spdlog::error("serial port /tmp/ttyV0 connection error\r\n");
 
	}
	server.disableLogger();


	//Client connect to virtual ports
	if (client.connect("/tmp/ttyV1", 1000000,true) == WS_OK) {
		spdlog::info("serial port connectted to /tmp/ttyV1 at 1000000\r\n");
		client.setFrameType(1);
	}
	else {
		spdlog::error("serial port /tmp/ttyV1 connection error\r\n");
 
	}
	client.disableLogger();
  
	//start test
	auto tic = client.getTimeUTC();
	while (1) {
 
		auto curTimeStamp = std::chrono::system_clock::now();
		auto toc = client.getTimeUTC() - tic;
        if(toc>testTimeMs){
            break;
        }

		client.transmitFrameRandom();
		client.transmitFrameGood();


		spdlog::info("{}  CT/SR/ERR= {}/{}/{}  ST/CR/ERR= {}/{}/{}   FCT/FCG/FSR/ERR = {}/{}/{}/{} FST/FCR/ERR = {}/{}/{} , roundtrip={}", 

			toc,

			client.getTotalTxBytes(),
			server.getTotalRxBytes(),
			client.getTotalTxBytes()-server.getTotalRxBytes(),

			server.getTotalTxBytes(),
			client.getTotalRxBytes(),
			server.getTotalTxBytes()-client.getTotalRxBytes(),

			client.getTotalTxFrames(),
			client.totalTxFrameGood,
			server.getTotalRxFrames(),
			client.totalTxFrameGood - server.getTotalRxFrames(),

			server.getTotalTxFrames(),
			server.totalTxFrameGood,
			client.getTotalRxFrames(),
			server.totalTxFrameGood - client.getTotalRxFrames(),

			client.getLastTxRxFrameTime()

			// server.getTicm_tic_rxDataCallback() - client.getTicm_tic_frameStart(),
			// server.getTicm_tic_frameParserEnd() - client.getTicm_tic_frameStart(),
			 
			// client.getTicm_tic_rxDataCallback() - client.getTicm_tic_frameStart(),
			// client.getTicm_tic_frameParserEnd() - client.getTicm_tic_frameStart(),
			// client.getTicm_tic_frameComplete() - client.getTicm_tic_frameStart()

		);
		std::this_thread::sleep_until(curTimeStamp+std::chrono::milliseconds(10));
	 
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(200));


	return {client.getTotalTxBytes(),
		server.getTotalTxBytes(),
		client.totalTxFrameGood,
		server.totalTxFrameGood,
		client.getTotalTxBytes()-server.getTotalRxBytes(),
		server.getTotalTxBytes()-client.getTotalRxBytes(),
		client.totalTxFrameGood - server.getTotalRxFrames(),
		server.totalTxFrameGood - client.getTotalRxFrames()};
}
  
 
TEST_CASE("basic serial TransmitReceive test ", "[TransmitReceive]") {
	
	printf("Need to manually start virtual serial port in another terminal !!!!!!!!!!! ");
    std::array<int,8> res = testCommunications(2000);

    REQUIRE(res[0]>0);
	REQUIRE(res[1]>0);
	REQUIRE(res[2]>0);
	REQUIRE(res[3]>0);
	REQUIRE(res[4]==0);
	REQUIRE(res[5]==0);
	REQUIRE(res[6]==0);
	REQUIRE(res[7]==0);
 
}
 