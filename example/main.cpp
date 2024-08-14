#include "SlimSerialRTDE.h"
#include "cliParser.h"
#include <iostream>
#include <string>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include "loguru.hpp"
#include "SlimSerialServer.h" 
#include "Maxwell_SoftArm_SerialClient.h"
#include <memory>
#include <thread>
#include "monos_arm.h" 
class SlimSerialServerTest :public SlimSerialRTDE {
public:
	SlimSerialServerTest() {
		addRxFrameCallback(std::bind(&SlimSerialServerTest::frameCallback, this, std::placeholders::_1,std::placeholders::_2));

		addAddressFilter(0x12);
		toggleAddressFilter(true);

		addFuncodeFilter(0x05);
		toggleFuncodeFilter(true);

		setFrameLengthFilter(100);

		std::vector<uint8_t>payload;
		for (int i = 0; i < 60; i++) {
			payload.emplace_back(i);
		}
		m_txframe = assembleTxFrameWithAddress(0x012, 0x05, payload);

		resetFrameCount();

		setServer(true);
	};

	void frameCallback(uint8_t *pdata,uint32_t databytes) {
		totalRxFrame += 1;
		//printf("Handling Rx frame with funcode %x\n", pdata[4]);

		switch (pdata[4]) {
		case 0x05: {
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
		totalRxFrame = 0;
		totalTxFrameGood = 0;
		totalTxFrameBad = 0;
	};
	uint32_t totalRxFrame;
	uint32_t totalTxFrameGood;
	uint32_t totalTxFrameBad;
	std::vector<uint8_t>  m_txframe;
};


class SlimSerialClientTest :public SlimSerialRTDE {
public:
	uint32_t txrxtimeout = 8;
	SlimSerialClientTest() {
		addRxFrameCallback(std::bind(&SlimSerialClientTest::frameCallback, this, std::placeholders::_1,std::placeholders::_2));

		addAddressFilter(0x12);
		toggleAddressFilter(false);

		addFuncodeFilter(0x05);
		toggleFuncodeFilter(false);

		setFrameLengthFilter(200);

		std::vector<uint8_t>payload;
		for (int i = 0; i < 30; i++) {
			payload.emplace_back(i);
		}
		m_txframe = assembleTxFrameWithAddress(0x00, FUNC_CUSTOM_MONO_PC_ACTION, payload);

		resetFrameCount();
	};

	void frameCallback(uint8_t *pdata,uint32_t databytes) {
		totalRxFrame += 1;
		//printf("Handling Rx frame with funcode %x\n", rxFrame[4]);

		//switch (pdata[4]) {
		//	case 0x04: {

		//		if (rand() % 4) {
		//			transmitFrameGood();
		//		}
		//		else {
		//			transmitFrameBad();
		//		}
		//		break;
		//	}
		//}
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
			LOG_F(ERROR,"timeout rxrx");
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
		totalRxFrame = 0;
		totalTxFrameGood = 0;
		totalTxFrameBad = 0;
	};
	uint32_t totalRxFrame=0;
	uint32_t totalTxFrameGood=0;
	uint32_t totalTxFrameBad=0;
	std::vector<uint8_t>  m_txframe;
};
 


static void sigHandler(int sig) {
	printf("Signal %d received, exiting\n", sig);
	 
	exit(0);
};
int main(int argc, char** argv) {
	

	signal(SIGINT, sigHandler);

    /*************************** Command line arguments will have higher priority over configuration file **************************/
    cli::Parser parser(argc, argv);

	parser.set_optional<std::string>("p", "port","COM3",  "the serial port");
    parser.set_optional<int>("b", "baudrate", 115200, "baudrate of the application code");
    parser.set_optional<int>("t", "timeout", 10, " timeout s");
	parser.set_optional<std::string>("d", "debug", "trace", "logger level");
	parser.set_optional<int>("s", "server", 0, "if it is server");
 

    parser.run_and_exit_if_error(); 
    std::string portnameCli = parser.get<std::string>("p");
    int appBaudrateCli = parser.get<int>("b"); 
    int timeoutCli = parser.get<int>("t");
    std::string debuglevelCli = parser.get<std::string>("d");
 

	appBaudrateCli = 3000000;
	debuglevelCli = "warning";

	#if defined(__linux__)
		portnameCli = "/tmp/ttyV0";
	#else
		portnameCli = "COM3";
	#endif
	SlimSerialServer server;
	if (server.connect(portnameCli, appBaudrateCli,true) == WS_OK) {
		printf("serial port connectted to %s at %d\r\n", portnameCli.c_str(), appBaudrateCli);
		server.setFrameType(1);
	}
	else {
		printf("serial port  %s connection error\r\n", portnameCli.c_str());
		return 0;
	}

	//Client
	#if defined(__linux__)
		portnameCli = "/tmp/ttyV1";
	#else
		portnameCli = "COM4";
	#endif
	MonosArm arm;
	arm.Connect(portnameCli,appBaudrateCli);
	Maxwell_SoftArm_SerialClient &client=arm.rtde_client_;

	// if (client.connect(portnameCli, appBaudrateCli,true) == WS_OK) {
	// 	printf("serial port connectted to %s at %d\r\n", portnameCli.c_str(), appBaudrateCli);
	// 	client.setLoggerLevel(debuglevelCli);
	// 	client.setFrameType(1); 
	// }
	// else {
	// 	printf("serial port  %s connection error\r\n", portnameCli.c_str());
	// 	return 0;
	// }

	printf("start serial test\r\n");
	auto tic = client.getTimeUTC();
	auto lastToc = tic;
	loguru::g_stderr_verbosity = 0;
	loguru::g_preamble_uptime = false;
	loguru::g_preamble_thread = false;
	 
	arm.addLoggingFile("serialTestStatistics.log","debug"); 
	int a=0;
	int tt=0;
	// std::array<float,6> jointc1={.1,.1,.1,.1,.1,.1};
	// std::array<float,6> jointc2={.2,.2,.2,.2,.2,.2};
	//std::vector<std::unique_ptr<std::jthread>> ts;
	// ts.push_back(std::move(std::make_unique<std::jthread>(
    //     [&](std::stop_token stop_token)
    //     {
    //         while (!stop_token.stop_requested())
    //         {
    //           auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(client.m_FrameTxRxPeriodMs*5);
    //           client.commandJoint(jointc1);
    //           std::this_thread::sleep_until(timeoutPoint);
    //         };
    //     }
    // )));

	// ts.push_back(std::move(std::make_unique<std::jthread>(
    //     [&](std::stop_token stop_token)
    //     {
    //         while (!stop_token.stop_requested())
    //         {
    //           auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(client.m_FrameTxRxPeriodMs*5);
    //           client.commandJoint(jointc2);
    //           std::this_thread::sleep_until(timeoutPoint);
    //         };
    //     }
    // )));
	std::array<float,6> targetq={0.2,0.1,0.2,0.1,1.7,0.0};
 
	// arm.MoveJoint(targetq,timeoutCli);
	// auto q = targetq-targetq;
	// arm.MoveJointAsync(q,timeoutCli,[](WS_STATUS &actionResutl){LOG_F(INFO,"haha CompleteCallback"); });
	// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// arm.MoveJointAsync(q,timeoutCli,[](WS_STATUS &actionResutl){LOG_F(INFO,"haha CompleteCallback"); });
	 int qq=0;


	// arm.ValveOn();
	// arm.MagnetOff();
	// arm.MagnetOn();
	// arm.ValveOff();
	arm.ValveOn();

	// arm.GunElongate();
	// arm.GunContract();
 
	// arm.GunLock();
	// arm.GunUnlock();

	// arm.GripperOpen();
	// arm.GripperClose();


	arm.MovePitch(0.1);
	arm.MovePitchAsync(0.2);

	sleep(5);

	arm.ErectUp();
	arm.BendDown();
	arm.ErectUpAsync();
	arm.BendDownAsync();
	arm.MoveJointPrecisely(targetq,0.001);


	arm.GunLock();
	arm.GunUnlock(); 

	std::array<float,6> aaa={0};
	aaa = std::array<float,6>({1,2,3,4,5,6});


	while (1) {
		auto tic_local = TIC();
		auto toc = client.getTimeUTC() - tic;
		auto period = toc - lastToc;
		lastToc = toc;
		
		//client.transmitFrameRandom();

		//client.transmitFrameGood();
		// if(tt++>10){
		// 	client.commandPitchUp();

		// 	tt=0;
		// }
		if(toc>7000 && qq==0){
			arm.MoveJoint(targetq,timeoutCli);
			//arm.stopMoveJoint();
			qq=1;
		}
		LOG_F(ERROR,"%ld  CT/SR= %d/%d/%d  ST/CR= %d/%d/%d   FCT/FSR = %ld/%ld/%ld FST/FCR = %ld/%ld/%ld , %ld ,    %ld %ld  %ld %ld %ld", 
		//printf("\r %d  CT/SR= %d/%d  ST/CR= %d/%d   CT/CTG/SR = %d/%d/%d  ST/SRG/CR = %d/%d/%d , %d", 

			toc,


			client.getTotalTxBytes(),
			server.getTotalRxBytes(),
			client.getTotalTxBytes()-server.getTotalRxBytes(),


			server.getTotalTxBytes(),
			client.getTotalRxBytes(),
			server.getTotalTxBytes()-client.getTotalRxBytes(),

			 
			client.getTotalTxFrames(),
			server.getTotalRxFrames(),
			client.getTotalTxFrames() - server.getTotalRxFrames(),

			server.getTotalTxFrames(),
			client.getTotalRxFrames(),
			server.getTotalTxFrames() - client.getTotalRxFrames(),

			client.getLastTxRxFrameTime(),

 
			server.getTicm_tic_rxDataCallback() - client.getTicm_tic_frameStart(),
			server.getTicm_tic_frameParserEnd() - client.getTicm_tic_frameStart(),
			 
			client.getTicm_tic_rxDataCallback() - client.getTicm_tic_frameStart(),
			client.getTicm_tic_frameParserEnd() - client.getTicm_tic_frameStart(),
			client.getTicm_tic_frameComplete() - client.getTicm_tic_frameStart()
	 

		);



		std::this_thread::sleep_until(tic_local + std::chrono::milliseconds(10));
	 
	}
 
 
}
