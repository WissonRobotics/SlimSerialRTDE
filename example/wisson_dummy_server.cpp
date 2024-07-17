#include "wisson_dummy_server.h"
#include <stdlib.h> 
#include <thread>      
#include <chrono>      
#include "cliParser.h"
#include "ini.h"
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif



WISSON_DUMMY_SERVER::WISSON_DUMMY_SERVER(std::string loggerName) :slimSerial(loggerName) {
	frameCallbackFunc = std::bind(&WISSON_DUMMY_SERVER::frameCallback, this, std::placeholders::_1);

	slimSerial.addRxFrameCallback(frameCallbackFunc);

	taskError = error_none;
	taskStatus = status_idle;
	taskStatusLast = taskStatus;
	chargeWellAck = 0;
	command = cmd_query;

	command_string.push_back("query");
	command_string.push_back("autoChargeStart");
	command_string.push_back("autoDischarge");
	command_string.push_back("emergencyStop");
	command_string.push_back("home");
	command_string.push_back("manualCharge");
	command_string.push_back("manualReturn");
	command_string.push_back("confirmChargedWell");
 
	status_string.push_back("status_idle");
	status_string.push_back("status_autoCharge");
	status_string.push_back("status_Charging");
	status_string.push_back("status_autoDischarge");
	status_string.push_back("status_manualCharge");
	status_string.push_back("status_manualReturn");
	status_string.push_back("status_stopped");
	status_string.push_back("status_homing");
	status_string.push_back("status_error");

}
WISSON_DUMMY_SERVER::~WISSON_DUMMY_SERVER() {
	slimSerial.disconnect();
}


int WISSON_DUMMY_SERVER::setup(std::string portname) {
	m_portname = portname;
	if ((slimSerial.connect(m_portname, 115200)) == true) {
		printf("serial port connectted to %s at %d\r\n", m_portname.c_str(), 115200);
		slimSerial.setLoggerLevel("info");
		slimSerial.setFrameType(1);
		return COM_OK;
	}
	else {
		printf("serial port  %s connection error\r\n", m_portname.c_str());
		return COM_ERROR;
	}
	 
}

void WISSON_DUMMY_SERVER::frameCallback(std::vector<uint8_t> &rxFrame){
	
	 
	if (rxFrame[4] == CONTROL_FUNC) {

		command = (uint16_t)((uint16_t)(rxFrame[5]) | (uint16_t)(rxFrame[6] << 8));

		printf("Get Command %d: %s\r\n",command, command_string[command].c_str());
		
		switch (command) {
		
		case cmd_query: {
			
			response();
			break;
		}
		case cmd_autoChargeStart: {
			if (taskStatus == status_idle) {
				taskStatus = status_autoCharge;
				response();
			}
			else {
				printf("Command ignored. Not in idle state\r\n");
			}
			break;
		}



		case cmd_autoDischarge: {
			if (taskStatus == status_Charging) {
				taskStatus = status_autoDischarge;
				response();
			}
			else {
				printf("Command ignored. Not in Charging state\r\n");
			}
			break;
		}
		case cmd_emergencyStop: {
			taskStatus = status_stopped;
			response();
			break;
		}

		case cmd_home: {
			taskStatus = status_homing;
			response();

			break;
		}

		case cmd_manualCharge: {
			if (taskStatus == status_idle) {
				taskStatus = status_manualCharge;
				response();
			}
			else {
				printf("Command ignored. Not in idle state\r\n");
			}

			break;
		}
		case cmd_manualDischarge: {
			if (taskStatus == status_manualCharge) {
				taskStatus = status_manualReturn;
				response();
			}
			else {
				printf("Command ignored. Not in  manual Charge state\r\n");
			}
			break;
		}
		case cmd_confirmChargedWell: {
			if (taskStatus == status_autoCharge) {
				chargeWellAck = 1;
				taskStatus = status_Charging;
				response();
			}
			else {
				printf("Command ignored. Not in  charging state\r\n");
			}
			break;
		}

		}
	}
}
 
void WISSON_DUMMY_SERVER::response(){

	std::vector<uint8_t> payload;
	payload.emplace_back(taskStatus & 0xFF);
	payload.emplace_back(taskStatus >>8 & 0xFF);
	payload.emplace_back(taskError & 0xFF);
	payload.emplace_back(taskError >>8 & 0xFF);
 
	 
	//prepare frame
	slimSerial.transmitFrame(slimSerial.assembleTxFrameWithAddress(ADDRESS,RESPONSE_FUNC,payload));

	printf("Response status=%x:%s, error=%x\r\n", taskStatus, status_string[taskStatus].c_str(), taskError);
}
 

void WISSON_DUMMY_SERVER::runTaskStateMachine() {
	switch (taskStatus) {

		case status_idle: {
			if (taskStatusLast != taskStatus) {
				printf("[status] in idle state\r\n");
			}
			taskStatusLast = taskStatus;
			break;
		}
		case status_autoCharge: {
			if (taskStatusLast != taskStatus) {
				printf("[status] Auto charging\r\n");
			}

			
			chargeWellAck = 0;
			int t = TIME_CHARGING_MAX*20;
			int tt = 20;
			while (t > 0) {
				if (chargeWellAck) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				if (tt-- == 0) {
					printf("[status] waiting for charge well confirmation\r\n");
					tt = 20;
				}
				t--;
			}
			if (t == 0) {
				printf("[status] Fail to get confirmation, going to discharging now\r\n");
				taskStatusLast = taskStatus;
				taskStatus = status_autoDischarge;
			}
			break;
		}
 
		case status_Charging: {
			if (taskStatusLast != taskStatus) {
				printf("[status] The car is successfully charging now\r\n");
				printf("[status] waiting for discharge command\r\n");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			taskStatusLast = taskStatus;
			break;
		}
		case status_autoDischarge: {
			if (taskStatusLast != taskStatus) {
				printf("[status] AutoDischarge\r\n"); 
			}
			uint32_t t = TIME_DISCHARGING;
 
			std::this_thread::sleep_for(std::chrono::seconds(t));
			taskStatusLast = taskStatus;
			taskStatus = status_homing;

			break;
		}
		case status_manualCharge: {
			if (taskStatusLast != taskStatus) {
				printf("[status] manual Charging mode now\r\n");
				printf("waiting for manual return command\r\n");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			taskStatusLast = taskStatus;
			break;
		}
		case status_manualReturn: {
			if (taskStatusLast != taskStatus) {
				printf("[status] manual return\r\n");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			taskStatusLast = taskStatus;
			taskStatus = status_homing;
			break;
		}

		case status_stopped: {
			if (taskStatusLast != taskStatus) {
				printf("[status] Stopped\r\n");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			taskStatusLast = taskStatus;
			break;
		}
		case status_homing: {
			if (taskStatusLast != taskStatus) {
				printf("[status] homing\r\n");
			}
			uint32_t t = TIME_HOMING ;
			std::this_thread::sleep_for(std::chrono::seconds(t));
			taskStatusLast = taskStatus;
			taskStatus = status_idle;
			break;
		}
		case status_error: {
			if (taskStatusLast != taskStatus) {
				printf("[status] error\r\n");
			}
			taskStatusLast = taskStatus;
			break;
		}
	}


}


int main(int argc, char** argv) {
	int ret = 0;
	cli::Parser parser(argc, argv);
	parser.set_optional<std::string>("c", "configuration", "config.ini", "Configuration File");
	parser.run_and_exit_if_error();


	std::string configurationFileName = parser.get<std::string>("c");

	mINI::INIFile file(configurationFileName);
	mINI::INIStructure ini;
	mINI::INIMap<std::string> collection;
	std::string portname;
	if (file.read(ini) == false) {
		printf("configuraion file %s not found \r\n", configurationFileName.c_str());
		ret = COM_ERROR;
	}
	else if (ini.has("SerialPort"))
	{
		collection = ini["SerialPort"];
		if (collection.has("portname"))
		{
			portname = collection["portname"];
		}
		else {
			printf("No portname found  in %s\r\n", configurationFileName.c_str());
			ret = COM_ERROR;
		}
	}
	else {
		printf("No configuration found in %s\r\n", configurationFileName.c_str());
		ret = COM_ERROR;
	}


	if (ret == COM_OK) {

		WISSON_DUMMY_SERVER wissonServer;
		if ((ret = wissonServer.setup(portname)) == COM_OK){
			while (1) {
				wissonServer.runTaskStateMachine();
				std::this_thread::sleep_for(std::chrono::microseconds(50));
			};
		}
		printf("run OK\r\n");
	}
	else {
		printf("run Error\r\n");
	}

	int32_t timeToClose = 10;
	for (int i = 0; i < timeToClose; i++) {
		printf("Closing in %d seconds...\r\n",timeToClose-i);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	 
	return 0;
}





