#pragma once
#include "SlimSerialRTDE.h"
 
#include "slimRegister.h" 
#include <string.h>
#include "loguru.hpp"
#include <thread>


class SlimSerialServer :public SlimSerialRTDE {
public:
	std::unique_ptr<std::jthread> internalDynamicsThread;
	SlimSerialServer(std::string logFileName = "SlimSerialServer") {
		addRxFrameCallback(std::bind(&SlimSerialServer::frameCallback, this, std::placeholders::_1,std::placeholders::_2));

		//addAddressFilter(0x12);
		toggleAddressFilter(false);

		addFuncodeFilter(0x05);
		toggleFuncodeFilter(false);

		setFrameLengthFilter(200);

		resetFrameCount();

		setServer(true);

        std::vector<uint8_t>payload;
		for (int i = 0; i < 60; i++) {
			payload.emplace_back(i);
		}
		m_txframe = assembleTxFrameWithAddress(0x00, FUNC_CUSTOM_MONO_PC_RESPONSE, payload);

        memset((uint8_t*)&dataFrame,0,sizeof(dataFrame));
		memset((uint8_t*)&dataInternal,0,sizeof(dataInternal));
        memset((uint8_t*)&dataCmd,0,sizeof(dataCmd));

		preparePCDataFrame();

		internalDynamicsThread = std::make_unique<std::jthread>(
			[this](std::stop_token stop_token)
			{ 
				while (!stop_token.stop_requested())
				{	
					if(m_enableInternalDynamics){
						dataInternal.X += (dataCmd.XCmd-dataInternal.X)>=5?5:((dataCmd.XCmd-dataInternal.X)<=-5?-5:0);
						dataInternal.Y += (dataCmd.YCmd-dataInternal.Y)>=5?5:((dataCmd.YCmd-dataInternal.Y)<=-5?-5:0);
						dataInternal.Z += (dataCmd.ZCmd-dataInternal.Z)>=5?5:((dataCmd.ZCmd-dataInternal.Z)<=-5?-5:0);
						dataInternal.R += (dataCmd.RCmd-dataInternal.R)>=5?5:((dataCmd.RCmd-dataInternal.R)<=-5?-5:0);
						dataInternal.Pitch += (dataCmd.PitchCmd-dataInternal.Pitch)>=20?20:((dataCmd.PitchCmd-dataInternal.Pitch)<=-20?-20:0);
						
						dataInternal.pressures.Ppitch_inner += (dataCmd.PitchCmd-dataInternal.Pitch)>=10?-10:((dataCmd.PitchCmd-dataInternal.Pitch)<=-10?10:0);
						dataInternal.pressures.Ppitch_outer += (dataCmd.PitchCmd-dataInternal.Pitch)>=10?10:((dataCmd.PitchCmd-dataInternal.Pitch)<=-10?-10:0);
						dataInternal.pressures.Pelongate += (dataCmd.elongateCmd-dataInternal.pressures.Pelongate)>=10?10:((dataCmd.elongateCmd-dataInternal.pressures.Pelongate)<=-10?-10:0);
						dataInternal.pressures.Pcable+= (dataCmd.cableCmd-dataInternal.pressures.Pcable)>=10?10:((dataCmd.cableCmd-dataInternal.pressures.Pcable)<=-10?-10:0);
						dataInternal.pressures.Pguide+= (dataCmd.guideCmd-dataInternal.pressures.Pguide)>=10?10:((dataCmd.guideCmd-dataInternal.pressures.Pguide)<=-10?-10:0);
						dataInternal.pressures.Pshift+= (dataCmd.shiftCmd-dataInternal.pressures.Pshift)>=10?10:((dataCmd.shiftCmd-dataInternal.pressures.Pshift)<=-10?-10:0);
						dataInternal.pressures.Pgrasp+= (dataCmd.graspCmd-dataInternal.pressures.Pgrasp)>=10?10:((dataCmd.graspCmd-dataInternal.pressures.Pgrasp)<=-10?-10:0);
						dataInternal.pressures.Plock+= (dataCmd.lockCmd-dataInternal.pressures.Plock)>=10?10:((dataCmd.lockCmd-dataInternal.pressures.Plock)<=-10?-10:0);

 
						dataInternal.lasers[0] += (dataCmd.elongateCmd-dataInternal.pressures.Pelongate)>=10?5:((dataCmd.elongateCmd-dataInternal.pressures.Pelongate)<=-10?-5:0);
						dataInternal.lasers[0] = dataInternal.lasers[0]>220?220:dataInternal.lasers[0]<68?68:dataInternal.lasers[0];

						dataInternal.lasers[1] += (dataCmd.guideCmd-dataInternal.pressures.Pguide)>=10?5:((dataCmd.guideCmd-dataInternal.pressures.Pguide)<=-10?-5:0);
						dataInternal.lasers[1] = dataInternal.lasers[1]>156?156:dataInternal.lasers[1]<82?82:dataInternal.lasers[1];

						if(dataInternal.pressures.Plock > 700){
							dataInternal.IOFlags.attach_status_0 = 1;
						}
						else if(dataInternal.pressures.Plock < -500){
							dataInternal.IOFlags.attach_status_1 = 0;
						}
	
						if(dataInternal.pressures.Pshift > 700){
							dataInternal.IOFlags.shift_away_status = 0;
							dataInternal.IOFlags.shift_central_status = 1;
						}
						else if(dataInternal.pressures.Pshift < -500){
							dataInternal.IOFlags.shift_away_status = 1;
							dataInternal.IOFlags.shift_central_status = 0;
						}

						 
					}
 
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				};
			}
		);
	}

	~SlimSerialServer(){
		if(internalDynamicsThread){
			internalDynamicsThread->request_stop();
			internalDynamicsThread->join();
		}
	}

	void frameCallback(uint8_t *pdata,uint32_t databytes){
 		FRAME_PC_To_Monosdrive *pMsg=(FRAME_PC_To_Monosdrive *)pdata;
		LOG_F(1,"[Server] Handling Rx frame with funcode %x",pdata[4]);
		totalRxFrame += 1;
		switch (pdata[4]) {
			//     case 0x05: {
			//         transmitFrameGood();
			//         //transmitFrameRandom();
			//     }
			// }
			
			case FUNC_START_VALVEPUMP:{
				m_enableInternalDynamics = true;
				transmitFrame(assembleTxFrameWithAddress(0x00,FUNC_START_VALVEPUMP));
				totalTxFrameGood++;
				break;
			}
			case FUNC_STOP_VALVEPUMP:{
				m_enableInternalDynamics = false;
				transmitFrame(assembleTxFrameWithAddress(0x00,FUNC_STOP_VALVEPUMP));
				totalTxFrameGood++;
				break;
			}

			case FUNC_CUSTOM_MONO_PC_ACTION:{
				preparePCDataFrame();
				transmitFrame((uint8_t*)&dataFrame,sizeof(dataFrame));
				totalTxFrameGood++;
 
				AXIS_CHOSEN axischosen = pMsg->axisChosen;
				MOTOR_CONFIG motor_config = (MOTOR_CONFIG)(pMsg->axisConfig.motor);

				if(axischosen.Xchosen){
					dataCmd.XCmd = pMsg->XCmd;
				}
				if(axischosen.Ychosen){
					dataCmd.YCmd = pMsg->YCmd;
				}
				if(axischosen.Zchosen){
					dataCmd.ZCmd = pMsg->ZCmd;
				}
				if(axischosen.Rchosen){
					dataCmd.RCmd = pMsg->RCmd;
				}
				if(axischosen.Pitch){
					dataCmd.PitchCmd = pMsg->PitchCmd;
				}
				if(axischosen.elongate){
					dataCmd.elongateCmd =pMsg->elongateCmd;
				}
				if(axischosen.cable){
					dataCmd.cableCmd = pMsg->cableCmd;
				}
				if(axischosen.guide){
					dataCmd.guideCmd = pMsg->guideCmd;
				}
				if(axischosen.shift){
					dataCmd.shiftCmd = pMsg->shiftCmd;
				}
				if(axischosen.grasp){
					dataCmd.graspCmd = pMsg->graspCmd;
				}
				if(axischosen.lock){
					dataCmd.lockCmd = pMsg->lockCmd;
				}
				if(axischosen.led ){
					dataCmd.IOCmd.globalLED = pMsg->IOCmd.globalLED;
					dataCmd.IOCmd.localLED = pMsg->IOCmd.localLED;
				}
				if(axischosen.rgy){
					dataCmd.IOCmd.greed = pMsg->IOCmd.greed;
					dataCmd.IOCmd.red = pMsg->IOCmd.red;
					dataCmd.IOCmd.yellow = pMsg->IOCmd.yellow;
				}
				if(axischosen.ph){
					dataCmd.IOCmd = pMsg->IOCmd;
				}



				break;
			}
 
			case FUNC_CUSTOM_MONO_PC_QUERY:{
				
				preparePCDataFrame();
				transmitFrame((uint8_t*)&dataFrame,sizeof(dataFrame));
				totalTxFrameGood++;
				
				break;
			}

	
		}
		
		
	}
 



    void preparePCDataFrame(){
		memcpy((uint8_t *)(&dataFrame),(uint8_t *)(&dataInternal),sizeof(dataInternal));
        dataFrame.header[0]=0x5A;
        dataFrame.header[1]=0xA5;
        dataFrame.src = 0x00;
        dataFrame.payloadBytes = sizeof(dataFrame)-7;
        dataFrame.funcode = FUNC_CUSTOM_MONO_PC_RESPONSE;
 


        addCRC((uint8_t*)&dataFrame,sizeof(dataFrame)-2);
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

 
	uint32_t totalRxFrame=0;
	uint32_t totalTxFrameGood=0;
	uint32_t totalTxFrameBad=0;
	std::vector<uint8_t>  m_txframe;
    static constexpr uint16_t dataFrameSize=37;
    FRAME_monosdrive_to_PC dataFrame;
	FRAME_monosdrive_to_PC dataInternal;
	FRAME_PC_To_Monosdrive dataCmd;
	float pitchangle;
	bool m_enableInternalDynamics = false;
};

  