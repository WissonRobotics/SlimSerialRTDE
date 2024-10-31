#include "modbus_client.h"
#include <stdlib.h>
#include <iomanip>
#include <cstring> 
#include <thread>
#include "loguru.hpp"

enum class CarPose : uint16_t { Left = 0x00, Right = 0x01 }; 
  enum class ActionEnum : uint16_t {
    DoNothing = 0x00,
    StartCharging = 0x01,
    StopCharging= 0x02, 
  };
  enum class TaskStateReply : uint16_t {
    Idle = 0x00,
    AutoChargingAttaching = 0x01,
    AutoChargingAttachingFinished = 0x02,
    AutoChargingDetaching = 0x03,
    AutoChargingDetachingFinished = 0x04,
    ChargingError = 0x05
  };
  enum class CarModelEnum : uint32_t {
    WenjieM9 = 0x0000,
    WenjieM7 = 0x0001,
    ZhijieS7 = 0x0002,
    AWEITA11 = 0x0003,
    AWEITA12 = 0x0004,
    UNKNOWN =  0xFFFF
  };


ModbusClient::ModbusClient():SlimSerialRTDE() {
  
  //set modbus frame type
  setFrameType(SLIMSERIAL_FRAME_TYPE_3_MODBUS_NUM);
 
  //set tx address
  setAddress(0x60); //     default tx set self address
 
  //add frame callback
  modbusFrameCallbackFunc =
      std::bind(&ModbusClient::modbusFrameCallback, this, std::placeholders::_1, std::placeholders::_2);
  addRxFrameCallback(modbusFrameCallbackFunc);

  //init modbus register address map
  m_modbus_register_address[0][0] = (uint8_t*)&m_device_info;
  m_modbus_register_address[1][0] = (uint8_t*)&m_realtime_data;
  m_modbus_register_address[2][0] = (uint8_t*)&m_settings;
  m_modbus_register_address[3][0] = (uint8_t*)&m_errors;
  for(int i=0;i<4;i++){
    if(modbus_register_max_index[i]>1){
      for(int j=1;j<modbus_register_max_index[i];j++){
        m_modbus_register_address[i][j] = m_modbus_register_address[i][j-1]+modbus_register_bytes[i][j-1];
      }
    }
  }
}

void ModbusClient::setAddress(uint8_t address){
  std::vector<uint8_t> legalAddresses={0x00,address};

  //add rx address filter
  setAddressFilter(legalAddresses);
  toggleAddressFilter(true);

  //set tx address
  setTxAddress(address);
}

void ModbusClient::modbusFrameCallback(uint8_t *pdata,uint32_t databytes) {
  SLIMSERIAL_FRAME_TYPE_3_MODBUS *pMsg = (SLIMSERIAL_FRAME_TYPE_3_MODBUS *)pdata;
 
  //update register contents from machine states
  updateRegisters(); 

  //handle Modbus command: read/write registers and reply
  WS_STATUS status = handleModbusProtocol(pdata,databytes);

  //execute command if needed
  if(status == WS_OK){
    executeCommand(pdata,databytes);
  }

  printf("Modbus handle =  %s\n",(status==WS_OK)?"OK":"Error");
  printf("m_device_info= %x,%x,%x, %x\r\n",
        m_device_info.manufactorer,
        m_device_info.protocalVersion,
        m_device_info.hardwareVersion,
        m_device_info.comFailDetectionTime
  );
    printf("realtime_data= %x %x\r\n",m_realtime_data.currentState,m_realtime_data.errorCode);
    printf("setting:%x %x %x %x %x %x \n",
      m_settings.gunIndex,
      m_settings.actionCommand,
      m_settings.robotPosition,
      m_settings.heightFromInletToGround,
      m_settings.distanceFromInletToRearTire,
      m_settings.carModel
    );
    printf("errors:%x\n",m_errors.deviceError);
}


void ModbusClient::updateRegisters(){
  //TODO: update registers
    // realtime_data_.currentState = ;
    // realtime_data_.errorCode = ;
    //errors_.deviceError = ;
}
 

WS_STATUS ModbusClient::handleModbusProtocol(uint8_t *pdata,uint32_t databytes){
  SLIMSERIAL_FRAME_TYPE_3_MODBUS *pMsg = (SLIMSERIAL_FRAME_TYPE_3_MODBUS *)pdata;
 
  m_targetRegister = (uint16_t)((uint16_t)(pMsg->payload.data8[0])<<8) | (pMsg->payload.data8[1]);
   
  m_targetRegisterZone =  m_targetRegister>>12;
  m_targetRegisterIndex = m_targetRegister&0x0F;
  
  if(pMsg->funcode == 0x03 || pMsg->funcode == 0x10){
     m_targetNumber = (uint16_t)((uint16_t)(pMsg->payload.data8[2])<<8) | (pMsg->payload.data8[3]);
  }
  else{
    m_targetNumber = 1;
  }
 
  //valid m_targetRegisterZone
  if(m_targetRegisterZone>4){
    //error
    //TODO:
    printf("Wrong m_targetRegisterZone\n");
    return WS_ERROR;
  }
  //valid m_targetRegisterIndex
  if(m_targetRegisterIndex==0x1b){
      m_targetRegisterIndex = 4;
  }
  else if(m_targetRegisterIndex==0x2b){
      m_targetRegisterIndex = 5;
  }
  if(m_targetRegisterIndex+m_targetNumber>modbus_register_max_index[m_targetRegisterZone]){
    //error
    //TODO:
    printf("Wrong request Number\n");
    return WS_ERROR;
  }
  m_targetRegisterAddress = m_modbus_register_address[m_targetRegisterZone][m_targetRegisterIndex];
  uint8_t* regAddr =  m_targetRegisterAddress;
 

 
  //handle 0x03 read
  m_targetBytes=0;
  if (pMsg->funcode == 0x03) {

      for(int i=m_targetRegisterIndex;i<m_targetRegisterIndex+m_targetNumber;i++){
        m_targetBytes +=modbus_register_bytes[m_targetRegisterZone][i];
      }
 
      //reply if not broadcasting
      if(pMsg->address==0x00){
        return WS_OK;
      }
      replyFrame[0]=pMsg->address;
      replyFrame[1]=pMsg->funcode;
      replyFrame[2] = m_targetBytes;
      uint8_t *pbuf = &replyFrame[3];
      for(int i=m_targetRegisterIndex;i<m_targetRegisterIndex+m_targetNumber;i++){
        if(modbus_register_bytes[m_targetRegisterZone][i]==2){
            *pbuf++=regAddr[1];
            *pbuf++=regAddr[0];
            regAddr+=2;
        }
        else if(modbus_register_bytes[m_targetRegisterZone][i]==4){
            *pbuf++=regAddr[3];
            *pbuf++=regAddr[2];
            *pbuf++=regAddr[1];
            *pbuf++=regAddr[0];
            regAddr+=4;
        }
        else{
            for(int j=0;j<modbus_register_bytes[m_targetRegisterZone][i];j++){
              *pbuf++=regAddr[j];
            }
            regAddr+=modbus_register_bytes[m_targetRegisterZone][i];
        }
      }
      uint16_t crc = calculateCRC((uint8_t*)(&replyFrame[0]),pbuf-&replyFrame[0]);
      *pbuf++ = (uint8_t)(crc & 0xFF);
      *pbuf++ = (uint8_t)((crc >> 8) & 0xFF);
      replyFrameSize = pbuf-&replyFrame[0];
      transmitFrame(&replyFrame[0],replyFrameSize);
      return WS_OK;
  }
  else if(pMsg->funcode == 0x06){

    //write
    if(modbus_register_rw[m_targetRegisterZone][m_targetRegisterIndex]==0){
      //error
      printf("not writable\n");
      return WS_ERROR;;
    }

    if(databytes-6!= modbus_register_bytes[m_targetRegisterZone][m_targetRegisterIndex]){
      //error
      printf("wrong write bytes\n");
      return WS_ERROR;
    }

    if(modbus_register_bytes[m_targetRegisterZone][m_targetRegisterIndex]==2){
      regAddr[0] = pMsg->payload.data8[3]; // low byte
      regAddr[1] = pMsg->payload.data8[2]; // high byte
    }
    else if(modbus_register_bytes[m_targetRegisterZone][m_targetRegisterIndex]==4){
      regAddr[0] = pMsg->payload.data8[5]; // low byte
      regAddr[1] = pMsg->payload.data8[4]; // high byte
      regAddr[2] = pMsg->payload.data8[3]; // low byte
      regAddr[3] = pMsg->payload.data8[2]; // high byte
    }
 
    //reply if not broadcasting
    if(pMsg->address==0x00){
      return WS_OK;
    }
    memcpy(&replyFrame[0],pdata,databytes);
    replyFrameSize = databytes;
    transmitFrame(&replyFrame[0],replyFrameSize);
 
    return WS_OK;
  }
  else if(pMsg->funcode == 0x10){
    if(modbus_register_rw[m_targetRegisterZone][m_targetRegisterIndex]==0){
      //error
      printf("not writable\n");
      return WS_ERROR;
    }
  
    for(int i=m_targetRegisterIndex;i<m_targetRegisterIndex+m_targetNumber;i++){
      m_targetBytes +=modbus_register_bytes[m_targetRegisterZone][i];
    }

    if(databytes-9!= m_targetBytes || m_targetBytes!=pdata[6]){
      //error
      printf("wrong write bytes\n");
      return WS_ERROR;
    }

    uint8_t *pbuf =  pdata+7;
    for(int i=m_targetRegisterIndex;i<m_targetRegisterIndex+m_targetNumber;i++){
      if(modbus_register_bytes[m_targetRegisterZone][i]==2){
          regAddr[1] = *pbuf++;
          regAddr[0] = *pbuf++;
          regAddr+=2;
      }
      else if(modbus_register_bytes[m_targetRegisterZone][i]==4){
          regAddr[3] = *pbuf++;
          regAddr[2] = *pbuf++;
          regAddr[1] = *pbuf++;
          regAddr[0] = *pbuf++;
          regAddr+=4;
      }

    }

    if(pMsg->address==0x00){
      return WS_OK;
    }

    memcpy(&replyFrame[0],pdata,6);
    uint16_t crc = calculateCRC((uint8_t*)(&replyFrame[0]),6);
    replyFrame[6] = (uint8_t)(crc & 0xFF);
    replyFrame[7] = (uint8_t)((crc >> 8) & 0xFF);
    replyFrameSize = 8;
    transmitFrame(&replyFrame[0],replyFrameSize);
    return WS_OK;
  }
}

void ModbusClient::executeCommand(uint8_t *pdata,uint32_t databytes){
  SLIMSERIAL_FRAME_TYPE_3_MODBUS *pMsg = (SLIMSERIAL_FRAME_TYPE_3_MODBUS *)pdata;

  //only need to execute command if 0x2001 is written
  if((m_targetRegisterZone==2) && ((m_targetRegisterIndex+m_targetNumber)>=1) && (m_targetRegisterIndex<=1)){
    if(m_settings.actionCommand == (uint16_t)(ActionEnum::StartCharging)){
      //TODO: execute start charging
      m_gunIndex = m_settings.gunIndex;
      m_carPose = m_settings.robotPosition;
      m_carModel = m_settings.carModel;
 

    }
    else if(m_settings.actionCommand == (uint16_t)(ActionEnum::StopCharging)){
      //TODO: execute stop charging

    }
  }
}


 
int main(int argc, const char *argv[]){
 
    ModbusClient modbusClient;
    loguru::g_stderr_verbosity = 2;
    modbusClient.connect("/dev/ttyUSB0",115200);
    while(true){
       std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

 
    return 0;
}