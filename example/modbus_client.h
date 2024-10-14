#pragma once
#include "slimRegister.h"
#include "SlimSerialRTDE.h"




#pragma pack(1)

  struct ModbusRegister_Device_Info{
    uint16_t manufactorer=0xABCD;
    uint16_t protocalVersion=0x1234;
    uint16_t hardwareVersion=0x2345;
    uint8_t softwareVersion[48]="V1.2.3.4";
    uint8_t deviceNumber[32]="V2.3.4.5";
    uint16_t comFailDetectionTime=60;
  };
  struct ModbusRegister_Realtime_Data{
    uint16_t currentState;
    uint16_t errorCode;
  };
  struct ModbusRegister_Settings{
    uint16_t gunIndex;
    uint16_t actionCommand;
    uint16_t robotPosition;
    uint16_t heightFromInletToGround;
    int16_t  distanceFromInletToRearTire;
    uint32_t carModel;
  };
  struct ModbusRegister_Errors{
    uint16_t deviceError;
  };
 


 struct ModbusRegisters{
    ModbusRegister_Device_Info device_info;
    ModbusRegister_Realtime_Data realtime_data;
    ModbusRegister_Settings settings;
    ModbusRegister_Errors errors;
 };

#pragma pack()

 

class ModbusClient : public SlimSerialRTDE {
 public:
  
  ModbusClient();
  void setAddress(uint8_t address);

private:
  void updateRegisters();
  WS_STATUS handleModbusProtocol(uint8_t *pdata,uint32_t databytes);
  void executeCommand(uint8_t *pdata,uint32_t databytes);
  
  void modbusFrameCallback(uint8_t *pdata,uint32_t databytes);
  std::function<void(uint8_t *pdata,uint32_t databytes)> modbusFrameCallbackFunc;
     
  std::array<uint8_t,1024> replyFrame;
  uint16_t replyFrameSize;
  

  ModbusRegister_Device_Info m_device_info;
  ModbusRegister_Realtime_Data m_realtime_data;
  ModbusRegister_Settings m_settings;
  ModbusRegister_Errors m_errors;


  uint16_t modbus_register_bytes[4][6]={
            {2,2,2,48,32,2},
            {2,2,0,0,0,0},
            {2,2,2,2,2,4},
            {2,0,0,0,0,0}
  };
  uint16_t modbus_register_rw[4][6]={
            {0,0,0,0,0,1},
            {0,0,0,0,0,0},
            {1,1,1,1,1,1},
            {0,0,0,0,0,0}
  };

  uint8_t *m_modbus_register_address[4][6];
  uint16_t modbus_register_max_index[4]={6,2,6,1};

  uint16_t m_targetRegister=0x00;
  uint16_t m_targetRegisterZone=0;
  uint16_t m_targetRegisterIndex=0;
  uint8_t *m_targetRegisterAddress = nullptr;
  uint16_t m_targetNumber=1;
  uint16_t m_targetBytes=0;


  uint16_t m_gunIndex;
  uint16_t m_carPose;
  uint32_t m_carModel;
};