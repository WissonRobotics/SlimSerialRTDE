#include "Maxwell_SoftArm_SerialClient.h"
#include <optional>
#include "loguru.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

enum SIGNAL_COLOR_MODE{
  COLOR_IGNORE=0,
  COLOR_ON=1,
  COLOR_OFF=2,
  COLOR_BLINK=3
};

Maxwell_SoftArm_SerialClient::Maxwell_SoftArm_SerialClient() : SlimSerialRTDE() {
  // change setting if necessary
  // setHeader(0x5A,0xA5);
 
  commandInProgress = false; 

   
  monosFrameCallbackFunc = std::bind(&Maxwell_SoftArm_SerialClient::monosFrameCallback, this, std::placeholders::_1, std::placeholders::_2);
  addRxFrameCallback(monosFrameCallbackFunc);

  //prefill query/start/stop frame buffer 
  auto qframe = assembleTxFrame(FUNC_CUSTOM_MONO_PC_QUERY);
  for(size_t i=0;i<queryFrame.size();i++){
    queryFrame[i] = qframe[i];
  }
 
}
WS_STATUS Maxwell_SoftArm_SerialClient::connect(std::string portname, uint32_t baudrate)
{
    if(SlimSerialRTDE::connect(portname,baudrate)==WS_OK){
      run();
      return WS_OK;
    }
    else{
      return WS_ERROR;
    }
    
}
bool Maxwell_SoftArm_SerialClient::isAlive(){
  return communication_is_alive;
}
uint32_t Maxwell_SoftArm_SerialClient::getNotAliveTime(){
  return m_NAKTime;
}


void Maxwell_SoftArm_SerialClient::run(){
    if (communicationThread)
    {
        communicationThread->request_stop();
        communicationThread->join();
    }
    communicationThread = std::make_unique<std::jthread>(
        [this](std::stop_token stop_token)
        {
          uint32_t printConnected=0;
          uint32_t printAlive=0;

            while (!stop_token.stop_requested())
            {
              auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_FrameTxRxPeriodMs);
              commandInProgress=true;  

              if(isConnected()){
                bool isQuery=true;
                
                if(commandFrame.commandResult==WS_NONE){
                    m_commandResult=transmitReceiveFrame(&(commandFrame.frame[0]),commandFrame.framesize,m_FrameTxRxPeriodMs);
                    commandFrame.commandResult=m_commandResult;
                    isQuery=false;
                }
                else{
                  m_commandResult=transmitReceiveFrame(&queryFrame[0],queryFrameSize,m_FrameTxRxPeriodMs);
                }

                if(m_commandResult!=WS_OK){
                  timeoutCount++;
                  m_NAKTime+=m_FrameTxRxPeriodMs;
                  if(!isQuery){
                    LOG_F(1,"Timeout to command");
                  }
                  else{
                    LOG_F(1,"Timeout to query");
                  }
                  timeoutCallback();
                }
                else{
                  m_NAKTime=0;
                }
                m_totalTxFrames_client = getTotalTxFrames();
                m_totalRxFrames_client = getTotalRxFrames();
                frameLostCount = m_totalTxFrames_client-m_totalRxFrames_client;

                if(m_NAKTime>m_NAKTimeMax){
                  m_NAKTimeMax=m_NAKTime;
                }

                if(m_NAKTime>=notAliveNAKTimeThreshold){
                  communication_is_alive = false;
                  notAliveTime = m_NAKTime;
                  notAliveCount++;
                  printAlive++;
                  if(printAlive==1){
                    LOG_F(WARNING,"Arm is not responding for %d ms",notAliveTime);
                  }
                  if(printAlive>=100){
                    printAlive=0;
                  }
                }
                else{
                  printAlive=0;
                  communication_is_alive = true;
                }
                printConnected=0;
              } 
              else{
                m_commandResult = WS_ABORT;
                commandFrame.commandResult = m_commandResult;
                printConnected++;
                if(printConnected>=100){
                  LOG_F(WARNING,"Arm is not connected.");
                  printConnected=0;
                }
              }




              commandInProgress=false;
 
 
              std::this_thread::sleep_until(timeoutPoint);
            };
        }
    );
}
Maxwell_SoftArm_SerialClient::~Maxwell_SoftArm_SerialClient(){
 

  if(communicationThread){
    communicationThread->request_stop();
    communicationThread->join();
  }
}


void Maxwell_SoftArm_SerialClient::monosFrameCallback(uint8_t *pdata,uint32_t databytes) {
  FRAME_monosdrive_to_PC* pMsg = (FRAME_monosdrive_to_PC*)pdata;
 
  switch (pMsg->funcode) {
    case FUNC_CUSTOM_MONO_PC_RESPONSE: {
      sensorData.jointState[0] = pMsg->X;
      sensorData.jointState[1] = pMsg->Y;
      sensorData.jointState[2] = pMsg->Z;
      sensorData.jointState[3] = pMsg->R;
      sensorData.jointState[4] = pMsg->Pitch;
      sensorData.jointState[5] = 0;  // no roll info

      sensorData.jointStatef[0] = sensorData.jointState[0] * 0.0001f;                                     // 0.1mm to m
      sensorData.jointStatef[1] = sensorData.jointState[1] * 0.0001f;                                     // 0.1mm to m
      sensorData.jointStatef[2] = sensorData.jointState[2] * 0.0001f;                                     // 0.1mm to m
      sensorData.jointStatef[3] = sensorData.jointState[3] / 18000.0f * M_PI - yaw_offset_ / 180 * M_PI;  // 0.01 deg to rad
      sensorData.jointStatef[4] = sensorData.jointState[4] / 18000.0f * M_PI;                             // 0.01 deg to rad
      sensorData.jointStatef[5] = 0;

      for (size_t i = 0; i < sensorData.pressure.size(); i++) {
        sensorData.pressure[i] = pMsg->pressures.pressureArray[i];
      }
      sensorData.pSource[0] = pMsg->pSource;
      sensorData.pSink[0] = pMsg->pSink;

      for(size_t i=0;i<sensorData.IOFlags.size();i++){
        sensorData.IOFlags[i] = (uint16_t)((pMsg->IOFlags.IOU16 >>i) & 0x01);
      }
 
      sensorData.laserDistance[0] = pMsg->lasers[0];
      sensorData.laserDistance[1] = pMsg->lasers[1];  

      for (size_t i = 0; i < sensorData.rotationEncoder.size(); i++) {
        sensorData.rotationEncoder[i] = pMsg->encoders[i];
      }

      for (size_t i = 0; i < sensorData.ultraSonic.size(); i++) {
        sensorData.ultraSonic[i] = pMsg->ultraSonics[i];
      }

      for (size_t i = 0; i < sensorData.errorList.size(); i++) {
        sensorData.errorList[i] = pMsg->errorList[i];
      }
 
      break;
    }

    case FUNC_CUSTOM_MONO_PC_ERROR: {
      // handling error

      break;
    }
 
  }
  if(updateCallback)
    updateCallback();
}

void Maxwell_SoftArm_SerialClient::setUpdateCallback(std::function<void()>  cb){
  updateCallback = cb;
}

void Maxwell_SoftArm_SerialClient::setYawOffset(float yaw_offset) { this->yaw_offset_ = yaw_offset; }
 
std::vector<uint8_t> Maxwell_SoftArm_SerialClient::assembleTxFrame(
SLIMDRIVE_FUNCODE_t fcode, std::vector<uint8_t> const& payload) {return assembleTxFrameWithAddress(0xFF,fcode, payload);}
  
// unit in m and rad
WS_STATUS Maxwell_SoftArm_SerialClient::commandJoint(std::array<float,6> &jointd) {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.Xchosen = 1;
  axisChosen.Ychosen = 1;
  axisChosen.Zchosen = 1;
  axisChosen.Rchosen = 1;
  axisChosen.Pitch = 1;

  AXIS_CONFIG axisConfig;
  axisConfig.axisConfigU16 = 0;
  axisConfig.motor = MOTOR_CONFIG_ABSOLUTE;

  int16_t xValue = (int16_t)(jointd[0] * 10000);                                   // m to 0.1mm
  int16_t yValue = (int16_t)(jointd[1] * 10000);                                   // m to 0.1mm
  int16_t zValue = (int16_t)(jointd[2] * 10000);                                   // m to 0.1mm
  int16_t yawValue = (int16_t)(jointd[3] / M_PI * 180 * 100 + yaw_offset_ * 100);  // rad to 0.01 deg
  int16_t pitchValue = (int16_t)(jointd[4] / M_PI * 180 * 100);                    // rad to 0.01 deg

  return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, xValue, yValue, zValue, yawValue, pitchValue, 0, 0,
      0, 0, 0, 0, 0, 0);
}


// // unit in m and rad
// WS_STATUS Maxwell_SoftArm_SerialClient::commandJointIndividual(
//     std::array<float,6> &jointd, std::vector<uint8_t> jointChosen) {
//   AXIS_CHOSEN axisChosen;
//   axisChosen.axisChosenU16 = 0;
//   axisChosen.Xchosen = jointChosen[0];
//   axisChosen.Ychosen = jointChosen[1];
//   axisChosen.Zchosen = jointChosen[2];
//   axisChosen.Rchosen = jointChosen[3];
//   axisChosen.Pitch = jointChosen[4];

//   AXIS_CONFIG axisConfig;
//   axisConfig.axisConfigU16 = 0;
//   axisConfig.motor = MOTOR_CONFIG_ABSOLUTE;

//   int16_t xValue = (int16_t)(jointd[0] * 10000);                 // m to 0.1mm
//   int16_t yValue = (int16_t)(jointd[1] * 1000 * 10);             // m to 0.1mm
//   int16_t zValue = (int16_t)(jointd[2] * 1000 * 10);             // m to 0.1mm
//   int16_t yawValue = (int16_t)(jointd[3] / M_PI * 180 * 100);    // rad to 0.01 deg
//   int16_t pitchValue = (int16_t)(jointd[4] / M_PI * 180 * 100);  // rad to 0.01 deg

//   return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, xValue, yValue, zValue, yawValue, pitchValue, 0, 0,
//       0, 0, 0, 0, 0, 0);
// }

// // unit in m and rad
// WS_STATUS Maxwell_SoftArm_SerialClient::commandJoint_delta(std::array<float,6> &jointd_delta) {
//   AXIS_CHOSEN axisChosen;
//   axisChosen.axisChosenU16 = 0;
//   axisChosen.Xchosen = 1;
//   axisChosen.Ychosen = 1;
//   axisChosen.Zchosen = 1;
//   axisChosen.Rchosen = 1;
//   axisChosen.Pitch = 1;

//   AXIS_CONFIG axisConfig;
//   axisConfig.axisConfigU16 = 0;
//   axisConfig.motor = MOTOR_CONFIG_RElATIVE;

//   int16_t xValue = (int16_t)(jointd_delta[0] * 10000);                 // m to 0.1mm
//   int16_t yValue = (int16_t)(jointd_delta[1] * 10000);                 // m to 0.1mm
//   int16_t zValue = (int16_t)(jointd_delta[2] * 10000);                 // m to 0.1mm
//   int16_t yawValue = (int16_t)(jointd_delta[3] / M_PI * 180 * 100);    // rad to 0.01 deg
//   int16_t pitchValue = (int16_t)(jointd_delta[4] / M_PI * 180 * 100);  // rad to 0.01 deg

//   return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, xValue, yValue, zValue, yawValue, pitchValue, 0, 0,
//       0, 0, 0, 0, 0, 0);
// }

// // unit in m and rad
// WS_STATUS Maxwell_SoftArm_SerialClient::commandJointIndividual_delta(
//     std::array<float,6> &jointd_delta, std::vector<uint8_t> jointChosen) {
//   AXIS_CHOSEN axisChosen;
//   axisChosen.axisChosenU16 = 0;
//   axisChosen.Xchosen = jointChosen[0];
//   axisChosen.Ychosen = jointChosen[1];
//   axisChosen.Zchosen = jointChosen[2];
//   axisChosen.Rchosen = jointChosen[3];
//   axisChosen.Pitch = jointChosen[4];

//   AXIS_CONFIG axisConfig;
//   axisConfig.axisConfigU16 = 0;
//   axisConfig.motor = MOTOR_CONFIG_RElATIVE;

//   int16_t xValue = (int16_t)(jointd_delta[0] * 1000 * 10);             // m to 0.1mm
//   int16_t yValue = (int16_t)(jointd_delta[1] * 1000 * 10);             // m to 0.1mm
//   int16_t zValue = (int16_t)(jointd_delta[2] * 1000 * 10);             // m to 0.1mm
//   int16_t yawValue = (int16_t)(jointd_delta[3] / M_PI * 180 * 100);    // rad to 0.01 deg
//   int16_t pitchValue = (int16_t)(jointd_delta[4] / M_PI * 180 * 100);  // rad to 0.01 deg

//   return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, xValue, yValue, zValue, yawValue, pitchValue, 0, 0,
//       0, 0, 0, 0, 0, 0);
// }
 
WS_STATUS Maxwell_SoftArm_SerialClient::commandVelocity(std::array<int16_t,6> &vd) {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.Xchosen = 1;
  axisChosen.Ychosen = 1;
  axisChosen.Zchosen = 1;
  axisChosen.Rchosen = 1;

  AXIS_CONFIG axisConfig;
  axisConfig.axisConfigU16 = 0;
  axisConfig.motor = MOTOR_CONFIG_VELOCITY;

  int16_t xValue = (int16_t)(vd[0]);
  int16_t yValue = (int16_t)(vd[1]);
  int16_t zValue = (int16_t)(vd[2]);
  int16_t yawValue = (int16_t)(vd[3]);

  return command(
      axisChosen.axisChosenU16, axisConfig.axisConfigU16, xValue, yValue, zValue, yawValue, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

WS_STATUS Maxwell_SoftArm_SerialClient::commandAcceleration(std::array<int16_t,6> &ad) {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.Xchosen = 1;
  axisChosen.Ychosen = 1;
  axisChosen.Zchosen = 1;
  axisChosen.Rchosen = 1;

  AXIS_CONFIG axisConfig;
  axisConfig.axisConfigU16 = 0;
  axisConfig.motor = MOTOR_CONFIG_ACCELERATION;

  int16_t xValue = (int16_t)(ad[0]);
  int16_t yValue = (int16_t)(ad[1]);
  int16_t zValue = (int16_t)(ad[2]);
  int16_t yawValue = (int16_t)(ad[3]);

  return command(
      axisChosen.axisChosenU16, axisConfig.axisConfigU16, xValue, yValue, zValue, yawValue, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}


WS_STATUS Maxwell_SoftArm_SerialClient::commandPitch(float pitch_desired) {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.Pitch = 1;

  AXIS_CONFIG axisConfig;
  axisConfig.axisConfigU16 = 0;
  axisConfig.motor = MOTOR_CONFIG_ABSOLUTE;

  int16_t pitchAngle = (int16_t)(pitch_desired / M_PI * 180 * 100);  // rad to 0.01 deg

  IOCMD iocmd;
  iocmd.IOU16 = 0;
  // iocmd.phAxis = 1;
  iocmd.ph = 0;  // unhold

  return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, pitchAngle, 0, 0, 0, 0, 0, 0, iocmd.IOU16, 0);
}


WS_STATUS Maxwell_SoftArm_SerialClient::commandPitchUp() {
    return commandPitch(0);
}

WS_STATUS Maxwell_SoftArm_SerialClient::commandPitchDown() {
    return commandPitch(90);
}

WS_STATUS Maxwell_SoftArm_SerialClient::commandPitchHold(bool pitchhold) {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.ph = 1;
  IOCMD iocmd;
  iocmd.IOU16 = 0;
  // iocmd.phAxis = 1;
  iocmd.ph = pitchhold?1:0;

  return command(axisChosen.axisChosenU16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, iocmd.IOU16, 0);
}
 
WS_STATUS Maxwell_SoftArm_SerialClient::commandElongateHold() {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.elongate = 1;

  AXIS_CONFIG axisConfig;
  axisConfig.chamber = 0x00;  // chamber command type -> status command

  int16_t elongateStatus = 0;

  return command(
      axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, elongateStatus, 0, 0, 0, 0, 0, 0, 0);
}
 


WS_STATUS Maxwell_SoftArm_SerialClient::commandGunElongate(int16_t targetPressure) {
   return commandChamber(IndexPressureElongate,targetPressure);
}
WS_STATUS Maxwell_SoftArm_SerialClient::commandGuide(int16_t targetPressure) {
   return commandChamber(IndexPressureGuide,targetPressure);
}
 
WS_STATUS Maxwell_SoftArm_SerialClient::commandGripperShift(int16_t targetPressure) {
  return commandChamber(IndexPressureShift,targetPressure);
}

WS_STATUS Maxwell_SoftArm_SerialClient::commandGrasp(int16_t targetPressure) {
  return commandChamber(IndexPressureGrasp,targetPressure);
}

WS_STATUS Maxwell_SoftArm_SerialClient::commandGunLock(int16_t targetPressure) {
  return commandChamber(IndexPressureLock,targetPressure);
}

WS_STATUS Maxwell_SoftArm_SerialClient::commandChamber(int pressureIndex,int16_t targetPressure) {
  AXIS_CONFIG axisConfig;
  axisConfig.chamber = 0x00;

  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  switch (pressureIndex)
  {
  case IndexPressureElongate:
    axisChosen.elongate = 1;
    return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, targetPressure, 0, 0, 0, 0, 0, 0, 0);
    break;
    case IndexPressureCable:
    axisChosen.cable = 1;
    return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, 0, targetPressure, 0, 0, 0, 0, 0, 0);
    break;
      case IndexPressureGuide:
    axisChosen.guide = 1;
    return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, 0, 0,targetPressure, 0, 0, 0, 0, 0);
    break;
      case IndexPressureShift:
    axisChosen.shift = 1;
    return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, 0, 0,0, targetPressure, 0, 0, 0, 0);
    break;
      case IndexPressureGrasp:
    axisChosen.grasp = 1;
    return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, 0, 0,0, 0, targetPressure, 0, 0, 0);
    break;
      case IndexPressureLock:
    axisChosen.lock = 1;
     return command(axisChosen.axisChosenU16, axisConfig.axisConfigU16, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, targetPressure, 0, 0);
    break;

  default:
    break;
  }  
  return WS_ERROR;
}



WS_STATUS Maxwell_SoftArm_SerialClient::commandSignalLight(LightSignalCommand signal_command)
{
  m_signal_light_command = signal_command;
  switch (m_signal_light_command) {
    case LightRedOn:{
      return  commandSignalLightRGB(COLOR_ON, COLOR_OFF, COLOR_OFF);
    }
    case LightRedBlink: {

      return commandSignalLightRGB(COLOR_BLINK, COLOR_OFF, COLOR_OFF);
    }
    case LightYellowOn: {

      return commandSignalLightRGB(COLOR_ON, COLOR_ON, COLOR_OFF);
    }
    case LightYellowBlink: {

      return commandSignalLightRGB(COLOR_BLINK, COLOR_BLINK, COLOR_OFF);
    }
    case LightBlueOn: {

      return commandSignalLightRGB(COLOR_OFF, COLOR_OFF, COLOR_ON);
    }
    case LightBlueBlink: {

      return commandSignalLightRGB(COLOR_OFF, COLOR_OFF, COLOR_BLINK);
    }
    case LightGreenOn: {

      return commandSignalLightRGB(COLOR_OFF, COLOR_ON, COLOR_OFF);
    }
    case LightGreenBlink: {

      return commandSignalLightRGB(COLOR_OFF, COLOR_BLINK, COLOR_OFF);
    }
    case LightOff: {

      return commandSignalLightRGB(COLOR_OFF, COLOR_OFF, COLOR_OFF);
    }
    case LightWhiteOn: {

      return commandSignalLightRGB(COLOR_ON, COLOR_ON, COLOR_ON);
    }
    case LightWhiteBlink: {

      return commandSignalLightRGB(COLOR_BLINK, COLOR_BLINK, COLOR_BLINK);
    }
  }
  return WS_STATUS();
}
WS_STATUS Maxwell_SoftArm_SerialClient::commandMagnet(int16_t command_value)
{
    AXIS_CHOSEN axisChosen;
    axisChosen.axisChosenU16 = 0;
    axisChosen.magnet = 1;

    IOCMD iocmd;
    iocmd.IOU16 = 0;
    iocmd.magnet = command_value;

    return command(axisChosen.axisChosenU16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, iocmd.IOU16, 0);
}
 
WS_STATUS Maxwell_SoftArm_SerialClient::commandSignalLightRGB(
    uint8_t red_status, uint8_t green_status, uint8_t yellow_status) {
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.rgy = 1;

  IOCMD iocmd;
  iocmd.IOU16 = 0;
  iocmd.red = red_status;
  iocmd.greed = green_status;
  iocmd.yellow = yellow_status;
  //  iocmd.doorconfig = int8_t(2);
  return command(axisChosen.axisChosenU16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, iocmd.IOU16, 0);
}

std::string Maxwell_SoftArm_SerialClient::getSignalLightName(LightSignalCommand signal_command)
{
  return lightSignalCommandName[signal_command];
}
std::string Maxwell_SoftArm_SerialClient::getSignalLightName()
{
  return lightSignalCommandName[m_signal_light_command];
}
WS_STATUS Maxwell_SoftArm_SerialClient::commandLight(std::string light_name, uint8_t light_cmd) {
   
  AXIS_CHOSEN axisChosen;
  axisChosen.axisChosenU16 = 0;
  axisChosen.led = 1;

  IOCMD iocmd;
  iocmd.IOU16 = 0;
  if(light_name=="Global" || light_name=="global"){
    m_flashLightGlobalCmd = light_cmd;
  }
  else if(light_name=="Local" || light_name=="local"){
    m_flashLightLocalCmd = light_cmd;
  }

  iocmd.globalLED = m_flashLightGlobalCmd;
  iocmd.localLED = m_flashLightLocalCmd;
  //  iocmd.doorconfig = int8_t(2); 
  return command(axisChosen.axisChosenU16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, iocmd.IOU16, 0);
}
 
 

WS_STATUS Maxwell_SoftArm_SerialClient::command(uint16_t axisChosen, uint16_t axisConfig, int16_t xValue, int16_t yValue,
    int16_t zValue, int16_t yawValue, int16_t pitchValue, int16_t elongateValue, int16_t cable, int16_t guide,
    int16_t shift, int16_t grasp, int16_t lock, uint16_t ioValue, int16_t door) {
 
 	std::unique_lock<std::mutex> lock_(commandMtx);
  
  commandFrame.index++;
  commandFrame.framesize = commandFrameBufSize;
  
  uint8_t *payload = &(commandFrame.frame[0]);
  *payload++=  getHeader(0);
  *payload++=  getHeader(1);
  *payload++=  getTxAddress();
  *payload++=  commandFrameBufSize-7;
  *payload++=  (uint8_t)FUNC_CUSTOM_MONO_PC_ACTION;

  *payload++=((uint8_t)(axisChosen & 0xFF));
  *payload++=((uint8_t)(axisChosen >> 8 & 0xFF));

  *payload++=((uint8_t)(axisConfig & 0xFF));
  *payload++=((uint8_t)(axisConfig >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)xValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)xValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)yValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)yValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)zValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)zValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)yawValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)yawValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)pitchValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)pitchValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)elongateValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)elongateValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)cable) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)cable) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)guide) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)guide) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)shift) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)shift) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)grasp) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)grasp) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)lock) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)lock) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)ioValue) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)ioValue) >> 8 & 0xFF));

  *payload++=((uint8_t)(((uint16_t)door) & 0xFF));
  *payload++=((uint8_t)(((uint16_t)door) >> 8 & 0xFF));
 
	addCRC(&(commandFrame.frame[0]),commandFrameBufSize-2);

  //set commandResult to WS_NONE at last, then the sending thread would start to send.
  commandFrame.commandResult = WS_NONE;

 	return spinWait([&](){ return (commandFrame.commandResult!=WS_NONE); },m_CommandTimeoutPeriodMs);
} 
 

 WS_STATUS Maxwell_SoftArm_SerialClient::commandStart() {
 
 	std::unique_lock<std::mutex> lock_(commandMtx);
  
  commandFrame.index++;
  commandFrame.framesize = 7;
  uint8_t *payload = &(commandFrame.frame[0]);
  *payload++=  getHeader(0);
  *payload++=  getHeader(1);
  *payload++=  getTxAddress();
  *payload++=  commandFrame.framesize-7;
  *payload++=  (uint8_t)FUNC_START_VALVEPUMP;

  //set commandResult to WS_NONE at last, then the sending thread would start to send.
  commandFrame.commandResult = WS_NONE;
 
	addCRC(&(commandFrame.frame[0]),commandFrame.framesize-2);

 	return spinWait([&](){ return (commandFrame.commandResult!=WS_NONE); },m_CommandTimeoutPeriodMs);
}
 WS_STATUS Maxwell_SoftArm_SerialClient::commandStop() {
 
 	std::unique_lock<std::mutex> lock_(commandMtx);
  
  commandFrame.index++;
  commandFrame.framesize = 7;
  uint8_t *payload = &(commandFrame.frame[0]);
  *payload++=  getHeader(0);
  *payload++=  getHeader(1);
  *payload++=  getTxAddress();
  *payload++=  commandFrame.framesize-7;
  *payload++=  (uint8_t)FUNC_STOP_VALVEPUMP;

    //set commandResult to WS_NONE at last, then the sending thread would start to send.
  commandFrame.commandResult = WS_NONE;
 
	addCRC(&(commandFrame.frame[0]),commandFrame.framesize-2);

 	return spinWait([&](){ return (commandFrame.commandResult!=WS_NONE); },m_CommandTimeoutPeriodMs);
}


// WS_STATUS Maxwell_SoftArm_SerialClient::commandQuery(uint32_t timeout) {

//   auto const timeoutPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_FrameTxRxPeriodMs);
// 	std::unique_lock<std::mutex> lock_(commandMtx);

  
 
//   if (commandCV.wait_until(lock_, timeoutPoint, [&]() {return !commandInProgress;})) {
// 		 return commandResult;
// 	}
// 	else {//timeout occurred
	   
// 		 return WS_TIMEOUT;
// 	}
    
// }


 


 