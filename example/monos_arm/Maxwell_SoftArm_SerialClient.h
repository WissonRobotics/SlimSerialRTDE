#pragma once
 
#include "slimRegister.h" 

#include "../../include/SlimSerialRTDE.h"
#include <mutex>
#include <thread>
#include "loguru.hpp"
#include <source_location>
#include <cmath>



enum LightSignalCommand {
  LightBlueOn = 0,
  LightBlueBlink,
  LightYellowOn,
  LightYellowBlink,
  LightGreenOn,
  LightGreenBlink,
  LightRedOn,
  LightRedBlink,
  LightOff,
  LightWhiteOn,
  LightWhiteBlink
};





template <typename T, size_t N>
std::array<T, N> operator+(const std::array<T, N> &ob1, const std::array<T, N> &ob2)
{
    std::array<T, N> res;
    for (size_t i = 0; i < N; ++i)
        res[i] = ob1[i] + ob2[i];
    return res;
}

template <typename T, size_t N>
std::array<T, N> operator-(const std::array<T, N> &ob1, const std::array<T, N> &ob2)
{
    std::array<T, N> res;
    for (size_t i = 0; i < N; ++i)
        res[i] = ob1[i] - ob2[i];
    return res;
}

template <typename T, size_t N>
T vectorDistance(const std::array<T, N> &ob1, const std::array<T, N> &ob2)
{
    std::array<T, N> err = ob1 - ob2;
    T res = 0;
    for (size_t i = 0; i < N; ++i)
        res += (err[i] * err[i]);
    res = std::sqrt(res);
    return res;
}

struct MaxwellSoftArmSensorData
{
    std::array<float, 6> jointStatef;
    std::array<int16_t, 6> jointState;
    std::array<int16_t, 8> pressure;
    std::array<int16_t, 1> pSource;
    std::array<int16_t, 1> pSink;
    std::array<uint8_t, 8> IOFlags;// = pinStatus + limitStatus + gunStatus
    std::array<int16_t, 2> laserDistance;
    std::array<int16_t, 4> rotationEncoder;
    std::array<int16_t, 3> ultraSonic;
    std::array<uint16_t, 8> errorList;
};

struct MaxwellSoftArmCommandData
{

    std::array<int16_t, 6> jointCmd;
    PressureUnion pressureCmd;
    std::array<int16_t, 2> pSourceCmd;
    std::array<int16_t, 2> pSinkCmd;
    std::array<int16_t, 2> laserDistanceCmd;
    
};

inline constexpr uint16_t defaultFrameTxRxPeriodMs = 10; 

inline constexpr uint16_t defaultCommandTimeoutPeriodMs = 3*defaultFrameTxRxPeriodMs; 
 

class Maxwell_SoftArm_SerialClient : public SlimSerialRTDE
{
public:
    Maxwell_SoftArm_SerialClient();
    ~Maxwell_SoftArm_SerialClient();
    WS_STATUS connect(std::string portname,uint32_t baudrate);
    bool isAlive();
    uint32_t getNotAliveTime();
    int m_totalTxFrames_client;
    int m_totalRxFrames_client;
    void run();

    std::function<void(uint8_t *pdata, uint32_t databytes)> monosFrameCallbackFunc;
    void monosFrameCallback(uint8_t *pdata, uint32_t databytes);
    void setYawOffset(float yaw_offset);

    std::function<void()> updateCallback;
    void setUpdateCallback(std::function<void()> cb);

    std::vector<uint8_t> assembleTxFrame(SLIMSERIAL_FUNCODE_t fcode, std::vector<uint8_t> const &payload = {});

    // std::array<uint8_t, 8> &getIOStatus() { return sensorData.IOFlags; }; 
    // std::array<int16_t, 2> &getLaserDistance() { return sensorData.laserDistance; };
    // std::array<int16_t, 8> &getPressure() { return sensorData.pressure; };
    // std::array<int16_t, 1> &getPSource() { return sensorData.pSource; };
    // std::array<int16_t, 1> &getPSink() { return sensorData.pSink; };
    // std::array<float, 6> &getJointf() { return sensorData.jointStatef; };
    // std::array<int16_t, 4> &getRotationEncoder() { return sensorData.rotationEncoder; };
    // std::array<int16_t, 3> &getUltraSonic() { return sensorData.ultraSonic; };
    // std::array<uint16_t,8> &getErrorList() { return sensorData.errorList; };

    std::vector<uint8_t> getIOStatus() {std::vector<uint8_t> vec(sensorData.IOFlags.begin(),sensorData.IOFlags.end()); return vec; }; 
    std::vector<int16_t> getLaserDistance() {std::vector<int16_t> vec(sensorData.laserDistance.begin(),sensorData.laserDistance.end()); return vec; }; 
    std::vector<int16_t> getPressure() {std::vector<int16_t> vec(sensorData.pressure.begin(),sensorData.pressure.end()); return vec; }; 
    std::vector<int16_t> getPSource() {std::vector<int16_t> vec(sensorData.pSource.begin(),sensorData.pSource.end()); return vec; }; 
    std::vector<int16_t> getPSink() {std::vector<int16_t> vec(sensorData.pSink.begin(),sensorData.pSink.end()); return vec; }; 
    std::vector<float>   getJointf() {std::vector<float> vec(sensorData.jointStatef.begin(),sensorData.jointStatef.end()); return vec; }; 
    std::vector<int16_t> getRotationEncoder() {std::vector<int16_t> vec(sensorData.rotationEncoder.begin(),sensorData.rotationEncoder.end()); return vec; }; 
    std::vector<int16_t> getUltraSonic() {std::vector<int16_t> vec(sensorData.ultraSonic.begin(),sensorData.ultraSonic.end()); return vec; }; 
    std::vector<uint16_t> getErrorList() {std::vector<uint16_t> vec(sensorData.errorList.begin(),sensorData.errorList.end()); return vec; }; 
    // WS_STATUS commandQuery(uint32_t timeout = 50);

    

    WS_STATUS commandStart();
    WS_STATUS commandStop();

    /*X Y Z Yaw axis*/
    WS_STATUS commandJoint(std::array<float, 6> &jointd);
    WS_STATUS commandVelocity(std::array<int16_t, 6> &vd);
    WS_STATUS commandAcceleration(std::array<int16_t, 6> &ad);

    /*Pitch axis*/
    WS_STATUS commandPitch(float pitch_desired);
    WS_STATUS commandPitchUp();
    WS_STATUS commandPitchDown();
    WS_STATUS commandPitchHold(bool pitchhold);

    /*Chambers*/

    WS_STATUS commandGunElongate(int16_t targetPressure);
    WS_STATUS commandGuide(int16_t targetPressure);
    WS_STATUS commandGripperShift(int16_t targetPressure);
    WS_STATUS commandGrasp(int16_t targetPressure);
    WS_STATUS commandGunLock(int16_t targetPressure);

    WS_STATUS commandChamber(int pressureIndex, int16_t targetPressure);

    WS_STATUS commandElongateHold();

    /*magnet*/
    WS_STATUS commandMagnet(int16_t command_value);

    /*flash light*/
    WS_STATUS commandLight(std::string light_name, uint8_t light_cmd);

    /*signal light*/
    WS_STATUS commandSignalLight(LightSignalCommand signalLightMode);
    WS_STATUS commandSignalLightRGB(uint8_t red_status, uint8_t green_status, uint8_t yellow_status);
    std::string getSignalLightName(LightSignalCommand signalLightMode);
    std::string getSignalLightName();



    /*Basic Command Frame*/
    WS_STATUS command(uint16_t axisChosen, uint16_t axisConfig, int16_t xValue, int16_t yValue, int16_t zValue,
                          int16_t yawValue, int16_t pitchValue, int16_t elongateValue, int16_t cable, int16_t guide, int16_t shift,
                          int16_t grasp, int16_t lock, uint16_t ioValue, int16_t door);

    virtual void timeoutCallback() {};

    

    MaxwellSoftArmSensorData sensorData;
    uint16_t m_FrameTxRxPeriodMs = defaultFrameTxRxPeriodMs;
    uint16_t m_CommandTimeoutPeriodMs = defaultCommandTimeoutPeriodMs;
    int timeoutCount=0;
    int frameLostCount=0;
    int notAliveCount=0;
    int notAliveTime=0;
    int m_NAKTimeMax=0;

protected:
 
    float yaw_offset_;
    uint8_t m_flashLightGlobalCmd = 0;
    uint8_t m_flashLightLocalCmd = 0;

    static constexpr uint16_t queryFrameSize = 7;
    std::array<uint8_t, queryFrameSize> queryFrame;

    static constexpr uint16_t commandFrameBufSize = 37;
    struct CommandFrameMeta
    {
        uint64_t index;
        std::array<uint8_t, commandFrameBufSize> frame;
        uint16_t framesize;
        WS_STATUS commandResult;
    } commandFrame;
 
 

    bool commandInProgress;
 
    std::mutex commandMtx;
 
    WS_STATUS m_commandResult; 

    std::unique_ptr<std::jthread> communicationThread;
    



    int m_NAKTime=0;
    int notAliveNAKTimeThreshold=200;
 
    bool communication_is_alive;

    LightSignalCommand m_signal_light_command;

    const std::array<std::string,11> lightSignalCommandName={
        "LightBlueOn",
        "LightBlueBlink",
        "LightYellowOn",
        "LightYellowBlink",
        "LightGreenOn",
        "LightGreenBlink",
        "LightRedOn",
        "LightRedBlink",
        "LightOff",
        "LightWhiteOn",
        "LightWhiteBlink"
    };
};