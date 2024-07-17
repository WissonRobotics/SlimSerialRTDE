#ifndef __WISSONDUMMYSERVER_H
#define __WISSONDUMMYSERVER_H
#include "SlimSerialRTDE.h"
 

#define ADDRESS 0x88
#define CONTROL_FUNC 0x80
#define RESPONSE_FUNC 0x82


enum {
    COM_OK=0,
    COM_ERROR=1,
};


enum {
    cmd_query = 0,
    cmd_autoChargeStart = 1,
    cmd_autoDischarge=2,
    cmd_emergencyStop=3,
    cmd_home=4,
    cmd_manualCharge=5,
    cmd_manualDischarge=6,
    cmd_confirmChargedWell=7

};


enum {
    status_idle = 0,
    status_autoCharge = 1,
    status_Charging = 2,
    status_autoDischarge = 3,
    status_manualCharge = 4,
    status_manualReturn = 5,
    status_stopped = 6,
    status_homing = 7,
    status_error = 8, 
};

enum {
    error_none=0,
    error_motor=1,
    error_vision=2,
};

#define TIME_CHARGING_MAX       10
#define TIME_DISCHARGING        3
#define TIME_HOMING             1

class WISSON_DUMMY_SERVER{
public:
    WISSON_DUMMY_SERVER(std::string loggerName = "WISSON_DUMMY_SERVER");
    ~WISSON_DUMMY_SERVER();
    int setup(std::string portname);
    void frameCallback(std::vector<uint8_t> &rxFrame);
    void response();
     
    SlimSerialRTDE slimSerial;
 
    uint16_t command;

    uint16_t taskStatus;
    uint16_t taskError;
    std::string m_portname;
    uint16_t taskStatusLast;
    uint8_t chargeWellAck;
    std::function<void(std::vector<uint8_t>& rxFrame)> frameCallbackFunc;

    std::vector<std::string> command_string;
    std::vector<std::string> status_string;
    void runTaskStateMachine();
};

#endif