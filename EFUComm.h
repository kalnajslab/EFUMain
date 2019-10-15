/*
 *  EFUComm.h
 *  Author:  Lars Kalnajs
 *  Created: October 2019
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the DIB and the EFU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef EFUComm_H
#define EFUComm_H

#include "SerialComm.h"

enum EFUMessages_t : uint8_t {
    EFU_NO_MESSAGE = 0,

    // DIB -> EFU (no params) Note these are for test only as comms is EFU-> DIB only
    EFU_SEND_STATUS, //1
    EFU_SEND_RECORD, //2
    EFU_RESET, //3

    // DIB-> EFU (with params) Note these are for test only as comms is EFU-> DIB only
    EFU_SET_HEATERS, //4
    EFU_GO_LOWPOWER, //5
    EFU_UPDATE_GPS, //6
    EFU_SOLAR_PWR, //7

    // EFU -> DIB (no params)
    EFU_DATA_RECORD,  // 8 binary transfer
    
    // EFU -> DIB (with params)
    EFU_STATUS, //9
    EFU_ERROR //10
};


class EFUComm : public SerialComm {
public:
    EFUComm(Stream * serial_port);
    ~EFUComm() { };

    // DIB -> EFU (with params) -----------------------
    bool TX_SetHeaters(float HeaterT); //Set the heater temperature (parameter not state)
    bool RX_SetHeaters(float * HeaterT);

    bool TX_LowPower(float survivalT); //go low power, heater set to survival_temp
    bool RX_LowPower(float * survivalT);

    bool TX_UpdateGPS(uint32_t GPSTime, float GPSlat, float GPSlon, uint16_t GPSAlt);
    bool RX_UpdateGPS(uint32_t * GPSTime, float * GPSlat, float * GPSlon, uint16_t * GPSAlt);
    
    bool TX_SolarPower(uint8_t solarChargerON); //sending true (!= 0) enables solar charger
    bool RX_SolarPower(uint8_t * solarChargerON);

    // EFU -> DIB (with params) -----------------------
    bool TX_Status(uint32_t EFUTime, float VBattery, float VSolarCharger, float V3v3, float PCBT, float BatteryT, float FiberT1, float FiberT2, float OAT, uint8_t HeaterStatus);
    bool RX_Status(uint32_t * EFUTime, float * VBattery, float * VSolarCharger, float * V3v3, float * PCBT, float * BatteryT, float * FiberT1, float * FiberT2, float * OAT, uint8_t * HeaterStatus);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

#endif /* EFUComm_H */
