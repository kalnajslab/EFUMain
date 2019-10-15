/*
 *  EFUComm.cpp
 *  Author:  Lars Kalnajs
 *  Created: October 2019
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the EFU and DIB. The class inherits its protocol from the SerialComm
 *  class.
 */

#include "EFUComm.h"

EFUComm::EFUComm(Stream * serial_port)
    : SerialComm(serial_port)
{
}

// DIB -> EFU (with params) ---------------------------

bool EFUComm::TX_SetHeaters(float HeaterT)
{
    if (!Add_float(HeaterT)) return false;
   
    TX_ASCII(EFU_SET_HEATERS);

    return true;
}

bool EFUComm::RX_SetHeaters(float * HeaterT)
{
    float temp1;

    if (!Get_float(&temp1)) return false;
   
    *HeaterT = temp1;

    Serial.println(temp1);

    return true;
}

bool EFUComm::TX_LowPower(float survivalT)
{
    if (!Add_float(survivalT)) return false;
   
    TX_ASCII(EFU_GO_LOWPOWER);

    return true;
}

bool EFUComm::RX_LowPower(float * survivalT)
{
    float temp1;

    if (!Get_float(&temp1)) return false;
   
    *survivalT = temp1;

    return true;
}

bool EFUComm::TX_UpdateGPS(uint32_t GPSTime, float GPSlat, float GPSlon, uint16_t GPSAlt)
{
    if (!Add_uint32(GPSTime)) return false;
    if (!Add_float(GPSlat)) return false;
    if (!Add_float(GPSlon)) return false;
    if (!Add_uint16(GPSAlt)) return false;
    
    TX_ASCII(EFU_UPDATE_GPS);

    return true;
}

bool EFUComm::RX_UpdateGPS(uint32_t * GPSTime, float * GPSlat, float * GPSlon, uint16_t * GPSAlt)
{
    uint32_t temp1;
    uint16_t temp4;
    float temp2, temp3;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_uint16(&temp4)) return false;
   
    *GPSTime= temp1;
    *GPSlat = temp2;
    *GPSlon = temp3;
    *GPSAlt = temp4;

    return true;
}

bool EFUComm::TX_SolarPower(uint8_t solarChargerON)
{
    if (!Add_uint8(solarChargerON)) return false;

    TX_ASCII(EFU_SOLAR_PWR);

    return true;
}

bool EFUComm::RX_SolarPower(uint8_t * solarChargerON)
{
    uint8_t temp;
    if(!Get_uint8(&temp)) return false;

    *solarChargerON = temp;

    return true;
}

bool EFUComm::TX_Status(uint32_t EFUTime, float VBattery, float VSolarCharger, float V3v3, float PCBT, float BatteryT, float FiberT1, float FiberT2, float OAT, uint8_t HeaterStatus)
{
    if (!Add_uint32(EFUTime)) return false;
    if (!Add_float(VBattery)) return false;
    if (!Add_float(VSolarCharger)) return false;
    if (!Add_float(V3v3)) return false;
    if (!Add_float(PCBT)) return false;
    if (!Add_float(BatteryT)) return false;
    if (!Add_float(FiberT1)) return false;
    if (!Add_float(FiberT2)) return false;
    if (!Add_float(OAT)) return false;
    if (!Add_uint8(HeaterStatus)) return false;

    TX_ASCII(EFU_STATUS);

    return true;
}

bool EFUComm::RX_Status(uint32_t * EFUTime, float * VBattery, float * VSolarCharger, float * V3v3, float * PCBT, float * BatteryT, float * FiberT1, float * FiberT2, float * OAT, uint8_t * HeaterStatus)
{
    uint32_t temp1;
    float temp2, temp3, temp4, temp5,temp6, temp7, temp8, temp9;
    uint8_t temp10;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_float(&temp4)) return false;
    if (!Get_float(&temp5)) return false;
    if (!Get_float(&temp6)) return false;
    if (!Get_float(&temp7)) return false;
    if (!Get_float(&temp8)) return false;
    if (!Get_float(&temp9)) return false;
    if (!Get_uint8(&temp10)) return false;

    *EFUTime = temp1;
    *VBattery = temp2;
    *VSolarCharger = temp3;
    *V3v3 = temp4;
    *PCBT = temp5;
    *BatteryT = temp6;
    *FiberT1 = temp7;
    *FiberT2 = temp8;
    *OAT = temp9;
    *HeaterStatus = temp10;

    return true;
}

// -- EFU to DIB error string

bool EFUComm::TX_Error(const char * error)
{
    if (Add_string(error)) return false;

    TX_ASCII(EFU_ERROR);

    return true;
}

bool EFUComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}


