/*

EFUMain

Created by Doug Goetz starting in 12/2017 

*/

#ifndef EFULibrary_h
#define EFULibrary_h


// verify that all arduino librarys are being called from SAMD folder
#include <Arduino.h>
#include "SPI.h"
#include "Wire.h"
#include <RTCZero.h>
#include <Adafruit_SleepyDog.h>


// LTC2983 Temperature IC Libraries 
#include "Charge_LTC2983_configuration_constants.h"
#include "Charge_LTC2983_support_functions.h"
#include "Charge_LTC2983_table_coeffs.h"


//////////// Serial Port constants /////////////
#define FibSerial  Serial //TX pin for fiber optic transmitter
#define GPSSerial Serial1 //RX and TX from UBLOX

///////// Pin Definitions //////////////

#define CHIP_SELECT 5 //LTC2983 chip select
#define RESET 6 //LTC2983 reset pin

#define FiberPower 13 // fiber transmitter power

#define GPS_ExtInt 2 //Interput Pin to place UBLOX in sleep mode
#define GPS_Reset 3 //Reset pin for UBLOX
#define GPS_Power 4 //Power to SIP switch that controls 3v to UBLOX

#define SHUTDOWN 9 //solar input power switch
#define CHARGE_OFF 10 //battery charger switch
#define HEATER_ON 11 //battery heater switch

////////// Analog Channels ////////////////////
#define INPUT_V A0 //should be 3.3V
#define SOLAR_V A1 //solar charger voltage
#define BATT_V  A2 //lithium battery voltage

#define TXBUFSIZE 2000


/////////// definitions of buffers, variables, and strings //////////////////////

//UBLOX config bytes
// byte PowerSaveMode[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x48, 0x01, 0x62, 0x12};
// byte CyclicTracking[] = {0xB5, 0x62, 0x06, 0x3B, 0x30, 0x00, 0x02, 0x06, 0x00, 0x00, 0x00, 0x10, 0x4, 0x01, 0xE8,0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x02};
// byte GNSSConfig[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2C, 0x4D}; 
// byte SaveUBLOXConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA9};
// byte GNGGL_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
// byte GNGSA_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
// byte GNGSV_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
// byte GNVTG_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
// byte UBX_Set_Airborne[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 
//                               0x05, 0x00, // mask (only update dyn model and fix mode)
// 	                          0x7, // dynamic model = airborne <2g
// 	                          0x3, // fix mode = 2D/3D
// 	                          0x0, 0x0, 0x0, 0x0, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
// 	                          0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 
// 	                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
// 	                          0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xf9}; // 0x1d, 0xd8 for 0x2 fix mode


class EFULibrary
{
  public:
    EFULibrary(int pin);
    
    void SetUp(); //Sets safe configuration for startup and watchdog reset
    
    //LTC Temperature measurement 
    void ConfigureChannels(); //Configure LTC2983 Channel settings
    void configure_global_parameters(); //configures globals for LTC chip
    void configure_memory_table(); //set up the coefs for the thermistors used
    float MeasureLTC2983(int channel);//returns the temperature of a given channel in degrees C.
    float MeasureLTC2983_V(int channel);//returns the temperature of a given channel in degrees C.
    void LTC_sleep(); //used to put LTC in low power sleep mode.
    void LTCReset(); //Reset the LTC part if it glitches.

    //Fiber TX on/off
    void FiberTXOff();  
    void FiberTXOn();

    // UBLOX GPS controls
    void GPSon();
    void GPSoff();
    void GPSreset(byte *);
    void configureUblox(byte *);
    void calcChecksum(byte *, byte);
    void sendUBX(byte *, byte);
    byte getUBX_ACK(byte *);
    void setBaud(byte);
    bool gpsStatus[7] = {false, false, false, false, false, false, false};

    // Generic Solar Charger functions
    void SolarOff(); //only off during reset
    void SolarOn();
    void ChargerOn();
    void ChargerOff();
    void HeaterOn();
    void HeaterOff();

    

    unsigned long counter;
    unsigned long lastcount;
    
  private:
    int _pin;

};

#endif
