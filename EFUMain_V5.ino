#include "EFULibrary.h"
#include "EFUComm.h"
#include <TinyGPS++.h>
#include <time.h>



RTCZero rtc;
EFUComm efucomm(&Serial1);
TinyGPSPlus gps;
EFULibrary EFU(13);

#define TM_BUFFER_LENGTH 62
#define TM_RECORD_LENGTH 12
#define TM_SEND_RETRIES 3 //try sending the TM buffer three times in a row

#define DEAD_BAND 1.0  //Heater dead band
#define MAX_BATTERY_T 50.0  //Maximum Battery T, stop heating above this

//Default Heater setpoints
float NominalSetPoint = -28.0;  //Set point for main board during normal ops
float BoostSetPoint = 45.0;  //Set point for battery when excess solar available
uint8_t BattHeaterState = 0;

uint16_t EFUData[TM_BUFFER_LENGTH][TM_RECORD_LENGTH];  //1.488 KB (61 records)

//LTC Temperatures
float PWD_Therm_T; //Ch 4
float Batt_Therm_T; //Ch 6
float FibPRT1_T; //Ch 16
float FibPRT2_T; //Ch 18
float OAT_T; //Ch 20 

//LTC Temperature accumulators
float PWD_Therm_T_ave; //Ch 4
float Batt_Therm_T_ave; //Ch 6
float FibPRT1_T_ave; //Ch 16
float FibPRT2_T_ave; //Ch 18
float OAT_T_ave; //Ch 20 
int temperatureAverageIndex = 0;

//Analog
float vIn;
float vBatt;
float vSolar;

//Analog accumulators
float vIn_ave;
float vBatt_ave;
float vSolar_ave;
int analogAverageIndex = 0;

//Heater
bool BoostHeatActive = false;

int RecordIndex = 0;



void setup()
{
    EFU.SetUp(); //Set up the ports and DIO

    rtc.begin();  //Initialize the RTC

    EFU.ConfigureChannels(); //Configure LTC2983 Channel settings
    EFU.configure_global_parameters(); //configures globals for LTC chip

    EFU.GPSon(); //Turn on the GPS
    delay(1000); //Wait for GPS to boot up
    
    /*Set up the UBLOX - note make sure it will work above 12km */
    byte settingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //
    EFU.configureUblox(settingsArray); 

    EFU.ChargerOn(); //Turn on the charger
    updateRTCfromGPS();

    SerialUSB.println("Setup Complete");
}

void loop()
{
    SerComRX();  //check for data on the serial port
    if (GPSSerial.available())  //Check for GPS serial
        gps.encode(GPSSerial.read());
  
    temperatureAverageIndex = AddToAverageTempC(temperatureAverageIndex);
    analogAverageIndex = AddToAnalogAverage(analogAverageIndex);

    if (rtc.getSeconds() < 3 && temperatureAverageIndex > 10) //  do this within 3 seconds of the minute roll over
    {
        calculateAverages(temperatureAverageIndex, analogAverageIndex);
        RecordIndex = SaveRecord(RecordIndex);
        SetHeater(NominalSetPoint, BoostSetPoint);
        analogAverageIndex = 0;
        temperatureAverageIndex = 0;
    }

    if (rtc.getMinutes() == 0 && RecordIndex > 10) // if we are within 1 minute of the hour and have at least 10 records to send
    {
        EFU.FiberTXOn();
        sendTM(RecordIndex);
        EFU.FiberTXOff();
        RecordIndex = 0;
        updateRTCfromGPS();
    }

}

bool updateRTCfromGPS()
{
    struct tm GPStm;
    time_t GPStime;
    if ((gps.time.age() < 1500) && gps.time.isValid())  //we have a GPS time fix within the last 1.5s
    {
        GPStm.tm_isdst = -1;
        GPStm.tm_yday = 0;
        GPStm.tm_wday = 0;
        GPStm.tm_hour = gps.time.hour(); 
        GPStm.tm_min = gps.time.minute();
        GPStm.tm_sec = gps.time.second();
        GPStm.tm_mday = gps.date.day();
        GPStm.tm_mon = gps.date.month();
        GPStm.tm_year = gps.date.year() + 30; //gps is since 2000, tmElements_t is since 1970
        GPStime = mktime(&GPStm); //convert GPS time to time_t

        if (abs(rtc.getEpoch() - GPStime) > 1) //if the clock is more than 1 second off
        {
            rtc.setEpoch(GPStime);
            SerialUSB.print("Updating RTC to GPS time: "); SerialUSB.println(GPStime);
            return true;
        }

        return false;

    }
}

int SaveRecord(int dataindx)
{
    float single;
    byte *bytes;
    
    if (dataindx >= TM_BUFFER_LENGTH) // wrap around if we somehow over fill the buffer.
        dataindx = 1;
    
    uint32_t GPSStartTime = rtc.getEpoch(); //get time as time_t from Teensy clock (4 bytes)

    if(dataindx == 0) //first line of array is header line
    {
        
        EFUData[0][0] = (uint16_t) (GPSStartTime);
        EFUData[0][1] = (uint16_t) (GPSStartTime >> 16);  
        
        float GPSStartLat = gps.location.lat();
        single = (float)GPSStartLat;  //convert double to single float
        bytes = (byte *)&single;  // convert float to bytes
        EFUData[0][2] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        EFUData[0][3] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        float GPSStartLon = gps.location.lng();
        single = (float)GPSStartLon;  //convert double to single float
        bytes = (byte *)&single;
        //*(float*)(bytes) = single;  // convert float to bytes
        EFUData[0][4] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        EFUData[0][5] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        EFUData[0][6] = (uint16_t)(vIn*1000);  //Voltage in mV
        EFUData[0][7] = (uint16_t)(vBatt*1000);  //Voltage in mV
        //Currently unused
        EFUData[0][8] = 0xFFFF;
        EFUData[0][9] = 0xFFFF;
        EFUData[0][10] = 0xFFFF;
        EFUData[0][11] = 0xFFFF;

        dataindx = 1;
    }

    EFUData[dataindx][0] = (uint16_t)(rtc.getEpoch() - GPSStartTime); // Elapsed time in seconds
    //Send as 32 bit floats
    single = (float)gps.location.lat();
    //*(float*)(bytes) = single;  // convert float to bytes
    bytes = (byte *)&single;
    EFUData[0][1] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
    EFUData[0][2] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
    single = (float)gps.location.lng();
    //*(float*)(bytes) = single;  // convert float to bytes
    bytes = (byte *)&single;
    EFUData[0][3] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
    EFUData[0][4] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)   
        
    EFUData[dataindx][5] = (uint16_t)(gps.altitude.meters());  //Altitude in meters
    EFUData[dataindx][6] = (uint16_t)((FibPRT1_T + 273.15)*100);  //Fiber Temp 1 in K to 0.01K
    EFUData[dataindx][7] = (uint16_t)((FibPRT2_T + 273.15)*100);  //Fiber Temp 2 in K to 0.01K
    EFUData[dataindx][8] = (uint16_t)((OAT_T + 273.15)*100);  //Outside air temp in K to 0.01K
    EFUData[dataindx][9] = (uint16_t)((Batt_Therm_T + 273.15)*100);  //Battery temp in K to 0.01K
    EFUData[dataindx][10] = (uint16_t)((PWD_Therm_T + 273.15)*100);  //PCB temp in K to 0.01K
    EFUData[dataindx][11] = (uint16_t)((vBatt - 2.0)*100) << 8 + BattHeaterState*128 + (uint8_t)gps.satellites.value();  
    //Battery V (2.0 - 4.55V at 0.01V resolution), heater status and GPS sattelites
    dataindx++;
    return dataindx;
}

void sendTM(int index)
{
 /*This sends a binary message with the last hours worth
 of data.   We send it X times then clear the buffer.
 */
 int i = 0;

 for (i = 0; i < TM_SEND_RETRIES; i++) //send the buffer TM_SEND_RETRIES times
 {
    efucomm.AssignBinaryTXBuffer((uint8_t *) EFUData, index * TM_RECORD_LENGTH * 2,index * TM_RECORD_LENGTH *2);
    efucomm.TX_Bin(EFU_SEND_RECORD);
    delay(3000); //wait a few seconds then try again
 }
RecordIndex = 0; //reset the index into the data array
}

bool SerComRX()
 {
     /* Recieves commands from the PIB and parses them.  
     *  If we receive an immediate change mode command then this 
     * returns true, which should cause the mode to return to main
     * and update to the new mode
     */
     int8_t tmp1;
     int32_t tmp2;
     uint32_t tmp3;

     switch (efucomm.RX()) {
         case ASCII_MESSAGE:
            Serial.print("Received message: "); Serial.println(efucomm.ascii_rx.msg_id);
            SerialUSB.print("Message Content: "); SerialUSB.println(efucomm.ascii_rx.buffer);
            switch (efucomm.ascii_rx.msg_id){
                case EFU_SEND_STATUS:
                    tmp3 = rtc.getEpoch();
                    tmp1 = efucomm.TX_Status(tmp3, vBatt,vBatt,vIn, PWD_Therm_T, Batt_Therm_T, FibPRT1_T, FibPRT2_T, OAT_T, BattHeaterState);
                    SerialUSB.println("Received EFU_SEND_STATUS");
                    return false;
                case EFU_SEND_RECORD:
                    SerialUSB.println("[nominal] Received PU_SEND_PROFILE_RECORD");
                    sendTM(RecordIndex);
                    return false;
                case EFU_RESET:
                    efucomm.TX_Ack(EFU_RESET,true);
                    SerialUSB.println("Rebooting in 3 seconds");
                    delay(3000);
                    return false;
                case EFU_SET_HEATERS:
                    tmp1 = efucomm.RX_SetHeaters(&NominalSetPoint);
                    efucomm.TX_Ack(EFU_SET_HEATERS,tmp1);
                    if(tmp1)
                    {
                        SerialUSB.println("Received EFU_SET_HEATER");
                        SerialUSB.println(NominalSetPoint);
                    }
                    return false;
               
                default:
                    efucomm.TX_Ack(efucomm.ascii_rx.msg_id,false);
                    return false;   
            }
            case ACK_MESSAGE:
                Serial.print("ACK/NAK for msg: "); Serial.println(efucomm.ack_id);
                Serial.print("Value: ");
                efucomm.ack_value ? Serial.println("ACK") : Serial.println("NAK");
                return false;
            case NO_MESSAGE:
            default:
                return false;                    

        }
 }

bool SetHeater(float LocalNominalSetPoint, float LocalBoostSetPoint)
{
    if (vBatt > 4.10)
        BoostHeatActive = true;
    if (vBatt < 3.90)
        BoostHeatActive = false;
    
    if(BoostHeatActive) //Battery is fully charged -> in boost mode control on TBatt
    {
        if(Batt_Therm_T < LocalBoostSetPoint)
        {
            digitalWrite(HEATER_ON, HIGH);
            BattHeaterState = 1; 
        }
        if(Batt_Therm_T > (LocalBoostSetPoint + DEAD_BAND))
        {
            digitalWrite(HEATER_ON, LOW);
            BattHeaterState = 0; 
        }
    }

    if ((PWD_Therm_T < LocalNominalSetPoint) && (Batt_Therm_T < MAX_BATTERY_T))
    {
        digitalWrite(HEATER_ON, HIGH);
        BattHeaterState = 1; 
    }
    if(PWD_Therm_T > (LocalNominalSetPoint + DEAD_BAND))
    {
        digitalWrite(HEATER_ON, LOW);
        BattHeaterState = 0; 
    }
}

int AddToAverageTempC(int aveindx){

	float tempFibPRT1_T = EFU.MeasureLTC2983(16); 
    if (tempFibPRT1_T < -100.0 || tempFibPRT1_T > 100.0)
        FibPRT1_T_ave += FibPRT1_T_ave/(float)aveindx;
    else 
        FibPRT1_T_ave += tempFibPRT1_T;
    
    float tempFibPRT2_T = EFU.MeasureLTC2983(18); 
    if (tempFibPRT2_T < -100.0 || tempFibPRT2_T > 100.0)
        FibPRT2_T_ave += FibPRT2_T_ave/(float)aveindx;
    else 
        FibPRT2_T_ave += tempFibPRT2_T;

    float tempOAT_T = EFU.MeasureLTC2983(20); 
    if (tempOAT_T < -100.0 || tempOAT_T > 100.0)
        OAT_T_ave += OAT_T_ave/(float)aveindx;
    else 
        OAT_T_ave += tempOAT_T;

    float tempPWD_Therm_T = EFU.MeasureLTC2983(6); 
    if (tempPWD_Therm_T< -100.0 || tempPWD_Therm_T > 100.0)
        PWD_Therm_T_ave += PWD_Therm_T_ave/(float)aveindx;
    else 
        PWD_Therm_T_ave += tempPWD_Therm_T;
    
    float tempBatt_Therm_T = EFU.MeasureLTC2983(4); 
    if (tempBatt_Therm_T< -100.0 || tempBatt_Therm_T > 100.0)
        Batt_Therm_T_ave += Batt_Therm_T_ave/(float)aveindx;
    else 
        Batt_Therm_T_ave += tempBatt_Therm_T;

    return aveindx+1;
} 

int AddToAnalogAverage(int aveindx) {
    int val = 0;
    float Vout = 0.0;

   val = analogRead(INPUT_V); 
   Vout = val*(3.0/1023.0);
   vIn_ave += ((Vout*(10000.0+23200.0))/23200.0);  
   SerialUSB.print(vIn);
   SerialUSB.println("");

   val = analogRead(BATT_V);    
   Vout = val*(3.0/1023.0);
   vBatt_ave += ((Vout*(4700.0+4700.0))/4700.0); 
   SerialUSB.print(vBatt);
   SerialUSB.print(", ");

   val = analogRead(SOLAR_V);
   Vout = val*(3/1023.0);
   vSolar_ave +=((Vout*(10000.0+10000.0))/10000.0); 
   SerialUSB.print(vSolar);
   SerialUSB.print(", "); 

   return aveindx+1;
}

void calculateAverages(int tempAveIndx, int analogAveIndx)
{
    float tempAveIndx_f = (float)tempAveIndx;
    float analogAveIndx_f = (float)analogAveIndx;

    FibPRT1_T = FibPRT1_T_ave/tempAveIndx_f;
    FibPRT2_T = FibPRT2_T_ave/tempAveIndx_f;
    Batt_Therm_T = Batt_Therm_T_ave/tempAveIndx_f;
    PWD_Therm_T = PWD_Therm_T_ave/tempAveIndx_f;
    OAT_T = OAT_T_ave/tempAveIndx_f;

    vBatt = vBatt_ave/analogAveIndx_f;
    vIn = vIn_ave/analogAveIndx_f;
    vSolar = vSolar_ave/analogAveIndx_f;

    FibPRT1_T_ave = 0.0;
    FibPRT2_T_ave = 0.0;
    Batt_Therm_T_ave = 0.0;
    PWD_Therm_T_ave = 0.0;
    OAT_T_ave = 0.0;
    
    vBatt_ave = 0.0;
    vIn_ave = 0.0;
    vSolar_ave = 0.0;
}
