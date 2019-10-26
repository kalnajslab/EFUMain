
#include "EFULibrary.h"




EFULibrary::EFULibrary(int pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
  delay(1000);//while (!Serial); // Wait until Serial is ready
}    

void EFULibrary::SetUp(){
   
  //configure comms 
  // setup native USB for debugging
  //SerialUSB.begin(9600);
  FibSerial.begin(115200);
  GPSSerial.begin(9600);

  //GPS pins
	pinMode(GPS_Power, OUTPUT); //digitalwrite high to turn on
  GPSoff();
  
  //Fiber TX DIO
	pinMode(FiberPower, OUTPUT); //digitalwrite high to turn on
  //FiberTXOn();
  FiberTXOff();


	//LTC Temperature IC Pins
   pinMode(CHIP_SELECT, OUTPUT); // Configure chip select pin on Linduino
   pinMode(RESET,OUTPUT);
   digitalWrite(RESET, HIGH);
   delay(100);
   SPI.begin();
   SPI.setClockDivider(SPI_CLOCK_DIV128);

  //Battery Charger and Heater
  pinMode(SHUTDOWN,OUTPUT); //or __charge__
  SolarOff(); //solar off during startup
  
  pinMode(CHARGE_OFF, OUTPUT); //LED mosfet for charger
  ChargerOn();//charger on


  pinMode(HEATER_ON, OUTPUT); // LED mosfet for heater
  HeaterOff(); //heater off

  analogReference(AR_EXTERNAL);
  analogWriteResolution(10);//Full DAQ resolution for arduino zero 

  SerialUSB.println("EFU Configured");
}

void EFULibrary::ConfigureChannels(){
   
   uint8_t channel_number;
   uint32_t channel_assignment_data;
   
 // ----- Channel 2: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0xFA000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 1000.
  assign_channel(CHIP_SELECT, 2, channel_assignment_data);
  // ----- Channel 4: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__2 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(CHIP_SELECT, 4, channel_assignment_data);
  // ----- Channel 6: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__2 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(CHIP_SELECT, 6, channel_assignment_data);
  // ----- Channel 14: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(CHIP_SELECT, 14, channel_assignment_data);
  // ----- Channel 16: Assign RTD PT-100 -----
  channel_assignment_data = 
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__14 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__50UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(CHIP_SELECT, 16, channel_assignment_data);
  // ----- Channel 18: Assign RTD PT-100 -----
  channel_assignment_data = 
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__14 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__50UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(CHIP_SELECT, 18, channel_assignment_data);
  // ----- Channel 20: Assign RTD PT-100 -----
  channel_assignment_data = 
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__14 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__50UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(CHIP_SELECT, 20, channel_assignment_data);

}
void EFULibrary::configure_memory_table()
{
    uint16_t start_address;
    
    //Configurations generated using Analog Devices LTC2983 Windows Demo Program
    
    //Configuration for NTCLE413E2103F102L
    // From: http://www.vishay.com/docs/29078/ntcle413.pdf
    
    uint32_t NTCLE413E2103F102L_steinhart_hart_coefficients[] =
    {
        979823667,  // -- For coefficient 0.0008809000137262046
        964985049,  // -- For coefficient 0.00025273000937886536
        0,  // -- For coefficient 0.0
        877110214,  // -- For coefficient 1.8592899664326978e-07
        0,  // -- For coefficient 0.0
        0   // -- For coefficient 0.0
    };
    start_address = 772;    // Real address = 6*30 + 0x250 = 772
    write_custom_steinhart_hart(CHIP_SELECT, NTCLE413E2103F102L_steinhart_hart_coefficients, start_address);
    
}

void EFULibrary::configure_global_parameters(){    

  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
    REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xFF, 0);
  

}

float EFULibrary::MeasureLTC2983(int channel){
   float temp;
    temp =  measure_channel(CHIP_SELECT, channel, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
   return temp;
} 

float EFULibrary::MeasureLTC2983_V(int channel){
   float temp;
    temp =  measure_channel(CHIP_SELECT, channel, VOLTAGE);
   return temp;
} 

void EFULibrary::LTCReset()
{
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  wait_for_process_to_finish(CHIP_SELECT);
  configure_memory_table();
  ConfigureChannels();
  configure_global_parameters(); 
}

void EFULibrary::LTC_sleep(){

	transfer_byte(CHIP_SELECT, WRITE_TO_RAM, COMMAND_STATUS_REGISTER, SLEEP_BYTE);	
} 

void EFULibrary::FiberTXOff(){
  digitalWrite(FiberPower, LOW);
}

void EFULibrary::FiberTXOn(){
  digitalWrite(FiberPower, HIGH);
}

void EFULibrary::GPSon(){
  digitalWrite(GPS_Power, HIGH);
}

void EFULibrary::GPSoff(){
  digitalWrite(GPS_Power, LOW);
}


void EFULibrary::SolarOff(){
  digitalWrite(SHUTDOWN, LOW); 
  //SolarChargeState = 0;
}

void EFULibrary::SolarOn(){
  digitalWrite(SHUTDOWN, HIGH);
  //SolarChargeState = 1; 
}

void EFULibrary::ChargerOn(){
  digitalWrite(CHARGE_OFF, HIGH);
}

void EFULibrary::ChargerOff(){
  digitalWrite(CHARGE_OFF, LOW);
}

void EFULibrary::HeaterOn(){
  digitalWrite(HEATER_ON, HIGH); 
  //BattHeaterState = 1;
}

void EFULibrary::HeaterOff(){
  digitalWrite(HEATER_ON, LOW); 
  //BattHeaterState = 0;
}

void EFULibrary::GPSreset(byte *settingsArrayPointer)
{
  GPSoff();
  delay(1000);
  GPSon();
  configureUblox(settingsArrayPointer);
}


void EFULibrary::configureUblox(byte *settingsArrayPointer) {
    byte gpsSetSuccess = 0;
    SerialUSB.println("Configuring u-Blox GPS initial state...");
    
    //Generate the configuration string for Navigation Mode
    byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    calcChecksum(&setNav[2], sizeof(setNav) - 4);
    
    //Generate the configuration string for Data Rate
    byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
    calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);
    
    //Generate the configuration string for Baud Rate
    byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);
    
    byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
    byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
    byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
    byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
    byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
    
    delay(2500);
    
    while(gpsSetSuccess < 3)
    {
        SerialUSB.print("Setting Navigation Mode... ");
        sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
        gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
        if (gpsSetSuccess == 5) {
            gpsSetSuccess -= 4;
            setBaud(settingsArrayPointer[4]);
            delay(1500);
            byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
            sendUBX(lowerPortRate, sizeof(lowerPortRate));
            GPSSerial.begin(9600);
            delay(2000);
        }
        if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
        if (gpsSetSuccess == 10) gpsStatus[0] = true;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("Navigation mode configuration failed.");
    gpsSetSuccess = 0;
    while(gpsSetSuccess < 3) {
        SerialUSB.print("Setting Data Update Rate... ");
        sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
        gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function
        if (gpsSetSuccess == 10) gpsStatus[1] = true;
        if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("Data update mode configuration failed.");
    gpsSetSuccess = 0;
    
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
        SerialUSB.print("Deactivating NMEA GLL Messages ");
        sendUBX(setGLL, sizeof(setGLL));
        gpsSetSuccess += getUBX_ACK(&setGLL[2]);
        if (gpsSetSuccess == 10) gpsStatus[2] = true;
        if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA GLL Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
        SerialUSB.print("Deactivating NMEA GSA Messages ");
        sendUBX(setGSA, sizeof(setGSA));
        gpsSetSuccess += getUBX_ACK(&setGSA[2]);
        if (gpsSetSuccess == 10) gpsStatus[3] = true;
        if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA GSA Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
        SerialUSB.print("Deactivating NMEA GSV Messages ");
        sendUBX(setGSV, sizeof(setGSV));
        gpsSetSuccess += getUBX_ACK(&setGSV[2]);
        if (gpsSetSuccess == 10) gpsStatus[4] = true;
        if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA GSV Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
        SerialUSB.print("Deactivating NMEA RMC Messages ");
        sendUBX(setRMC, sizeof(setRMC));
        gpsSetSuccess += getUBX_ACK(&setRMC[2]);
        if (gpsSetSuccess == 10) gpsStatus[5] = true;
        if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA RMC Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
        SerialUSB.print("Deactivating NMEA VTG Messages ");
        sendUBX(setVTG, sizeof(setVTG));
        gpsSetSuccess += getUBX_ACK(&setVTG[2]);
        if (gpsSetSuccess == 10) gpsStatus[6] = true;
        if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA VTG Message Deactivation Failed!");
    
    gpsSetSuccess = 0;
    if (settingsArrayPointer[4] != 0x25) {
        SerialUSB.print("Setting Port Baud Rate... ");
        sendUBX(&setPortRate[0], sizeof(setPortRate));
        setBaud(settingsArrayPointer[4]);
        SerialUSB.println("Success!");
        delay(500);
    }
}


void EFULibrary::calcChecksum(byte *checksumPayload, byte payloadSize) {
    byte CK_A = 0, CK_B = 0;
    for (int i = 0; i < payloadSize ;i++) {
        CK_A = CK_A + *checksumPayload;
        CK_B = CK_B + CK_A;
        checksumPayload++;
    }
    *checksumPayload = CK_A;
    checksumPayload++;
    *checksumPayload = CK_B;
}

void EFULibrary::sendUBX(byte *UBXmsg, byte msgLength) {
    for(int i = 0; i < msgLength; i++) {
        GPSSerial.write(UBXmsg[i]);
        GPSSerial.flush();
    }
    GPSSerial.println();
    GPSSerial.flush();
}


byte EFULibrary::getUBX_ACK(byte *msgID) {
    byte CK_A = 0, CK_B = 0;
    byte incoming_char;
    boolean headerReceived = false;
    unsigned long ackWait = millis();
    byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int i = 0;
    while (1) {
        if (GPSSerial.available()) {
            incoming_char = GPSSerial.read();
            if (incoming_char == ackPacket[i]) {
                i++;
            }
            else if (i > 2) {
                ackPacket[i] = incoming_char;
                i++;
            }
        }
        if (i > 9) break;
        if ((millis() - ackWait) > 1500) {
            SerialUSB.println("ACK Timeout");
            return 5;
        }
        if (i == 4 && ackPacket[3] == 0x00) {
            SerialUSB.println("NAK Received");
            return 1;
        }
    }
    
    for (i = 2; i < 8 ;i++) {
        CK_A = CK_A + ackPacket[i];
        CK_B = CK_B + CK_A;
    }
    if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
        //    Serial.println("Success!");
        //    Serial.print("ACK Received! ");
        //    printHex(ackPacket, sizeof(ackPacket));
        return 10;
    }
    else {
        //    Serial.print("ACK Checksum Failure: ");
        //    printHex(ackPacket, sizeof(ackPacket));
        //    delay(1000);
        return 1;
    }
}


void EFULibrary::setBaud(byte baudSetting) {
    if (baudSetting == 0x12) GPSSerial.begin(4800);
    if (baudSetting == 0x4B) GPSSerial.begin(19200);
    if (baudSetting == 0x96) GPSSerial.begin(38400);
    if (baudSetting == 0xE1) GPSSerial.begin(57600);
    if (baudSetting == 0xC2) GPSSerial.begin(115200);
    if (baudSetting == 0x84) GPSSerial.begin(230400);
}


