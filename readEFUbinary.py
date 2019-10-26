#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 17 19:25:31 2019
Routine to convert EFU binray telemetry packets to CSV files. 

Usage: python3 ConvertEFU filename.bin
Output: filenma.csv

@author: kalnajs
"""
import sys
import struct
import csv
import os
from datetime import datetime


def parseEFUDatatoCSV(binData, OutFile):
    
    start = binData.find(0x3B)+1 # Find the colon after the EFUComm header
    print('Colon found at: ' + str(start))
    binData = binData[start:]    
    #Print the number of lines in the file    
    print(len(data)/26)    
    
    
    with open(OutFile, mode='w') as out_file:
        file_writer = csv.writer(out_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        
        #read the header
        startTime = (struct.unpack_from('<I',binData,0)[0])
        GPSStartLat = (struct.unpack_from('<f',binData,4)[0])
        GPSStartLon = (struct.unpack_from('<f',binData,8)[0])
        V_3v3 = (struct.unpack_from('<H',binData,12)[0]) / 1000.0 #3v3 supply in volts
        V_Battery = (struct.unpack_from('<H',binData,14)[0]) / 1000.0 #Battery Voltage
        Up_Time = (struct.unpack_from('<I',binData,16)[0])/1000.0 # up time in seconds
       
        #write the header data to csv
        header = ['Profile Start Time', 'Initial Lat', 'Initial Lon',  '3.3V supply', 'Battery V', 'Up Time [s]']
        file_writer.writerow(header)
        header = [startTime,GPSStartLat, GPSStartLon, V_3v3, V_Battery,Up_Time]
        file_writer.writerow(header)
        header = (['Date String','Time POSIX','Altitude [m]','Latitude','Longitude','Fiber PRT #1 [C]',
                                 'Fiber PRT #2 [C]','OAT [C]','Battery T [C]','PCB T [C]','Battery Volts',
                                 'Samples in Averages','Battery Heater [1=On]','GPS Satellites'])
        file_writer.writerow(header)
        
        Time = startTime
        
        
        
        for y in range(int(len(binData)/26)-1):
           indx = (y+1)*26
           
           #Time is uint16, difference from start time
           Time = struct.unpack_from('<H',binData,indx)[0] + startTime  
           date_time = datetime.fromtimestamp(Time)
           d = date_time.strftime("%m/%d/%Y, %H:%M:%S")
           
           #Lat/long are floats
           Latitude = "%.6f"%(struct.unpack_from('<f',binData,indx + 2)[0])
           Longitude = "%.6f"%(struct.unpack_from('<f',binData,indx + 6)[0])
           
           #GPS Altitude uint16 in meters
           Altitude = struct.unpack_from('<H',binData,indx + 10)[0] 
           
           #Temperatures are uint16 in K divided by 100
           FiberPRT1_T = "%.2f"%((struct.unpack_from('<H',binData,indx + 12)[0]) / 100.0 - 273.15)
           FiberPRT2_T = "%.2f"%((struct.unpack_from('<H',binData,indx + 14)[0]) / 100.0 - 273.15)
           OAT_T = "%.2f"%((struct.unpack_from('<H',binData,indx + 16)[0]) / 100.0 - 273.15)
           Battery_T = "%.2f"%((struct.unpack_from('<H',binData,indx + 18)[0]) / 100.0 - 273.15)
           PCB_T = "%.2f"%((struct.unpack_from('<H',binData,indx + 20)[0]) / 100.0 - 273.15)
           V_Battery = "%.2f"%((float(struct.unpack_from('<B',binData,indx+23)[0])) / 100.0+2.0) #Battery voltage in V
           Samples_in_Average = ((struct.unpack_from('<B',binData,indx+22)[0]))
           GPS_Sats = (struct.unpack_from('<B',binData,indx+24)[0]) 
           Battery_Heater_State = (struct.unpack_from('<B',binData,indx+25)[0])  #Heater status, 1 = on
           
           
           file_writer.writerow([d,Time,Altitude,Latitude,Longitude,FiberPRT1_T,FiberPRT2_T, OAT_T, Battery_T,
                                 PCB_T, V_Battery,Samples_in_Average, Battery_Heater_State,GPS_Sats])
    

def main():
    
    #InputFile = sys.argv[1]
    #InputFile = '/Users/kalnajs/Documents/Strateole/Python/EFU_TM/EFU1203232.bin'
    InputDir = '/Users/kalnajs/Documents/Strateole/Python/EFU_TM/'
    
    directory = os.fsencode(InputDir)

    for file in os.listdir(directory):
        filename = os.fsdecode(file)
        if filename.endswith(".bin"): 
            InputFile = os.path.join(InputDir, filename)
            OutputFile = os.path.splitext(InputFile)[0] + '.csv'
            print("Processing: "+InputFile + " to: "+  OutputFile)
            with open(InputFile, "rb") as binary_file:
                 data = binary_file.read() # Read the whole file at once
            parseEFUDatatoCSV(data,OutputFile)
            continue
        else:
            continue

    #generate the output file name
    
    with open(InputFile, "rb") as binary_file:
    # Read the whole file at once
        data = binary_file.read()
        
    
    parseEFUDatatoCSV(data,OutputFile)
    
if __name__ == "__main__": 
  # calling main function 
  main() 