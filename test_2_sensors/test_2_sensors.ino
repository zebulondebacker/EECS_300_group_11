/**
 ******************************************************************************
 * @file    VL53L3CX_Sat_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    30 July 2020
 * @brief   Arduino test application for the STMicrolectronics VL53L3CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>© COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use this sketch you need to connect the VL53L3CX satellite sensor directly to the Nucleo board with wires in this way:
 * pin 1 (Interrupt) of the VL53L3CX satellite connected to pin A2 of the Nucleo board 
 * pin 2 (SCL_I) of the VL53L3CX satellite connected to pin D15 (SCL) of the Nucleo board with a Pull-Up resistor of 4.7 KOhm
 * pin 3 (XSDN_I) of the VL53L3CX satellite connected to pin A1 of the Nucleo board
 * pin 4 (SDA_I) of the VL53L3CX satellite connected to pin D14 (SDA) of the Nucleo board with a Pull-Up resistor of 4.7 KOhm
 * pin 5 (VDD) of the VL53L3CX satellite connected to 3V3 pin of the Nucleo board
 * pin 6 (GND) of the VL53L3CX satellite connected to GND of the Nucleo board
 * pins 7, 8, 9 and 10 are not connected.
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial
#define SDA_pin 21
#define SCL_pin 22
#define XSHUT1_pin 18
#define GPIO1_pin 19 

#define XSHUT2_pin 17
#define GPIO2_pin 5

#define XSHUT3_pin 4
#define GPIO3_pin 16

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define LedPin LED_BUILTIN

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, XSHUT1_pin);
VL53LX sensor_vl53lx2_sat(&DEV_I2C, XSHUT2_pin);
VL53LX sensor_vl53lx3_sat(&DEV_I2C, XSHUT3_pin);

/* Setup ---------------------------------------------------------------------*/

void setup()
{
   // Led.
   pinMode(LedPin, OUTPUT);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin();

     // Set Addresses
   digitalWrite(XSHUT1_pin, LOW);
   digitalWrite(XSHUT2_pin, LOW);
   digitalWrite(XSHUT3_pin, LOW);
   delay(10);
   
   digitalWrite(XSHUT1_pin, HIGH);
   sensor_vl53lx_sat.begin();
   sensor_vl53lx_sat.InitSensor(0x12);

   digitalWrite(XSHUT2_pin, HIGH);
   sensor_vl53lx2_sat.begin();
   sensor_vl53lx2_sat.InitSensor(0x14);

   digitalWrite(XSHUT3_pin, HIGH);
   sensor_vl53lx3_sat.begin();
   sensor_vl53lx3_sat.InitSensor(0x52);
   // Start Measurements
   sensor_vl53lx_sat.VL53LX_StartMeasurement();
   sensor_vl53lx2_sat.VL53LX_StartMeasurement();
   sensor_vl53lx3_sat.VL53LX_StartMeasurement();
//     while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\nI2C Scanner");
}

void loop()
{
  /*
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan

*/
  
   VL53LX_MultiRangingData_t MultiRangingData;
   VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int no_of_object_found = 0, j;
   char report[64];
   int status;

   do
   {
      status = sensor_vl53lx2_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53lx2_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53LX Satellite #2: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      SerialPort.print(report);
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                               ");
         SerialPort.print("status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
      }
      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53lx2_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }

     

      status = sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53LX Satellite #1: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      SerialPort.print(report);
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                               ");
         SerialPort.print("status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
      }
      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }




      status = sensor_vl53lx3_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53LX Satellite #3: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      SerialPort.print(report);
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                               ");
         SerialPort.print("status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
      }
      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53lx3_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }
      
   }

   digitalWrite(LedPin, LOW);

   
   
}
