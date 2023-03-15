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
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
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
 * pin 1 (Interrupt) of the VL53L3CX satellite connected to pin 19 of the ESP32 board 
 * pin 2 (SCL_I) of the VL53L3CX satellite connected to pin 23 (SCL) of the ESP32 board with a Pull-Up resistor of 4.7 KOhm
 * pin 3 (XSDN_I) of the VL53L3CX satellite connected to pin A1 of the ESP32 board
 * pin 4 (SDA_I) of the VL53L3CX satellite connected to pin 22 (SDA) of the ESP32 board with a Pull-Up resistor of 4.7 KOhm
 * pin 5 (VDD) of the VL53L3CX satellite connected to 3V3 pin of the ESP32 board
 * pin 6 (GND) of the VL53L3CX satellite connected to GND of the ESP32 board
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
#include <math.h>

#define DEV_I2C Wire
#define SerialPort Serial
#define SDA_pin 21
#define SCL_pin 22
#define XSHUT_pin 18
#define GPIO_pin 19 
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define LedPin LED_BUILTIN

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, XSHUT_pin);


/* Setup ---------------------------------------------------------------------*/
float curTime = 0;
float previousTime = 0;
float velocity = 0;
double curPosition = 0;
double prevPosition = 0;
double avgVelocity = 0;
int counter = 0;
int peopleCounter = 0;

float arrNumbers[5] = {0};
int arrobjects[5] = {0};
bool no_object_detected;

int sum_of_objects = 0;
int pos = 0;
int pos_object = 0;
float detectForObject = 0;
float newAvg = 0;
float sum = 0;
float len = 5.0 ;
  
void setup()
{
   // Led.
   pinMode(LedPin, OUTPUT);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin(SDA_pin,SCL_pin); // Pin 21 to SDA, Pin 22 to SCL

   // Configure VL53LX satellite component.
   sensor_vl53lx_sat.begin();

   // Switch off VL53LX satellite component.
   sensor_vl53lx_sat.VL53LX_Off();

   //Initialize VL53LX satellite component.
   sensor_vl53lx_sat.InitSensor(0x12);

  //Set the distance mode. Options are short, medium, and long.
   sensor_vl53lx_sat.VL53LX_SetDistanceMode(VL53LX_DISTANCEMODE_MEDIUM);
   
   // Start Measurements
   sensor_vl53lx_sat.VL53LX_StartMeasurement();
}

void loop()
{
   VL53LX_MultiRangingData_t MultiRangingData;
   VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int no_of_object_found = 0, j;
   char report[64];
   int status;

   do
   {
      status = sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53LX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      //SerialPort.print(report);
      
      no_object_detected = detect_no_objects(arrobjects, &sum_of_objects, pos_object, len, no_of_object_found));
      pos_object++;
      if (pos_object >= len){
        pos_object = 0;
      }
      
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                               ");
         if(pMultiRangingData->RangeData[j].RangeStatus == 0 /*|| pMultiRangingData->RangeData[j].RangeStatus == 7*/){
//         SerialPort.print("status=");
//         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
//         SerialPort.print(", D=");
//         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
//         SerialPort.print("mm");

       
        // Calculate velocity
         previousTime = curTime;
         curTime = millis();
         float interval = curTime - previousTime;
         curPosition = pMultiRangingData->RangeData[j].RangeMilliMeter;
         
         //Check if people walked though the door
         peopleCounter = peopleCounter + people(curPosition, prevPosition, newAvg);
         //SerialPort.print(peopleCounter);
          
         velocity = (curPosition - prevPosition)/ interval;
         if(isinf(velocity)){
          velocity = 0;
         }
         prevPosition = curPosition;
         newAvg = movingAvg(arrNumbers, &sum, pos, len, velocity);
         pos++;
         if (pos >= len){
          pos = 0;
         }

         SerialPort.println("");
         }

      
      }
      
      if (status==0)
      {
         status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }
   }

   digitalWrite(LedPin, LOW);
} // End of void loop funtion



// FUNCTION LIST----------------------------------------------------------------------
//
// Average velocity function

float movingAvg(float *ptrArrNumbers, float *ptrSum, int pos, float len, float nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //SerialPort.print(*ptrSum);
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  //SerialPort.print(*ptrSum / len);
  return *ptrSum / len;
}

// Function to return true if sensor is no longer seeing objects based on
// last 5 readings for no_of_object_found

bool detect_no_objects(int *ptrArrNumbers, int *ptrSum, int pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  ptrArrNumbers[pos] = nextNum;
  return (*ptrSum < 1) ;
}

// People counting function: determines when to consider if a person walked through the door based in sensor tracking a new object

int people(double currentPosition, double previousPosition, float averageVel,) {
  if(abs(currentPosition - previousPosition) > 300) {
    if(averageVel > 0.5)
    {
      return -1;
    }
    else if(averageVel < -0.5)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  return 0;
}
