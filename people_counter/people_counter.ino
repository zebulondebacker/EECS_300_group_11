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
 * pin 3 (XSDN_I) of the VL53L3CX satellite connected to pin 18 of the ESP32 board
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

#include "WirelessCommunication.h"
#include "sharedVariable.h"
#include "Preferences.h"

void init_non_vol_storage();
void update_non_vol_count();
void update_button_count();
int amountPeople(int, int, int, int);
int updatedCount(int*);

volatile int count = 0;
volatile shared_uint32 x;
Preferences nonVol;//used to store the count in nonvolatile memory

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

#define twoPersonWidth 200
#define maxCycles 150

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, XSHUT1_pin);
VL53LX sensor_vl53lx2_sat(&DEV_I2C, XSHUT2_pin);
VL53LX sensor_vl53lx3_sat(&DEV_I2C, XSHUT3_pin);

/* Setup ---------------------------------------------------------------------*/
float curTime = 0;
float previousTime = 0;
float cur_cycle_time = 0;
float prev_cycle_time = 0;
float velocity = 0;
double curPosition = 0;
double prevPosition = 0;
double avgVelocity = 0;
int counter = 0;
int peopleCounter = 0;
int peopleInDoorway = 0;
int amountExiting = 0;

float arrNumbers[10] = {0}; // Array to hold velocity readings for moving average calculations
int arrChange[10] = {0};
int prevCount = 0;
int mostRecentChange = 0;
int prevChange = 0;
int cycles = 0;

// for detecting people in doorway
int dist2 = 0;
int dist3 = 0;
int status2 = 0;
int status3 = 0;

bool no_object_detected = 0;
bool not_repeating = 0;
int sum_of_objects = 0;
int pos_object = 0;
float detectForObject = 0;

int times_without_status_0 = 0;

int pos = 0;
float newAvg = 0;
float sum = 0;
float len = 10.0 ;
float len_ob = 5.0;
void setup()
{

   // Wireless Comunications
    init_wifi_task();
    init_non_vol_count();//initializes nonvolatile memory and retrieves latest count
    count = 0;
    INIT_SHARED_VARIABLE(x, count);//init shared variable used to tranfer info to WiFi core
    
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
}

void loop()
{
//  cur_cycle_time = millis();
//  SerialPort.print(cur_cycle_time - prev_cycle_time);
//  SerialPort.println(" ms");

  prev_cycle_time = cur_cycle_time;

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

      prevCount = count;

       // if statement that runs when sensor is no longer detecting objects.
       // not_repeating variable stops the "if" function from running multiple times while sensor continuously detects no objects
      if(times_without_status_0 == 10 && not_repeating) {
        not_repeating = 0;
        int num_of_zeros_out = 0;
         //Check if people walked though the door
         for(int i=0;i<len;i++){
          if(arrNumbers[i] == 0){
            num_of_zeros_out++;
          }
        }
        if(num_of_zeros_out < 5){
         count = count + people(curPosition, prevPosition, newAvg, 1);
         update_button_count();//update shared variable x (shared with WiFi task)
         update_non_vol_count();//updates nonvolatile count
         // SerialPort.print("outside loop    ");
         // SerialPort.print(count);
        }
        
        // Sets moving average to zero while sensor does not detect objects
        for(int i=0;i<len;i++){
          newAvg = movingAvg(arrNumbers, &sum, pos, len, 0);
          pos++;
          if (pos >= len){
            pos = 0;
          }
        }
      }
        
      for(j=0;j<1;j++)
      {
         // if(j!=0)SerialPort.print("\r\n                               ");
         if(pMultiRangingData->RangeData[j].RangeStatus == 0){
      
        // Calculate velocity
         previousTime = curTime;
         curTime = millis();
         float interval = curTime - previousTime;
         curPosition = pMultiRangingData->RangeData[j].RangeMilliMeter;
         
         int num_of_zeros = 0;
         // Determine whether to increment count if sensor has atleast 6 velocity readings
         for(int i=0;i<len;i++){
          if(arrNumbers[i] == 0){
            num_of_zeros++;
          }
        }
        if(num_of_zeros < 7){
         count = count + people(curPosition, prevPosition, newAvg, no_object_detected);
         update_button_count();//update shared variable x (shared with WiFi task)
         update_non_vol_count();//updates nonvolatile count
        }
         // SerialPort.print(count);

         not_repeating = 1;
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
         
         times_without_status_0 = 0;
         // SerialPort.println("");
         }// end of if statement that only runs when object with status 0 is detected 
      }// end of for loop


      
      
//      if(count - prevCount == 2) {              // must of incremented count twice (through outside loop and inside loop)
//        count--;
//        update_button_count();//update shared variable x (shared with WiFi task)
//        update_non_vol_count();//updates nonvolatile count
//      }
//      else if(count - prevCount == -2) {        // must of decremented count twice (through outside loop and inside loop)
//        count++;
//        update_button_count();//update shared variable x (shared with WiFi task)
//        update_non_vol_count();//updates nonvolatile count
//      }


      
      /*
      for(int i = 0; i < 10; i++) {
        arrChange[i + 1] = arrChange[i];
      }
      arrChange[0] = count - prevCount;
      */
      if(count - prevCount != 0 || cycles > maxCycles) {
        if(cycles > maxCycles) {
          cycles = 0;
        }
        if(mostRecentChange == 2) {
            if(count - prevCount == -1) {
              count--;
              update_button_count();//update shared variable x (shared with WiFi task)
              update_non_vol_count();//updates nonvolatile count
              mostRecentChange = -2;
            }
            else {
              mostRecentChange = count - prevCount;
            }
        }
        else {
          mostRecentChange = count - prevCount;
        }
      }

      if (status==0)
      {
         status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }


      // At this point change status to side sensors
      status = sensor_vl53lx2_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      
      status2 = pMultiRangingData->RangeData[0].RangeStatus;
      if(status2 == 0){
        dist2 = pMultiRangingData->RangeData[0].RangeMilliMeter;
      }
      if (status==0)
      {
         status = sensor_vl53lx2_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }

      status = sensor_vl53lx3_sat.VL53LX_GetMultiRangingData(pMultiRangingData);

      
      status3 = pMultiRangingData->RangeData[0].RangeStatus;
      if(status3 == 0){
        dist3 = pMultiRangingData->RangeData[0].RangeMilliMeter;
      }
//      SerialPort.print(dist3);
//      SerialPort.println(": Sensor 3");
//      SerialPort.print(dist2);
//      SerialPort.println(": Sensor 2");

      if (status==0)
      {
         status = sensor_vl53lx3_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }      

      peopleInDoorway = amountPeople(dist2, dist3, status2, status3);
//      SerialPort.print(peopleInDoorway);
//      SerialPort.println(" people");
      // if someone entered/exited change the count depending on whether two or one person entered
      if(peopleInDoorway == 2) {
        if(mostRecentChange == 1) {
          count++;
          update_button_count();//update shared variable x (shared with WiFi task)
          update_non_vol_count();//updates nonvolatile count
          mostRecentChange = 2;
          cycles = 0;
          amountExiting = 0;
        }
        else if(mostRecentChange == 0) {
          amountExiting = 2;
        }
      }

      if(mostRecentChange == -1) {
        if(amountExiting == 2) {
          if(count >= 1) {          
            count--;
            update_button_count();//update shared variable x (shared with WiFi task)
            update_non_vol_count();//updates nonvolatile count
          }
          mostRecentChange = -2;
          cycles = 0;
          amountExiting = 0;
        }
      }
      /*
      if(updatedCount(arrChange) == 1) {            // someone entered
        if(peopleInDoorway == 2) {                  // check if two people or one person entered
          count++;
          update_button_count();//update shared variable x (shared with WiFi task)
          update_non_vol_count();//updates nonvolatile count
          arrChange[0] = 2;
        }
      }
      else if(updatedCount(arrChange) == 0) {       // front sensor did not see anyone enter
          if(peopleInDoorway == 2) {                // someone must be leaving
            amountExiting = peopleInDoorway;
          }
      }
      else if(updatedCount(arrChange) == -1) {      // someone left
        if(amountExiting == 2 && count > 0) {                    // change the count based on what the side sensors first detected (cannot go below zero)
          count--;
          update_button_count();//update shared variable x (shared with WiFi task)
          update_non_vol_count();//updates nonvolatile count
          amountExiting = 0;
          arrChange[0] = -2;
        }
      }

      if(updatedCount(arrChange) == 2 && newAvg > 0.3) {      // correct count if a person entered and a person exited at the same time
        count--;
        update_button_count();//update shared variable x (shared with WiFi task)
        update_non_vol_count();//updates nonvolatile count
        arrChange[0] = 0;
      }
      */
      cycles++;

      times_without_status_0++;
      if (times_without_status_0 > 10) {
        times_without_status_0 = 0;
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

int people(double currentPosition, double previousPosition, float averageVel, bool zero_objects_detected) {
  if(abs(currentPosition - previousPosition) > 225 || zero_objects_detected) {
    if(averageVel > 0.3 && count > 0)
    {
      // SerialPort.println(averageVel);
      return -1;
    }
    else if(averageVel < -0.3)
    {
      // SerialPort.println(averageVel);
      return 1;
    }
    else
    {
      return 0;
    }
  }
  return 0;
}

int amountPeople(int dist1, int dist2, int status1, int status2) {
  float width = dist1 + dist2;
//  SerialPort.print(width);
//  SerialPort.println(" width");
//  SerialPort.print(status1);
//  SerialPort.println(status2);
    if(width < twoPersonWidth) {
      return 2;
    }
    if(width > twoPersonWidth && width < 900) {
      return 1;
    }
    else {
      return 0;
    }
}

int updatedCount(int *ptrArrChange) {
  for(int i = 0; i < 10; i++) {
    if(ptrArrChange[i] != 0) {
      return ptrArrChange[i];      
    }
  }
  return 0;
}

// Wireless Communication Functions---------------------------------------------------------------------------------

//initializes nonvolatile memory and retrieves latest count
void init_non_vol_count()
{
  nonVol.begin("nonVolData", false);//Create a “storage space” in the flash memory called "nonVolData" in read/write mode
  count = nonVol.getUInt("count", 0);//attempts to retrieve "count" from nonVolData, sets it 0 if not found
}

//updates nonvolatile memery with lates value of count
void update_non_vol_count()
{
  nonVol.putUInt("count", count);//write count to nonvolatile memory
}

//example code that updates a shared variable (which is printed to server)
//under the hood, this implementation uses a semaphore to arbitrate access to x.value
void update_button_count()
{
  //minimized time spend holding semaphore
  LOCK_SHARED_VARIABLE(x);
  x.value = count;
  UNLOCK_SHARED_VARIABLE(x);   
}
