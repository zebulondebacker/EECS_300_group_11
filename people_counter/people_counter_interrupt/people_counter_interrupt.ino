//On some boards like the Arduino Uno the pin used by the sensor to raise interrupts (A2)
//can't be mapped as an interrupt pin. For this this reason this sketch will not work
//unless some additional cabling is done and the interrupt pin is changed.
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

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define LedPin LED_BUILTIN

#define SDA_pin 21
#define SCL_pin 22
#define XSHUT_pin 18

#define interruptPin 19

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, XSHUT_pin);

volatile int interruptCount=0;

void measure()
{
   interruptCount=1;
}

void setup()
{
   VL53LX_Error status;
   // Led.
   pinMode(LedPin, OUTPUT);
   pinMode(interruptPin, INPUT_PULLUP);
   attachInterrupt(interruptPin, measure, FALLING);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin(SDA_pin,SCL_pin);

   // Configure VL53LX satellite component.
   sensor_vl53lx_sat.begin();

   // Switch off VL53LX satellite component.
   sensor_vl53lx_sat.VL53LX_Off();

   // Initialize VL53LX satellite component.
   status = sensor_vl53lx_sat.InitSensor(0x12);
   if(status)
   {
      SerialPort.println("Init sensor_vl53lx_sat failed...");
   }

   sensor_vl53lx_sat.VL53LX_StartMeasurement();
}

void loop()
{
   VL53LX_MultiRangingData_t MultiRangingData;
   VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int no_of_object_found = 0, j;
   char report[64];
   if (interruptCount)
   {
      int status;

      interruptCount=0;
      // Led blinking.
      digitalWrite(LedPin, HIGH);

      status = sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);
      if((!status)&&(NewDataReady!=0))
      {
         status = sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
         no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
         snprintf(report, sizeof(report), "Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
         SerialPort.print(report);
         for(j=0;j<no_of_object_found;j++)
         {
            if(j!=0)SerialPort.print("\r\n                   ");
            SerialPort.print("status=");
            SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print(", D=");
            SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
            SerialPort.print("mm");
            SerialPort.print(", Signal=");
            SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0);
            SerialPort.print(" Mcps, Ambient=");
            SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
            SerialPort.print(" Mcps");
         }
         SerialPort.println("");
         if (status==0)
         {
            status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
         }
      }

      digitalWrite(LedPin, LOW);
   }
}
