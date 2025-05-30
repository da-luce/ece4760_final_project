// Code structured from example code here: https://github.com/stm32duino/VL53L4CX

/**k
 ******************************************************************************
 * @file    VL53L4CX_Sat_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    16 March 2022
 * @brief   Arduino test application for the STMicrolectronics VL53L4CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
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
 * To use this sketch you need to connect the VL53L4CD satellite sensor directly to the Nucleo board with wires in this way:
 * pin 1 (GND) of the VL53L4CD satellite connected to GND of the Nucleo board
 * pin 2 (VDD) of the VL53L4CD satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (SCL) of the VL53L4CD satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 4 (SDA) of the VL53L4CD satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 5 (GPIO1) of the VL53L4CD satellite connected to pin A2 of the Nucleo board
 * pin 6 (XSHUT) of the VL53L4CD satellite connected to pin A1 of the Nucleo board
 */

/* IMPORTANT: NOTE THAT PRINTING IN THE SERIAL TERMINAL WILL ALSO SEND OVER UART!!!
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN
#define CALIB_DIST 100
#define I2C_ADDR 0x12

// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);

VL53L4CX_CalibrationData_t CalData;
VL53L4CX_Error status = VL53L4CX_ERROR_NONE;

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  // SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  //Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(I2C_ADDR);

  sensor_vl53l4cx_sat.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);

  status = sensor_vl53l4cx_sat.VL53L4CX_PerformRefSpadManagement();
  if(status != VL53L4CX_ERROR_NONE) {
		return;
	}

  status = sensor_vl53l4cx_sat.VL53L4CX_PerformXTalkCalibration();
	if(status != VL53L4CX_ERROR_NONE) {
		return;
	}

  status = sensor_vl53l4cx_sat.VL53L4CX_PerformOffsetPerVcselCalibration(CALIB_DIST);
  if(status != VL53L4CX_ERROR_NONE) {
		return;
	}

  status = sensor_vl53l4cx_sat.VL53L4CX_GetCalibrationData(&CalData);
  if(status != VL53L4CX_ERROR_NONE) {
		return;
	}

  sensor_vl53l4cx_sat.VL53L4CX_SetCalibrationData(&CalData);

  sensor_vl53l4cx_sat.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);

  // sensor_vl53l4cx_sat.VL53L4CX_SmudgeCorrectionEnable(VL53L4CX_SMUDGE_CORRECTION_CONTINUOUS);
  // if(status != VL53L4CX_ERROR_NONE) {
	// 	return;
	// }

  sensor_vl53l4cx_sat.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(32000);

  // Switch off VL53L4CX satellite component.
  // sensor_vl53l4cx_sat.VL53L4CX_Off();

  // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
}

/* Send a new line terminated int16_t */
void sendInt16(int16_t value, int32_t value2) {
  // Send each byte, LSB first (little-endian)
  Serial.write((uint8_t)value & 0xFF);
  Serial.write((uint8_t)((value >> 8) & 0xFF));
  Serial.write((uint8_t)(value2 & 0xFF));          
  Serial.write((uint8_t)((value2 >> 8) & 0xFF));   
  Serial.write((uint8_t)(value2>>16 & 0xFF));          
  Serial.write((uint8_t)((value2 >> 24) & 0xFF)); 
  Serial.write((uint8_t)'\n'); 

}

void loop()
{
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    // SerialPort.print(report);
    // for (j = 0; j < no_of_object_found; j++) {
    //   int16_t range_val = pMultiRangingData->RangeData[j].RangeMilliMeter;
    //   sendInt16(range_val);
    // }
    // If we want to include signal light measurement
    for (j = 0; j < no_of_object_found; j++) {
      int16_t range_val = pMultiRangingData->RangeData[j].RangeMilliMeter;
      int32_t signal_mcps = (int32_t)(pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0f* (1000));
      // float ambient_mcps = pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0f;
      sendInt16(range_val, signal_mcps);
    }
    // SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  digitalWrite(LedPin, LOW);
}
