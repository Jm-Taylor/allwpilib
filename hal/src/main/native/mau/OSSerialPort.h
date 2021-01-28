/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include "hal/SerialPort.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_SerialPortHandle HAL_InitializeOSSerialPort(HAL_SerialPort port, int32_t* status);
void HAL_SetOSSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud,
                             int32_t* status);
void HAL_SetOSSerialDataBits(HAL_SerialPortHandle handle, int32_t bits,
                             int32_t* status);
void HAL_SetOSSerialParity(HAL_SerialPortHandle handle, int32_t parity,
                           int32_t* status);
void HAL_SetOSSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits,
                             int32_t* status);
void HAL_SetOSSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode,
                              int32_t* status);
void HAL_SetOSSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow,
                                int32_t* status);
void HAL_SetOSSerialTimeout(HAL_SerialPortHandle handle, double timeout,
                            int32_t* status);
void HAL_EnableOSSerialTermination(HAL_SerialPortHandle handle, char terminator,
                                   int32_t* status);
void HAL_DisableOSSerialTermination(HAL_SerialPortHandle handle, int32_t* status);
void HAL_SetOSSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size,
                                   int32_t* status);
void HAL_SetOSSerialWriteBufferSize(HAL_SerialPortHandle handlet, int32_t size,
                                    int32_t* status);
int32_t HAL_GetOSSerialBytesReceived(HAL_SerialPortHandle handle, int32_t* status);
int32_t HAL_ReadOSSerial(HAL_SerialPortHandle handle, char* buffer, int32_t count,
                         int32_t* status);
int32_t HAL_WriteOSSerial(HAL_SerialPortHandle handle, const char* buffer,
                          int32_t count, int32_t* status);
void HAL_FlushOSSerial(HAL_SerialPortHandle handle, int32_t* status);
void HAL_ClearOSSerial(HAL_SerialPortHandle handle, int32_t* status);
void HAL_CloseOSSerial(HAL_SerialPortHandle handle, int32_t* status);

int HAL_GetOSSerialFD(HAL_SerialPortHandle handle, int32_t* status);
#ifdef __cplusplus
}  // extern "C"
#endif
