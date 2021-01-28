/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/SerialPort.h"
#include "OSSerialPort.h"

#include "HALInitializer.h"

namespace hal {
    namespace init {
        void InitializeSerialPort() {}
    }
}

// All VMX-pi serial ports are accessible directly from Linux; therefore all of the HAL Serial
// Port calls map one-to-on onto the HAL_OSSerialPort interface.

extern "C" {
    HAL_SerialPortHandle HAL_InitializeSerialPort(HAL_SerialPort port, int32_t* status) {
        hal::init::CheckInit();
        return HAL_InitializeOSSerialPort(port, status);
    }

    HAL_SerialPortHandle HAL_InitializeSerialPortDirect(HAL_SerialPort port, const char* portName, int32_t* status) {
    	return HAL_InitializeSerialPort(port, status);
    }

    void HAL_SetSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud, int32_t* status) {
    	HAL_SetOSSerialBaudRate(handle, baud, status);
    }

    void HAL_SetSerialDataBits(HAL_SerialPortHandle handle, int32_t bits, int32_t* status) {
    	HAL_SetOSSerialDataBits(handle, bits, status);
    }

    void HAL_SetSerialParity(HAL_SerialPortHandle handle, int32_t parity, int32_t* status) {
    	HAL_SetOSSerialParity(handle, parity, status);
    }

    void HAL_SetSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits, int32_t* status) {
    	HAL_SetOSSerialStopBits(handle, stopBits, status);
    }

    void HAL_SetSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode, int32_t* status) {
    	HAL_SetOSSerialWriteMode(handle, mode, status);
    }

    void HAL_SetSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow, int32_t* status) {
    	HAL_SetOSSerialFlowControl(handle, flow, status);
    }

    void HAL_SetSerialTimeout(HAL_SerialPortHandle handle, double timeout, int32_t* status) {
    	HAL_SetOSSerialTimeout(handle, timeout, status);
    }

    void HAL_EnableSerialTermination(HAL_SerialPortHandle handle, char terminator, int32_t* status) {
    	HAL_EnableOSSerialTermination(handle, terminator, status);
    }

    void HAL_DisableSerialTermination(HAL_SerialPortHandle handle, int32_t* status) {
    	HAL_DisableOSSerialTermination(handle, status);
    }

    void HAL_SetSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t* status) {
    	HAL_SetOSSerialReadBufferSize(handle, size, status);
    }

    void HAL_SetSerialWriteBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t* status) {
    	HAL_SetOSSerialWriteBufferSize(handle, size, status);
    }

    int32_t HAL_GetSerialBytesReceived(HAL_SerialPortHandle handle, int32_t* status) {
        return HAL_GetOSSerialBytesReceived(handle, status);
    }

    int32_t HAL_ReadSerial(HAL_SerialPortHandle handle, char* buffer, int32_t count, int32_t* status) {
        return HAL_ReadOSSerial(handle, buffer, count, status);
    }

    int32_t HAL_WriteSerial(HAL_SerialPortHandle handle, const char* buffer, int32_t count, int32_t* status) {
        return HAL_WriteOSSerial(handle, buffer, count, status);
    }

    void HAL_FlushSerial(HAL_SerialPortHandle handle, int32_t* status) {
    	HAL_FlushOSSerial(handle, status);
    }

    void HAL_ClearSerial(HAL_SerialPortHandle handle, int32_t* status) {
    	HAL_ClearOSSerial(handle, status);
    }

    void HAL_CloseSerial(HAL_SerialPortHandle handle, int32_t* status) {
    	HAL_CloseOSSerial(handle, status);
    }
    int HAL_GetSerialFD(HAL_SerialPortHandle handle, int32_t* status) {
        return HAL_GetOSSerialFD(handle, status);
    }
}
