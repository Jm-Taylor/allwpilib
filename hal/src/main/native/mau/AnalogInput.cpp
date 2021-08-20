/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/AnalogInput.h"

#include "AnalogInternal.h"
#include "hal/handles/HandlesInternal.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauInternal.h"
#include <VMXIO.h>

using namespace hal;

namespace hal {
    namespace init {
        void InitializeAnalogInput() {}
    }
}

extern "C" {
    HAL_AnalogInputHandle HAL_InitializeAnalogInputPort(HAL_PortHandle portHandle, int32_t* status) {
        hal::init::CheckInit();
        int16_t wpi_analogin_channel = getPortHandleChannel(portHandle);
        if (wpi_analogin_channel == InvalidHandleIndex) {
            *status = PARAMETER_OUT_OF_RANGE;
            return HAL_kInvalidHandle;
        }

        HAL_AnalogInputHandle handle;
        auto port = allocateAnalogInputHandleAndInitializedPort(wpi_analogin_channel, handle, status);
        if (port == nullptr) {
            return HAL_kInvalidHandle;
        }

        port->vmx_config = AccumulatorConfig();
        // Clear all capabilities except for Accumulator Input
        if ((port->vmx_chan_info.capabilities & VMXChannelCapability::AccumulatorInput) == 0) {
        	// Error:  This VMX Channel doesn't have AccumulatorInput Capability.
        	return HAL_kInvalidHandle;
        }
        port->vmx_chan_info.capabilities = VMXChannelCapability::AccumulatorInput;
        if (!AllocateVMXAnalogIn(port, status)) {
        	analogInputHandles->Free(handle);
        	return HAL_kInvalidHandle;
        }
        port->channel = wpi_analogin_channel;

        return handle;
    }

    void HAL_FreeAnalogInputPort(HAL_AnalogInputHandle analogPortHandle) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port != nullptr) {

            bool active;
            DeallocateVMXAnalogIn(port, active);

        	analogInputHandles->Free(analogPortHandle);
        }
    }

    HAL_Bool HAL_CheckAnalogModule(int32_t module) { return module == 1; }

    HAL_Bool HAL_CheckAnalogInputChannel(int32_t channel) {
   		return isWPILibChannelValid(HAL_ChannelAddressDomain::AnalogInput, channel);
    }

    void HAL_SetAnalogInputSimDevice(HAL_AnalogInputHandle handle,
                                     HAL_SimDeviceHandle device) {}

    /**
     * Set the sample rate per channel for all analog channels.
     *
     * The maximum rate is 500kS/s divided by the number of channels in use.
     * This is 62500 samples/s per channel.
     *
     * @param samplesPerSecond The number of samples per second.
     */
    void HAL_SetAnalogSampleRate(double samplesPerSecond, int32_t* status) {
        // No op; the VMX-pi sample rate is fixed currently, as is the reference HAL implementation.
    }

    double HAL_GetAnalogSampleRate(int32_t* status) {
        return kDefaultSampleRate;
    }

    void HAL_SetAnalogAverageBits(HAL_AnalogInputHandle analogPortHandle, int32_t bits, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        bool active;
        DeallocateVMXAnalogIn(port, active);
        port->vmx_config.SetNumAverageBits(uint8_t(bits));
        AllocateVMXAnalogIn(port,status);
    }

    int32_t HAL_GetAnalogAverageBits(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return 0;
        }

        return static_cast<int32_t>(port->vmx_config.GetNumAverageBits());
    }

    void HAL_SetAnalogOversampleBits(HAL_AnalogInputHandle analogPortHandle, int32_t bits, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        bool active;
        DeallocateVMXAnalogIn(port, active);
        port->vmx_config.SetNumOversampleBits(uint8_t(bits));
        AllocateVMXAnalogIn(port,status);
    }

    int32_t HAL_GetAnalogOversampleBits(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return 0;
        }

        return static_cast<int32_t>(port->vmx_config.GetNumOversampleBits());
    }

    int32_t HAL_GetAnalogValue(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return 0;
        }
        uint32_t instantaneousValue;
    	mau::vmxIO->Accumulator_GetInstantaneousValue(port->vmx_res_handle, instantaneousValue, status);
        instantaneousValue = mau::DefaultOnBoardCommError(status, instantaneousValue, port->last_adc_counts);

        return static_cast<int32_t>(instantaneousValue);
    }

    /* Returns value in ADC counts after Oversample/Average engine output */
    int32_t HAL_GetAnalogAverageValue(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return 0;
        }
        uint32_t averageValue;
    	mau::vmxIO->Accumulator_GetAverageValue(port->vmx_res_handle, averageValue, status);
        averageValue = mau::DefaultOnBoardCommError(status, averageValue, port->last_adc_counts);	
        
        return static_cast<int32_t>(averageValue);
    }

    int32_t HAL_GetAnalogVoltsToValue(HAL_AnalogInputHandle analogPortHandle, double voltage, int32_t* status) {
        if (voltage > 5.0) {
            voltage = 5.0;
            *status = VOLTAGE_OUT_OF_RANGE;
        }
        if (voltage < 0.0) {
            voltage = 0.0;
            *status = VOLTAGE_OUT_OF_RANGE;
        }
        int32_t LSBWeight = HAL_GetAnalogLSBWeight(analogPortHandle, status);
        int32_t offset = HAL_GetAnalogOffset(analogPortHandle, status);
        int32_t value =
                static_cast<int32_t>((voltage + offset * 1.0e-9) / (LSBWeight * 1.0e-9));
        return value;
    }

    double HAL_GetAnalogVoltage(HAL_AnalogInputHandle analogPortHandle,
                                int32_t* status) {
        return HAL_GetAnalogAverageVoltage(analogPortHandle,status);
    }

    double HAL_GetAnalogAverageVoltage(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return 0.0;
        }
        
        float averageVoltage;
    	mau::vmxIO->Accumulator_GetAverageVoltage(port->vmx_res_handle, averageVoltage, status);
        averageVoltage = mau::DefaultOnBoardCommError(status, averageVoltage, port->last_adc_voltage);        
        
        return static_cast<double>(averageVoltage);        
    }

    int32_t HAL_GetAnalogLSBWeight(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
		float full_scale_voltage = 5.0;
		constexpr int32_t kNanovoltsPerVolt = 1000000000;
        mau::vmxIO->Accumulator_GetFullScaleVoltage(full_scale_voltage, status);
        mau::ClearBoardCommErrorStatus(status);
		double full_scale_nanovolts = static_cast<double>(full_scale_voltage) * kNanovoltsPerVolt;
		return static_cast<int32_t>(full_scale_nanovolts);
    }

    /* Returns the amount of ADC offset voltage, in units of nanovolts */
    int32_t HAL_GetAnalogOffset(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
    	// TODO:  This is not currently supported by VMX-pi.  Once it is, update
    	// this HAL API Function to return a factory-calibrated offset value.
        return 0;
    }
}
