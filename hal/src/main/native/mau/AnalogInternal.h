/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include <memory>

#include "hal/Ports.h"
#include "hal/handles/IndexedHandleResource.h"
#include "hal/handles/HandlesInternal.h"
#include "PortsInternal.h"
#include <VMXPiConstants.h>
#include <VMXResource.h>
#include <VMXResourceConfig.h>

namespace hal {
    constexpr int32_t kTimebase = 40000000;  ///< 40 MHz clock
    constexpr int32_t kDefaultOversampleBits = 0;
    constexpr int32_t kDefaultAverageBits = 7;
    constexpr double kDefaultSampleRate = double(VMXPI_ADC_SAMPLE_RATE_HZ);
    static constexpr int32_t kAccumulatorChannels[] = {0, 1};

    struct AnalogPort {
    	VMXChannelInfo vmx_chan_info;
    	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::Undefined,INVALID_VMX_RESOURCE_INDEX);
    	AccumulatorConfig vmx_config;
        int32_t channel;		/* WPI Library Channel Number (in Analog Channel Address Domain) */
	uint32_t last_adc_counts = 0;
	float last_adc_voltage = 0.0;
    };

    extern IndexedHandleResource<HAL_AnalogInputHandle, hal::AnalogPort,
            kNumAnalogInputs, HAL_HandleEnum::AnalogInput> *
            analogInputHandles;

    int32_t GetAnalogTriggerInputIndex(HAL_AnalogTriggerHandle handle, int32_t *status);
    std::shared_ptr<AnalogPort> allocateAnalogInputHandleAndInitializedPort(int32_t wpi_channel, HAL_AnalogInputHandle& anInHandle, int32_t *status);
    bool AllocateVMXAnalogIn(std::shared_ptr<AnalogPort> anPort, int32_t* status);
    void DeallocateVMXAnalogIn(std::shared_ptr<AnalogPort> anPort, bool& isActive);
}
