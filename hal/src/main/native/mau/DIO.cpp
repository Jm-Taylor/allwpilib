/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/DIO.h"
#include "hal/PWM.h"

#include <cmath>

#include "DigitalInternal.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauInternal.h"
#include "DigitalFilterShared.h"
#include "PWMShared.h"
#include <VMXIO.h>
#include <VMXResource.h>

using namespace hal;
using namespace mau;

struct DigitalPWMGenerator {
	PWMGeneratorConfig config;						// Current/pending configuration of DIO in PWMGenerator Mode
	bool active = false;							// true if PWMGenerator is currently active, output channel is assigned
	HAL_DigitalHandle assigned_output_channel = HAL_kInvalidHandle;
	bool output_channel_previously_active = false;	// true if the digital handle was allocated before it was assigned to this pwm generator
	bool output_channel_input = false;				// stores state of digital output channel before assigned to ths pwm generator
};

constexpr double kDefPwmGeneratorRate = 200.0;
static double digPwmGeneratorRate = kDefPwmGeneratorRate;

static LimitedHandleResource<HAL_DigitalPWMHandle, DigitalPWMGenerator, kNumDigitalPWMOutputs, HAL_HandleEnum::DigitalPWM>*
        digitalPWMHandles;

uint64_t digFilterPeriodNanoseconds[kNumDigitalFilters] = {0,0,0,0};

constexpr uint64_t kNumNanosecondsPerFPGACycle = 100;

void HAL_Internal_SetDigitalFilterPeriodNanoseconds(uint8_t digFilterIndex, uint64_t period_nanoseconds)
{
	if (digFilterIndex < kNumDigitalFilters) {
		digFilterPeriodNanoseconds[digFilterIndex] = period_nanoseconds;
	}
}

uint64_t HAL_Internal_GetDigitalFilterPeriodNanoseconds(uint8_t digFilterIndex)
{
	if (digFilterIndex < kNumDigitalFilters) {
		return digFilterPeriodNanoseconds[digFilterIndex];
	}
	return 0;
}

namespace hal {
    namespace init {
        void InitializeDIO() {
            static LimitedHandleResource<HAL_DigitalPWMHandle, DigitalPWMGenerator, kNumDigitalPWMOutputs, HAL_HandleEnum::DigitalPWM> dpH;
            digitalPWMHandles = &dpH;
        }
    }
}

static bool AllocateVMXPiDIO(std::shared_ptr<DigitalPort> digPort, bool input, int32_t* status) {
	VMXChannelInfo vmx_chan_info = digPort->vmx_chan_info;
    /* Determine VMX Channel DIO Capabilities appropriate for allocation request */
	if (input) {
		if (mau::vmxIO->ChannelSupportsCapability(digPort->vmx_chan_info.index, VMXChannelCapability::DigitalInput)) {
			digPort->dio_config.SetInput(true);
			digPort->dio_config.SetInputMode(DIOConfig::InputMode::PULLUP);
			vmx_chan_info.capabilities = VMXChannelCapability::DigitalInput;
		} else {
			*status = VMXERR_IO_INVALID_CHANNEL_TYPE;
			return false;
		}
	} else {
		if (mau::vmxIO->ChannelSupportsCapability(digPort->vmx_chan_info.index, VMXChannelCapability::DigitalOutput)) {
			digPort->dio_config.SetInput(false);
			digPort->dio_config.SetOutputMode(DIOConfig::OutputMode::PUSHPULL);
			vmx_chan_info.capabilities = VMXChannelCapability::DigitalOutput;
		} else {
			*status = VMXERR_IO_INVALID_CHANNEL_TYPE;
			return false;
		}
	}

	if (!mau::vmxIO->ActivateSinglechannelResource(vmx_chan_info, &digPort->dio_config, digPort->vmx_res_handle, status)) {
		return false;
	}

	return true;
}

/* Can be used to deallocate either a DigitalIO or a PWMGenerator VMXPi Resource. */
static void DeallocateVMXPiDIO(std::shared_ptr<DigitalPort> digPort, bool& isActive) {
	isActive = false;
	mau::vmxIO->IsResourceActive(digPort->vmx_res_handle, isActive, mau::vmxError);
	if (isActive) {
		mau::vmxIO->DeallocateResource(digPort->vmx_res_handle, mau::vmxError);
	}
	digPort->vmx_res_handle = 0;
}

extern "C" {
    /**
     * Create a new instance of a digital port.
     */
    HAL_DigitalHandle HAL_InitializeDIOPort(HAL_PortHandle portHandle, HAL_Bool input, int32_t* status) {
        hal::init::CheckInit();
        if (*status != 0) return HAL_kInvalidHandle;

        int16_t wpi_digital_channel = getPortHandleChannel(portHandle);
        if (wpi_digital_channel == InvalidHandleIndex) {
            *status = PARAMETER_OUT_OF_RANGE;
            return HAL_kInvalidHandle;
        }

        HAL_DigitalHandle digHandle;
        auto port = allocateDigitalHandleAndInitializedPort(HAL_HandleEnum::DIO, wpi_digital_channel, digHandle, status);
        if (port == nullptr) {
            return HAL_kInvalidHandle;
        }

        /* Set configuration to defaults */
        port->dio_config = DIOConfig();
        if (!AllocateVMXPiDIO(port, input ? true : false, status)) {
            digitalChannelHandles->Free(digHandle, HAL_HandleEnum::DIO);
        	return HAL_kInvalidHandle;
        }

        return digHandle;
    }

    HAL_Bool HAL_CheckDIOChannel(int32_t channel) {
		return isWPILibChannelValid(HAL_ChannelAddressDomain::DIO, channel);
    }

    void HAL_FreeDIOPort(HAL_DigitalHandle dioPortHandle) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) return;

        bool active;
        DeallocateVMXPiDIO(port, active);

        // no status, so no need to check for a proper free.
        digitalChannelHandles->Free(dioPortHandle, HAL_HandleEnum::DIO);
    }

    void HAL_SetDIOSimDevice(HAL_DigitalHandle handle, HAL_SimDeviceHandle device) {
    }

    /**
     * Allocate a DO PWM Generator.
     * Allocate PWM generators so that they are not accidentally reused.
     *
     * @return PWM Generator handle
     */
    HAL_DigitalPWMHandle HAL_AllocateDigitalPWM(int32_t* status) {
        auto handle = digitalPWMHandles->Allocate();
        if (handle == HAL_kInvalidHandle) {
            *status = NO_AVAILABLE_RESOURCES;
            return HAL_kInvalidHandle;
        }

        auto digPwmGen = digitalPWMHandles->Get(handle);
        if (digPwmGen == nullptr) {  // would only occur on thread issue.
            *status = HAL_HANDLE_ERROR;
            return HAL_kInvalidHandle;
        }

        // Mau Note:  Each DIO channel with "output capability" may
        // have the corresponding VMX-pi "PWMGen" resource allocated
        // for it.  This allocation does not occur until later; at that
        // time, the corresponding DigitalIO resource will be released,
        // and replaced with an allocated PWMGen resource instead.

        return handle;
    }

    /**
     * Free the resource associated with a DO PWM generator.
     *
     * @param pwmGenerator The pwmGen to free that was allocated with
     * allocateDigitalPWM()
     */
    void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator, int32_t* status) {

        auto digPwmGen = digitalPWMHandles->Get(pwmGenerator);
        if (digPwmGen == nullptr) {
        	return;
        }

        if (digPwmGen->active) {
            auto port = digitalChannelHandles->Get(digPwmGen->assigned_output_channel, HAL_HandleEnum::DIO);
            if (port == nullptr) return;
            bool active;
            DeallocateVMXPiDIO(port, active);
            digPwmGen->active = false;

            // Restore previous digital channel allocation
            if (digPwmGen->output_channel_previously_active) {
            	AllocateVMXPiDIO(port, digPwmGen->output_channel_input, status);
            }
        }

    	digitalPWMHandles->Free(pwmGenerator);
   }

    /**
     * Change the frequency of the DO PWM generator.
     *
     * The valid range is from 0.6 Hz to 19 kHz.  The frequency resolution is
     * logarithmic (this sentence applies to athena, not Mau).
     *
     * @param rate The frequency to output all digital output PWM signals.
     */
    void HAL_SetDigitalPWMRate(double rate, int32_t* status) {
    	digPwmGeneratorRate = rate;
    }

    /**
     * Configure the duty-cycle of the PWM generator
     *
     * @param pwmGenerator The generator index reserved by allocateDigitalPWM()
     * @param dutyCycle The percent duty cycle to output [0..1].
     */
    void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle, int32_t* status) {
        auto digPwmGen = digitalPWMHandles->Get(pwmGenerator);
        if (digPwmGen == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        if (dutyCycle > 1.0) dutyCycle = 1.0;
        if (dutyCycle < 0.0) dutyCycle = 0.0;

        if (digPwmGen->active) {
            auto port = digitalChannelHandles->Get(digPwmGen->assigned_output_channel, HAL_HandleEnum::DIO);
            if (port == nullptr) return;
            HAL_SetPWMSpeed(digPwmGen->assigned_output_channel, dutyCycle, status);
        } else {
        	*status = INCOMPATIBLE_STATE;
        }
    }

    /**
     * Configure which DO channel the PWM signal is output on
     *
     * @param pwmGenerator The generator index reserved by allocateDigitalPWM()
     * @param channel The Digital Output channel to output on
     */
    void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator, int32_t channel, int32_t* status) {
        auto pwmGenPort = digitalPWMHandles->Get(pwmGenerator);
        if (pwmGenPort == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        VMXChannelInfo vmx_chan_info;
        HAL_DigitalHandle dioPortHandle =
        		getDigitalHandleAndVMXChannelInfo(HAL_HandleEnum::DIO, channel, vmx_chan_info, status);
        if (dioPortHandle == HAL_kInvalidHandle) {
        	return;
        }

        auto dioPort = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (dioPort == nullptr) return;
        bool input = (HAL_GetDIODirection(dioPortHandle, status) ? false : true);
        // Disassociate the digital IO handle from any DIO resource
        bool active;
        DeallocateVMXPiDIO(dioPort, active);

        if (!HAL_Internal_ActivatePWMGenerator(dioPortHandle, status)) {
        	return;
        }
        pwmGenPort->active = true;
        pwmGenPort->assigned_output_channel = dioPortHandle;
        pwmGenPort->output_channel_previously_active = active;
        pwmGenPort->output_channel_input = input;
    }

    /**
     * Write a digital I/O bit to VMX-pi.
     * Set a single value on a digital I/O channel.
     *
     * @param channel The Digital I/O channel
     * @param value The state to set the digital channel (if it is configured as an
     * output)
     */
    void HAL_SetDIO(HAL_DigitalHandle dioPortHandle, HAL_Bool value, int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        if (value != 0 && value != 1) {
            if (value != 0) value = 1;
        }

        if (EXTRACT_VMX_RESOURCE_TYPE(port->vmx_res_handle) == VMXResourceType::PWMGenerator) {
        	*status = MAU_DIO_ASSIGNED_TO_PWMGEN;
        	return;
        }

        uint8_t retry_count = 3;
        do { 
                mau::vmxIO->DIO_Set(port->vmx_res_handle, value ? true : false, status);
        } while (RetryWriteOnBoardCommError(status, retry_count));         
    }

    /**
     * Set (or change) direction of a DIO channel.
     *
     * @param channel The Digital I/O channel
     * @param input true to set input, false for output
     */
    void HAL_SetDIODirection(HAL_DigitalHandle dioPortHandle, HAL_Bool input, int32_t* status) {

		// NOTE:  This HAL API function not appear to be invoked from anywhere in wpilibc.

        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        if (EXTRACT_VMX_RESOURCE_TYPE(port->vmx_res_handle) == VMXResourceType::PWMGenerator) {
        	*status = MAU_DIO_ASSIGNED_TO_PWMGEN;
        	return;
        }

    	/* Only take action iIf direction already matches the current channel configuration */
    	if (port->dio_config.GetInput() == (input ? true : false)) {
    		return;
    	}

    	/* Determine if channel supports the requested direction */
		if (input) {
			if (!(port->vmx_chan_info.capabilities & VMXChannelCapability::DigitalInput)) {
				*status = VMXERR_IO_INVALID_CHANNEL_TYPE;
				return;
			}
		} else {
			if (!(port->vmx_chan_info.capabilities & VMXChannelCapability::DigitalOutput)) {
				*status = VMXERR_IO_INVALID_CHANNEL_TYPE;
				return;
			}
		}

		/* Deallocate the channel if it is already allocated. */
        bool active;
        DeallocateVMXPiDIO(port, active);

        AllocateVMXPiDIO(port, input ? true : false, status);
    }

    /**
     * Read a digital I/O bit from the FPGA.
     * Get a single value from a digital I/O channel.
     *
     * @param channel The digital I/O channel
     * @return The state of the specified channel
     */
    HAL_Bool HAL_GetDIO(HAL_DigitalHandle dioPortHandle, int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }

        if (EXTRACT_VMX_RESOURCE_TYPE(port->vmx_res_handle) == VMXResourceType::PWMGenerator) {
        	*status = MAU_DIO_ASSIGNED_TO_PWMGEN;
        	return false;
        }

        bool high = true;
        mau::vmxIO->DIO_Get(port->vmx_res_handle, high, status);
        ClearBoardCommErrorStatus(status);
        return high;
    }

    /**
     * Read the direction of a the Digital I/O lines
     * A 1 bit means output and a 0 bit means input.
     *
     * @param channel The digital I/O channel
     * @return The direction of the specified channel
     */
    HAL_Bool HAL_GetDIODirection(HAL_DigitalHandle dioPortHandle, int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }

        if (EXTRACT_VMX_RESOURCE_TYPE(port->vmx_res_handle) == VMXResourceType::PWMGenerator) {
        	*status = MAU_DIO_ASSIGNED_TO_PWMGEN;
        	return false;
        }

		return (port->dio_config.GetInput() ? 0 : 1);
    }

    /**
     * Generate a single pulse.
     * Write a pulse to the specified digital output channel. There can only be a
     * single pulse going at any time.
     *
     * @param channel The Digital Output channel that the pulse should be output on
     * @param pulseLength The active length of the pulse (in seconds)
     */
	// NOTE:  This does not appear to be invoked from anywhere in wpilibc.
    void HAL_Pulse(HAL_DigitalHandle dioPortHandle, double pulseLength,
                   int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        if (EXTRACT_VMX_RESOURCE_TYPE(port->vmx_res_handle) == VMXResourceType::PWMGenerator) {
        	*status = MAU_DIO_ASSIGNED_TO_PWMGEN;
        	return;
        }

    	/* We assume here that all pulses are "active high", since interestingly */
    	/* the active state is not provided in the parameters to this function. */

        uint8_t retry_count = 3;
        uint32_t num_microseconds = static_cast<uint32_t>(pulseLength / 1.0e-6);
        do { 
                mau::vmxIO->DIO_Pulse(port->vmx_res_handle, true /*high*/, num_microseconds, status);
        } while (RetryWriteOnBoardCommError(status, retry_count));         
        return;
    }

    /**
     * Check a DIO line to see if it is currently generating a pulse.
     *
     * @return A pulse is in progress
     */
	// NOTE:  This does not appear to be invoked from anywhere in wpilibc.
    HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }

        if (EXTRACT_VMX_RESOURCE_TYPE(port->vmx_res_handle) == VMXResourceType::PWMGenerator) {
        	*status = MAU_DIO_ASSIGNED_TO_PWMGEN;
        	return false;
        }

        bool is_pulsing = false;
        mau::vmxIO->DIO_IsPulsing(port->vmx_res_handle, is_pulsing, status);
        ClearBoardCommErrorStatus(status);

        return is_pulsing;
    }

    /**
     * Check if any DIO line is currently generating a pulse.
     *
     * @return A pulse on some line is in progress
     */
	// NOTE:  This does not appear to be invoked from anywhere in wpilibc.
    HAL_Bool HAL_IsAnyPulsing(int32_t* status) {
        uint8_t num_pulsing = 0;
        mau::vmxIO->DIO_GetNumPulsing(num_pulsing);
        ClearBoardCommErrorStatus(status);        
        return (num_pulsing > 0);
    }

    /**
     * Write the filter index from the FPGA.
     * Set the filter index used to filter out short pulses.
     *
     * @param dioPortHandle Handle to the digital I/O channel
     * @param filterIndex The filter index.  Must be in the range 0 - 3, where 0
     *                    means "none" and 1 - 3 means filter # filterIndex - 1.
     */
    void HAL_SetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t filterIndex, int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        if (filterIndex >= kNumDigitalFilters) {
        	*status = PARAMETER_OUT_OF_RANGE;
        	return;
        }

        port->digFilterIndex = filterIndex;

    	// TODO:  If previously configured for PWMGeneration mode, this should fail with a
    	// "DIO currently configured in PWM Generation Mode" message...

        // TODO:  If configured as a DigitalOutput-only capable channel, should this
        // return a failure status?

        // TODO:  Apply the filter to the digital channel
    }

    /**
     * Read the filter index from the FPGA.
     * Get the filter index used to filter out short pulses.
     *
     * @param dioPortHandle Handle to the digital I/O channel
     * @return filterIndex The filter index.  Must be in the range 0 - 3,
     * where 0 means "none" and 1 - 3 means filter # filterIndex - 1.
     */
    int32_t HAL_GetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t* status) {
        auto port = digitalChannelHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return 0;
        }

        return port->digFilterIndex;
    }

    /**
     * Set the filter period for the specified filter index.
     *
     * Set the filter period in FPGA cycles.  Even though there are 2 different
     * filter index domains (MXP vs HDR), ignore that distinction for now since it
     * compilicates the interface.  That can be changed later.
     *
     * @param filterIndex The filter index, 0 - 2.
     * @param value The number of cycles that the signal must not transition to be
     * counted as a transition.
     */
    void HAL_SetFilterPeriod(int32_t filterIndex, int64_t value_cycles, int32_t* status) {
    	if (value_cycles < 0) return;
    	// FPGA cycles are 100 nanoseconds
    	uint64_t num_nanoseconds = static_cast<uint64_t>(value_cycles) * kNumNanosecondsPerFPGACycle;
    	HAL_Internal_SetDigitalFilterPeriodNanoseconds(filterIndex, num_nanoseconds);
    }

    /**
     * Get the filter period for the specified filter index.
     *
     * Get the filter period in FPGA cycles.  Even though there are 2 different
     * filter index domains (MXP vs HDR), ignore that distinction for now since it
     * compilicates the interface.  Set status to NiFpga_Status_SoftwareFault if the
     * filter values miss-match.
     *
     * @param filterIndex The filter index, 0 - 2.
     * @param value The number of cycles that the signal must not transition to be
     * counted as a transition.
     */
    int64_t HAL_GetFilterPeriod(int32_t filterIndex, int32_t* status) {
    	uint64_t num_nanoseconds = HAL_Internal_GetDigitalFilterPeriodNanoseconds(filterIndex);
    	int64_t num_cycles = num_nanoseconds / kNumNanosecondsPerFPGACycle;
    	return num_cycles;
    }
}
