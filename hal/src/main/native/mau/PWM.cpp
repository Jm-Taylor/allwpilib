/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/PWM.h"

#include "ConstantsInternal.h"
#include "DigitalInternal.h"
#include "hal/handles/HandlesInternal.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauInternal.h"
#include "PWMShared.h"

#include <VMXResource.h>

using namespace hal;

namespace hal {
    namespace init {
        void InitializePWM() {}
    }
}

/* NOTE:  This function can operate on either a PWM or an ouput-capable DIO resource. */

bool HAL_Internal_ActivatePWMGenerator(HAL_DigitalHandle pwmPortHandle, int32_t *status)
{
	HAL_HandleEnum handleType = getHandleType(pwmPortHandle);
	if ((handleType != HAL_HandleEnum::PWM) && (handleType != HAL_HandleEnum::DIO)) {
		*status = HAL_HANDLE_ERROR;
		return false;
	}

    auto port = digitalChannelHandles->Get(pwmPortHandle, handleType);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return false;
    }

    VMXChannelInfo vmx_chan_info = port->vmx_chan_info;
    /* Determine VMX Channel PWM Generator Port Index assignment for allocation request */
    if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput) {
    	// Use the first port (0) on the PWM Generator resource
    	vmx_chan_info.capabilities = VMXChannelCapability::PWMGeneratorOutput;
    } else if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput2) {
    	// Use the second port (1) on the PWM Generator resource
    	vmx_chan_info.capabilities = VMXChannelCapability::PWMGeneratorOutput2;
    }
	port->vmx_chan_info.capabilities = vmx_chan_info.capabilities;

    HAL_SetPWMConfig(pwmPortHandle, 2.0, 1.501, 1.5, 1.499, 1.0, status);

    /* Set Configuration to defaults, including WPI-library compliant PWM Frequency and DutyCycle */
    port->pwmgen_config = PWMGeneratorConfig(kPwmFrequencyHz);
	port->pwmgen_config.SetMaxDutyCycleValue(kDutyCycleTicks); /* Update Duty Cycle Range to match WPI Library cycle resolution (1 us/tick) */
	if (!mau::vmxIO->ActivateSinglechannelResource(vmx_chan_info, &port->pwmgen_config, port->vmx_res_handle, status)) {
		if (*status == VMXERR_IO_NO_UNALLOCATED_COMPATIBLE_RESOURCES) {
			*status = 0;
			VMXResourceHandle resourceWithAvailablePort;
			bool allocated;
			if (mau::vmxIO->GetResourceHandleWithAvailablePortForChannel(
				PWMGenerator, port->vmx_chan_info.index, port->vmx_chan_info.capabilities, resourceWithAvailablePort, allocated, status)) {
				if (mau::vmxIO->RouteChannelToResource(port->vmx_chan_info.index, resourceWithAvailablePort, status)) {
					port->vmx_res_handle = resourceWithAvailablePort;
					return true;
				}
			}
		}
		return false;
	}
	return true;
}

extern "C" {
HAL_DigitalHandle HAL_InitializePWMPort(HAL_PortHandle portHandle, int32_t* status) {
    hal::init::CheckInit();
    if (*status != 0) return HAL_kInvalidHandle;

    int16_t wpi_channel = getPortHandleChannel(portHandle);
    if (InvalidHandleIndex == wpi_channel) {
        *status = PARAMETER_OUT_OF_RANGE;
        return HAL_kInvalidHandle;
    }

    HAL_DigitalHandle digHandle;
    auto port = allocateDigitalHandleAndInitializedPort(HAL_HandleEnum::PWM, wpi_channel, digHandle, status);
    if (port == nullptr) {
        return HAL_kInvalidHandle;
    }

	// TODO:  If HiCurrDIOJumper is "INPUT", show error if requesting one of those channels?

    if (!HAL_Internal_ActivatePWMGenerator(digHandle, status)) {
		digitalChannelHandles->Free(digHandle, HAL_HandleEnum::PWM);
     	return HAL_kInvalidHandle;
    }

    return digHandle;
}

void HAL_FreePWMPort(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    VMXResourceHandle vmxResource = port->vmx_res_handle;
    mau::vmxIO->UnrouteChannelFromResource(port->vmx_chan_info.index, vmxResource, status);
    bool isActive = false;
    mau::vmxIO->IsResourceActive(vmxResource, isActive, status);
    if (isActive) {
        bool allocated = false;
        bool isShared = false;
        mau::vmxIO->IsResourceAllocated(vmxResource, allocated, isShared, status);
        if (allocated) {
			uint8_t num_routed_channels = 0;
			mau::vmxIO->GetNumChannelsRoutedToResource(vmxResource, num_routed_channels, status);
			if (num_routed_channels == 0) {
					mau::vmxIO->DeallocateResource(vmxResource, status);
			}
        }
    }

    digitalChannelHandles->Free(pwmPortHandle, HAL_HandleEnum::PWM);
}

HAL_Bool HAL_CheckPWMChannel(int32_t channel) {
	return isWPILibChannelValid(HAL_ChannelAddressDomain::PWM, channel);
}

/* NOTE:  This function can operate on either a PWM or an ouput-capable DIO resource. */

void
HAL_SetPWMConfig(HAL_DigitalHandle pwmPortHandle, double max, double deadbandMax, double center, double deadbandMin,
                 double min, int32_t* status) {
	HAL_HandleEnum handleType = getHandleType(pwmPortHandle);
	if ((handleType != HAL_HandleEnum::PWM) && (handleType != HAL_HandleEnum::DIO)) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

    auto port = digitalChannelHandles->Get(pwmPortHandle, handleType);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    // calculate the loop time in milliseconds
    double loopTime = HAL_GetPWMLoopTiming(status) / (kSystemClockTicksPerMicrosecond * 1e3);
    if (*status != 0) return;

    int32_t maxPwm = static_cast<int32_t>((max - kDefaultPwmCenter) / loopTime +
                                          kDefaultPwmStepsDown);
    int32_t deadbandMaxPwm = static_cast<int32_t>(
            (deadbandMax - kDefaultPwmCenter) / loopTime + kDefaultPwmStepsDown);
    int32_t centerPwm = static_cast<int32_t>(
            (center - kDefaultPwmCenter) / loopTime + kDefaultPwmStepsDown);
    int32_t deadbandMinPwm = static_cast<int32_t>(
            (deadbandMin - kDefaultPwmCenter) / loopTime + kDefaultPwmStepsDown);
    int32_t minPwm = static_cast<int32_t>((min - kDefaultPwmCenter) / loopTime +
                                          kDefaultPwmStepsDown);

    port->maxPwm = maxPwm;
    port->deadbandMaxPwm = deadbandMaxPwm;
    port->deadbandMinPwm = deadbandMinPwm;
    port->centerPwm = centerPwm;
    port->minPwm = minPwm;
    port->configSet = true;
}

void HAL_SetPWMConfigRaw(HAL_DigitalHandle pwmPortHandle, int32_t maxPwm, int32_t deadbandMaxPwm, int32_t centerPwm,
                         int32_t deadbandMinPwm, int32_t minPwm, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    port->maxPwm = maxPwm;
    port->deadbandMaxPwm = deadbandMaxPwm;
    port->deadbandMinPwm = deadbandMinPwm;
    port->centerPwm = centerPwm;
    port->minPwm = minPwm;
}

void HAL_GetPWMConfigRaw(HAL_DigitalHandle pwmPortHandle, int32_t* maxPwm, int32_t* deadbandMaxPwm, int32_t* centerPwm,
                         int32_t* deadbandMinPwm, int32_t* minPwm, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    *maxPwm = port->maxPwm;
    *deadbandMaxPwm = port->deadbandMaxPwm;
    *deadbandMinPwm = port->deadbandMinPwm;
    *centerPwm = port->centerPwm;
    *minPwm = port->minPwm;
}

void HAL_SetPWMEliminateDeadband(HAL_DigitalHandle pwmPortHandle, HAL_Bool eliminateDeadband, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    port->eliminateDeadband = eliminateDeadband;
}

HAL_Bool HAL_GetPWMEliminateDeadband(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return false;
    }
    return port->eliminateDeadband;
}

/**
 * Set a PWM channel to the desired value. The values range from 0 to 255 and
 * the period is controlled
 * by the PWM Period and MinHigh registers.
 *
 * @param channel The PWM channel to set.
 * @param value The PWM value to set.
 */
void HAL_SetPWMRaw(HAL_DigitalHandle pwmPortHandle, int32_t value, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

	VMXResourcePortIndex portIndex = 0;
    if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }

    mau::vmxIO->PWMGenerator_SetDutyCycle(port->vmx_res_handle, portIndex, value, status);
}

/**
 * Set a PWM channel to the desired scaled value. The values range from -1 to 1
 * and
 * the period is controlled
 * by the PWM Period and MinHigh registers.
 *
 * @param channel The PWM channel to set.
 * @param value The scaled PWM value to set.
 */
void HAL_SetPWMSpeed(HAL_DigitalHandle pwmPortHandle, double speed, int32_t* status) {

	HAL_HandleEnum handleType = getHandleType(pwmPortHandle);
	if ((handleType != HAL_HandleEnum::PWM) && (handleType != HAL_HandleEnum::DIO)) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

    auto port = digitalChannelHandles->Get(pwmPortHandle, handleType);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    if (!port->configSet) {
        *status = INCOMPATIBLE_STATE;
        return;
    }

    if (speed < -1.0) {
        speed = -1.0;
    } else if (speed > 1.0) {
        speed = 1.0;
    }

    int dutyCycle;
    int minPwm = port->minPwm;
    int maxPwm = port->maxPwm;
    if (speed <= -1.0) {
        dutyCycle = minPwm;
    } else if (speed == 1) {
        dutyCycle = maxPwm;
    } else {
        speed += 1;
        double diff = maxPwm - minPwm;
        dutyCycle = minPwm + (speed * (diff / 2.0));
    }

	VMXResourcePortIndex portIndex = 0;
    if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }

    mau::vmxIO->PWMGenerator_SetDutyCycle(port->vmx_res_handle, portIndex, dutyCycle, status);
}

/**
 * Set a PWM channel to the desired position value. The values range from 0 to 1
 * and
 * the period is controlled
 * by the PWM Period and MinHigh registers.
 *
 * @param channel The PWM channel to set.
 * @param value The scaled PWM value to set.
 */
void HAL_SetPWMPosition(HAL_DigitalHandle pwmPortHandle, double pos, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    if (!port->configSet) {
        *status = INCOMPATIBLE_STATE;
        return;
    }

    if (pos < 0.0) {
        pos = 0.0;
    } else if (pos > 1.0) {
        pos = 1.0;
    }

    int dutyCycle = port->minPwm + (int)(pos * (port->maxPwm - port->minPwm));

	VMXResourcePortIndex portIndex = 0;
    if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }

    mau::vmxIO->PWMGenerator_SetDutyCycle(port->vmx_res_handle, portIndex, dutyCycle, status);
}

void HAL_SetPWMDisabled(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

	VMXResourcePortIndex portIndex = 0;
    if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }

    mau::vmxIO->PWMGenerator_SetDutyCycle(port->vmx_res_handle, portIndex, 0, status);
}

/**
 * Get a value from a PWM channel. The values range from 0 to 255.
 *
 * @param channel The PWM channel to read from.
 * @return The raw PWM value.
 */
int32_t HAL_GetPWMRaw(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return 0;
    }

	VMXResourcePortIndex portIndex = 0;
    if (port->vmx_chan_info.capabilities & VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }

    uint16_t currDutyCycleValue;
    if (mau::vmxIO->PWMGenerator_GetDutyCycle(port->vmx_res_handle, portIndex, &currDutyCycleValue, status)) {
    	return static_cast<int32_t>(currDutyCycleValue);
    } else {
    	return 0;
    }
}

/**
 * Get a scaled value from a PWM channel. The values range from -1 to 1.
 *
 * @param channel The PWM channel to read from.
 * @return The scaled PWM value.
 */
double HAL_GetPWMSpeed(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return 0;
    }
    if (!port->configSet) {
        *status = INCOMPATIBLE_STATE;
        return 0;
    }

    int32_t currDutyCycle = HAL_GetPWMRaw(pwmPortHandle, status);
    if (currDutyCycle == 0) {
    	return 0.0f;
    } else {
        double speed;
        if (currDutyCycle <= port->minPwm) {
            speed = -1.0f;
        } else if (currDutyCycle >= port->maxPwm) {
            speed = 1.0f;
        } else {
        	if ((currDutyCycle >= port->deadbandMinPwm) &&
        		(currDutyCycle <= port->deadbandMaxPwm)) {
        		speed = 0.0f;
        	} else {
            	double speedPerDutyCycleTick = 2.0 / (port->maxPwm - port->minPwm);
            	speed = -1.0f + (speedPerDutyCycleTick * (currDutyCycle - port->minPwm));
        	}
        }

    	return speed;
    }
}

/**
 * Get a position value from a PWM channel. The values range from 0 to 1.
 *
 * @param channel The PWM channel to read from.
 * @return The scaled PWM value.
 */
double HAL_GetPWMPosition(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return 0;
    }
    if (!port->configSet) {
        *status = INCOMPATIBLE_STATE;
        return 0;
    }

    int32_t currDutyCycle = HAL_GetPWMRaw(pwmPortHandle, status);
    if (currDutyCycle == 0) {
    	return 0.5f;
    } else {
        double speed;
        if (currDutyCycle <= port->minPwm) {
            speed = 0.0f;
        } else if (currDutyCycle >= port->maxPwm) {
            speed = 1.0f;
        } else {
        	if ((currDutyCycle >= port->deadbandMinPwm) &&
        		(currDutyCycle <= port->deadbandMaxPwm)) {
        		speed = 0.5f;
        	} else {
            	double speedPerDutyCycleTick = 1.0 / (port->maxPwm - port->minPwm);
            	speed = 0.0f + (speedPerDutyCycleTick * (currDutyCycle - port->minPwm));
        	}
        }

    	return speed;
    }

    return 0;
}

void HAL_LatchPWMZero(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    // TODO:  Review this w/Thad
    // NOTE:  The purpose of latching of PWM Zero is not currently understood.
    // This is invoked from the constructors of the various PWM Motor Controller.
    // At this time, it is not implemented.
}

/**
 * Set how how often the PWM signal is squelched, thus scaling the period.
 *
 * @param channel The PWM channel to configure.
 * @param squelchMask The 2-bit mask of outputs to squelch.
 */
void HAL_SetPWMPeriodScale(HAL_DigitalHandle pwmPortHandle, int32_t squelchMask, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    // The VMX-pi PWM resource has already been initialized at this point.
    // If the squelch mask is non-zero, deallocate the resource, reconfigure the PWM Generator Config
    // with the appropriate
    if (squelchMask != 0) {
		bool isActive = false;
		mau::vmxIO->IsResourceActive(port->vmx_res_handle, isActive, status);
		if(isActive) {
			mau::vmxIO->DeallocateResource(port->vmx_res_handle, status);
		}

		if (squelchMask == 1) {
			port->pwmgen_config.SetFrameOutputFilter(PWMGeneratorConfig::FrameOutputFilter::x2);
		} else if (squelchMask == 3) {
			port->pwmgen_config.SetFrameOutputFilter(PWMGeneratorConfig::FrameOutputFilter::x4);
		}
		mau::vmxIO->ActivateSinglechannelResource(port->vmx_chan_info, &port->pwmgen_config, port->vmx_res_handle, status);
    }
}

/**
 * Get the loop timing of the PWM system
 * This is the pwm duty cycle resolution in milliseconds
 *
 * @return The loop time
 */
int32_t HAL_GetPWMLoopTiming(int32_t* status) { return kExpectedLoopTiming; }

/**
 * Get the pwm starting cycle time.  This is believed to be the
 * system timestamp at which the current PWM cycle started.  VMX-pi
 * does not currently capture the start timestamp for PWM Generators.
 *
 * @return The pwm cycle start time.
 */
uint64_t HAL_GetPWMCycleStartTime(int32_t* status) { return 0; }
	// TODO:  If necessary, provide a VMX-pi implementation for this.
}  // extern "C"
