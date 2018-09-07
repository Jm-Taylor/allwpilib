/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "HAL/PWM.h"

#include "ConstantsInternal.h"
#include "DigitalInternal.h"
#include "HAL/handles/HandlesInternal.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauInternal.h"

#include <VMXResource.h>

using namespace hal;

namespace hal {
    namespace init {
        void InitializePWM() {}
    }
}

static const uint16_t dutyCycleTicks = 5000;

extern "C" {
HAL_DigitalHandle HAL_InitializePWMPort(HAL_PortHandle portHandle, int32_t* status) {
    hal::init::CheckInit();
    if (*status != 0) return HAL_kInvalidHandle;

    int16_t channel = getPortHandleChannel(portHandle);
    if (channel == InvalidHandleIndex) {
        *status = PARAMETER_OUT_OF_RANGE;
        return HAL_kInvalidHandle;
    }

    uint8_t origChannel = static_cast<uint8_t>(channel);

    auto handle =
            digitalChannelHandles->Allocate(channel, HAL_HandleEnum::PWM, status);

    if (*status != 0)
        return HAL_kInvalidHandle;  // failed to allocate. Pass error back.

    auto port = digitalChannelHandles->Get(handle, HAL_HandleEnum::PWM);
    if (port == nullptr) {  // would only occur on thread issue.
        *status = HAL_HANDLE_ERROR;
        return HAL_kInvalidHandle;
    }

    port->channel = origChannel;

    // ---------------------------------------

    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, origChannel);

    /* Determine VMX Channel PWM Generator Port Index assignment */
    VMXChannelType channelType;
    VMXChannelCapability channelCapabilityBits;
    mau::vmxIO->GetChannelCapabilities(mauChannel->vmxIndex, channelType, channelCapabilityBits);
    if (channelCapabilityBits & VMXChannelCapability::PWMGeneratorOutput) {
	// This channel must use the first port (0) on the PWM Generator resource
    	mauChannel->vmxAbility = VMXChannelCapability::PWMGeneratorOutput;
    } else if (channelCapabilityBits & VMXChannelCapability::PWMGeneratorOutput2) {
	// This channel must use the second port (1) on the PWM Generator resource
    	mauChannel->vmxAbility = VMXChannelCapability::PWMGeneratorOutput2;
    }

    HAL_SetPWMConfig(handle, 2.0, 1.501, 1.5, 1.499, 1.0, status);

    PWMGeneratorConfig vmxConfig(200 /* Frequency in Hz */);
    vmxConfig.SetMaxDutyCycleValue(dutyCycleTicks); /* Update Duty Cycle Range to match WPI Library cycle resolution (1 us/tick) */
    if (!mau::vmxIO->ActivateSinglechannelResource(mauChannel->getInfo(), &vmxConfig, mauChannel->vmxResHandle, mau::vmxError)) {
    	if (*mau::vmxError == VMXERR_IO_NO_UNALLOCATED_COMPATIBLE_RESOURCES) {
			VMXResourceHandle resourceWithAvailablePort;
			bool allocated;
    		if (mau::vmxIO->GetResourceHandleWithAvailablePortForChannel(
    				PWMGenerator, mauChannel->vmxIndex, mauChannel->vmxAbility, resourceWithAvailablePort, allocated, mau::vmxError)) {
    			if (mau::vmxIO->RouteChannelToResource(mauChannel->vmxIndex, resourceWithAvailablePort, mau::vmxError)) {
    				mauChannel->vmxResHandle = resourceWithAvailablePort;
    				return handle;
    			}
    		}
    	}
    	// TODO:  Log VMX Error Code Description
    	return HAL_kInvalidHandle;
    }

    return handle;
}

void HAL_FreePWMPort(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
    VMXResourceHandle vmxResource = mauChannel->vmxResHandle;
    mau::vmxIO->UnrouteChannelFromResource(mauChannel->vmxIndex, vmxResource, mau::vmxError);
    bool isActive;
    mau::vmxIO->IsResourceActive(vmxResource, isActive, mau::vmxError);
    if (isActive) {
        bool allocated;
        bool isShared;
        mau::vmxIO->IsResourceAllocated(vmxResource, allocated, isShared, mau::vmxError);
        if (allocated) {
	    uint8_t num_routed_channels = 0;
	    mau::vmxIO->GetNumChannelsRoutedToResource(vmxResource, num_routed_channels, mau::vmxError);
	    if (num_routed_channels == 0) {
	            mau::vmxIO->DeactivateResource(vmxResource, mau::vmxError);
		mauChannel->vmxResHandle = 0;
	    }
        }
    }

    digitalChannelHandles->Free(pwmPortHandle, HAL_HandleEnum::PWM);
}

HAL_Bool HAL_CheckPWMChannel(int32_t channel) {
    return channel < kNumPWMChannels && channel >= 0;
}

void
HAL_SetPWMConfig(HAL_DigitalHandle pwmPortHandle, double max, double deadbandMax, double center, double deadbandMin,
                 double min, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
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

    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
    VMXResourcePortIndex portIndex = 0;
    if (mauChannel->vmxAbility == VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }

    mau::vmxIO->PWMGenerator_SetDutyCycle(mauChannel->vmxResHandle, portIndex, value, mau::vmxError);
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
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
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

    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
    VMXResourcePortIndex portIndex = 0;
    if (mauChannel->vmxAbility == VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }
    mau::vmxIO->PWMGenerator_SetDutyCycle(mauChannel->vmxResHandle, portIndex, dutyCycle, mau::vmxError);
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
    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
    VMXResourcePortIndex portIndex = 0;
    if (mauChannel->vmxAbility == VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }
    mau::vmxIO->PWMGenerator_SetDutyCycle(mauChannel->vmxResHandle, portIndex, dutyCycle, mau::vmxError);
}

void HAL_SetPWMDisabled(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
    auto port = digitalChannelHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
    if (port == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
    VMXResourcePortIndex portIndex = 0;
    if (mauChannel->vmxAbility == VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }
    mau::vmxIO->PWMGenerator_SetDutyCycle(mauChannel->vmxResHandle, portIndex, 0, mau::vmxError);
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

    std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
    Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
    VMXResourcePortIndex portIndex = 0;
    if (mauChannel->vmxAbility == VMXChannelCapability::PWMGeneratorOutput2) {
    	portIndex = 1;
    }
    uint16_t currDutyCycleValue;
    if (mau::vmxIO->PWMGenerator_GetDutyCycle(mauChannel->vmxResHandle, portIndex, &currDutyCycleValue, mau::vmxError)) {
    	return int32_t(currDutyCycleValue);
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
		std::string vmxLabel = mau::enumConverter->getHandleLabel(HAL_HandleEnum::PWM);
		Mau_Channel* mauChannel = mau::channelMap->getChannel(vmxLabel, port->channel);
		bool isActive;
		mau::vmxIO->IsResourceActive(mauChannel->vmxResHandle, isActive, mau::vmxError);
		if(isActive) {
			mau::vmxIO->DeallocateResource(mauChannel->vmxResHandle, mau::vmxError);
		}
		PWMGeneratorConfig vmxConfig(200 /* Frequency in Hz */);
		vmxConfig.SetMaxDutyCycleValue(5000); /* Update Duty Cycle Range to match WPI Library cycle resolution (1 us/tick) */
		if (squelchMask == 1) {
			vmxConfig.SetFrameOutputFilter(PWMGeneratorConfig::FrameOutputFilter::x2);
		} else if (squelchMask == 3) {
			vmxConfig.SetFrameOutputFilter(PWMGeneratorConfig::FrameOutputFilter::x4);
		}
		mau::vmxIO->ActivateSinglechannelResource(mauChannel->getInfo(), &vmxConfig, mauChannel->vmxResHandle, mau::vmxError);
    }
}

/**
 * Get the loop timing of the PWM system
 *
 * @return The loop time
 */
int32_t HAL_GetPWMLoopTiming(int32_t* status) { return kExpectedLoopTiming; }

/**
 * Get the pwm starting cycle time
 *
 * @return The pwm cycle start time.
 */
uint64_t HAL_GetPWMCycleStartTime(int32_t* status) { return 0; }
}  // extern "C"