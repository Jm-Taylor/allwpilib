/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include <memory>

#include "hal/AnalogTrigger.h"
#include "hal/Ports.h"
#include "hal/Types.h"
#include "hal/handles/DigitalHandleResource.h"
#include "hal/handles/HandlesInternal.h"
#include "PortsInternal.h"
#include "MauInternal.h"

namespace hal {
/**
 * MXP channels when used as digital output PWM are offset from actual value
 */
constexpr int32_t kMXPDigitalPWMOffset = 6;

// The VMX-pi system clock resolution is 1 microsecond.
// The VMX-pi duty cycle frequency is 1 microsecond.
// kExpectedLoopTiming is the pwm duty cycle resolution in milliseconds
constexpr int32_t kExpectedLoopTiming = 1;

/**
 * kDefaultPwmPeriod is in ms
 *
 * - 20ms periods (50 Hz) are the "safest" setting in that this works for all
 *   devices
 * - 20ms periods seem to be desirable for Vex Motors
 * - 20ms periods are the specified period for HS-322HD servos, but work
 *   reliably down to 10.0 ms; starting at about 8.5ms, the servo sometimes hums
 *   and get hot; by 5.0ms the hum is nearly continuous
 * - 10ms periods work well for Victor 884
 * - 5ms periods allows higher update rates for Luminary Micro Jaguar speed
 *   controllers. Due to the shipping firmware on the Jaguar, we can't run the
 *   update period less than 5.05 ms.
 *
 * kDefaultPwmPeriod is the 1x period (5.05 ms).  In hardware, the period
 * scaling is implemented as an output squelch to get longer periods for old
 * devices.
 */
constexpr float kDefaultPwmPeriod = 5.05;
/**
 * kDefaultPwmCenter is the PWM range center in ms
 */
constexpr float kDefaultPwmCenter = 1.5;
/**
 * kDefaultPWMStepsDown is the number of PWM steps below the centerpoint
 */
constexpr int32_t kDefaultPwmStepsDown = 1500;
constexpr int32_t kPwmDisabled = 0;

constexpr uint32_t kPwmFrequencyHz = 50;
constexpr uint16_t kDutyCycleTicks = 5000;

struct DigitalPort {
  int32_t channel;
  bool configSet = false;
  bool eliminateDeadband = false;
  int32_t maxPwm = 0;
  int32_t deadbandMaxPwm = 0;
  int32_t centerPwm = 0;
  int32_t deadbandMinPwm = 0;
  int32_t minPwm = 0;
  uint8_t digFilterIndex = 0;
  VMXChannelInfo vmx_chan_info;
  VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::Undefined,INVALID_VMX_RESOURCE_INDEX);
  /* The following configuration objects hold configuration before resource has been activated. */
  /* (after activation, the active configuration can be retrieved from the VMX Resource. */
  DIOConfig dio_config;
  PWMGeneratorConfig pwmgen_config;
  InterruptConfig interrupt_config;
  InputCaptureConfig inputcap_config;
  PWMCaptureConfig pwmcap_config;
  EncoderConfig encoder_config;
  LEDArray_OneWireConfig ledarray_config;
};

constexpr HAL_DigitalHandle kMaxValidDigitalHandleIndex = kNumDigitalChannels-1;

extern DigitalHandleResource<HAL_DigitalHandle, DigitalPort,
                             kNumDigitalChannels>*
    digitalChannelHandles;

/* FlexDIO Channels:  [ 0-11] */
/* HiCrDIO Channels:  [12-21] */
/* CommDIO Channels:  [22-29] */

HAL_DigitalHandle getDigitalHandleForVMXChannelIndex(VMXChannelIndex index, HAL_HandleEnum handleType);
HAL_DigitalHandle getDigitalHandleAndVMXChannelInfo(HAL_HandleEnum handleType, int32_t wpiLibPwmChannel, VMXChannelInfo& info, int32_t *status);
std::shared_ptr<DigitalPort> allocateDigitalPort(HAL_DigitalHandle digHandle, HAL_HandleEnum handleType, int32_t *status);
std::shared_ptr<DigitalPort> allocateDigitalHandleAndInitializedPort(HAL_HandleEnum handleType, int32_t wpiLibChannel, HAL_DigitalHandle& digHandle, int32_t *status);

bool remapDigitalSource(HAL_Handle digitalSourceHandle,
                        HAL_AnalogTriggerType analogTriggerType,
                        uint8_t& channel, uint8_t& module, bool& analogTrigger);
int32_t remapMXPPWMChannel(int32_t channel);
int32_t remapMXPChannel(int32_t channel);

int32_t GetDigitalInputChannel(HAL_DigitalHandle handle, int32_t* status);
}  // namespace hal
