/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Encoder.h"

#include "CounterInternal.h"
#include "hal/Counter.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "DigitalInternal.h"
#include "AnalogInternal.h"
#include "AnalogTriggerInternal.h"
#include "InterruptsInternal.h"
#include <VMXChannel.h>
#include <VMXResource.h>
#include <VMXResourceConfig.h>
#include <VMXIO.h>

using namespace hal;

namespace {
struct EncoderResource {
	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
			VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	EncoderConfig vmx_config;
	HAL_DigitalHandle a_input_digital_handle = HAL_kInvalidHandle;
	HAL_DigitalHandle b_input_digital_handle = HAL_kInvalidHandle;
	HAL_DigitalHandle index_input_digital_handle = HAL_kInvalidHandle;
	double distancePerPulse = 1.0;
	int16_t index = 0; // index into resource handle table; this is also used as the externally-visibile "FPGA Index"
				   	   // it is not at all clear why the FPGA Index is made visible from the HAL API....
	bool reverse_direction = false;
	int32_t samplesToAverage = 0;
	int32_t lastEncoderValue = 0;
	uint32_t readErrorCount = 0;
	VMXResourceHandle index_interrupt_vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
			VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	bool index_interrupt_is_owned = false;
};
}  // namespace

static bool AllocateVMXPiEncoder(
		std::shared_ptr<EncoderResource> encoderResource,
		std::shared_ptr<DigitalPort> digPortA,
		std::shared_ptr<DigitalPort> digPortB, int32_t* status) {

	VMXChannelInfo channel_infos[2];
	channel_infos[0] = digPortA->vmx_chan_info;
	channel_infos[1] = digPortB->vmx_chan_info;
	// Only request the appropriate capabilities for both encoder iput channels
	for (int i = 0; i < 2; i++) {
		if (channel_infos[i].capabilities & VMXChannelCapability::EncoderAInput) {
			channel_infos[i].capabilities = VMXChannelCapability::EncoderAInput;
		} else if (channel_infos[i].capabilities & VMXChannelCapability::EncoderBInput) {
			channel_infos[i].capabilities = VMXChannelCapability::EncoderBInput;
		}
		// Deallocate exiting routings for these channels
		// TODO:  Save the previous allocation and restore upon completion?
		VMXResourceHandle res_handle;
		if (mau::vmxIO->GetResourceFromRoutedChannel(channel_infos[i].index, res_handle, NULL)) {
			mau::vmxIO->DeallocateResource(res_handle, NULL);
		}
	}
	const VMXChannelInfo *p_channel_infos = &channel_infos[0];
	if (!mau::vmxIO->ActivateMultichannelResource(2, p_channel_infos,
			&encoderResource->vmx_config, encoderResource->vmx_res_handle,
			status)) {
		return false;
	}

	return true;
}

/* Can be used to deallocate either a DigitalIO or a PWMGenerator VMXPi Resource. */
static void DeallocateVMXPiEncoder(std::shared_ptr<EncoderResource> encoderResource,
		bool& isActive) {

	// Deallocate any owned index source resources
	if (encoderResource->index_interrupt_is_owned &&
		(!INVALID_VMX_RESOURCE_HANDLE(encoderResource->index_interrupt_vmx_res_handle))) {
		mau::vmxIO->DeactivateResource(encoderResource->index_interrupt_vmx_res_handle, mau::vmxError);
	}

	isActive = false;
	mau::vmxIO->IsResourceActive(encoderResource->vmx_res_handle, isActive,
			mau::vmxError);
	if (isActive) {
		mau::vmxIO->DeallocateResource(encoderResource->vmx_res_handle, mau::vmxError);
	}
	encoderResource->vmx_res_handle = 0;
}

static LimitedHandleResource<HAL_EncoderHandle, EncoderResource, kNumEncoders,
		HAL_HandleEnum::Encoder>* encoderHandles;

namespace hal {
namespace init {
void InitializeEncoder() {
	static LimitedHandleResource<HAL_EncoderHandle, EncoderResource,
			kNumEncoders, HAL_HandleEnum::Encoder> eH;
	encoderHandles = &eH;
}
}  // namespace init
}  // namespace hal

extern "C" {
HAL_EncoderHandle HAL_InitializeEncoder(HAL_Handle digitalSourceHandleA,
		HAL_AnalogTriggerType analogTriggerTypeA,
		HAL_Handle digitalSourceHandleB,
		HAL_AnalogTriggerType analogTriggerTypeB, HAL_Bool reverseDirection,
		HAL_EncoderEncodingType encodingType, int32_t* status) {
	hal::init::CheckInit();

	// NOTE:  Unlike the reference implementation, VMX-pi does not
	// support analog triggers as encoder sources.
	HAL_HandleEnum sourceAType = getHandleType(digitalSourceHandleA);
	HAL_HandleEnum sourceBType = getHandleType(digitalSourceHandleB);
	if ((sourceAType == HAL_HandleEnum::AnalogInput) ||
		(sourceAType == HAL_HandleEnum::AnalogInput) ||
		(sourceBType == HAL_HandleEnum::AnalogTrigger) ||
		(sourceBType == HAL_HandleEnum::AnalogTrigger)) {
		*status = MAU_ENCODER_ANALOG_INPUTS_UNSUPPORTED;
		return HAL_kInvalidHandle;
	}

	auto dioPortA = digitalChannelHandles->Get(digitalSourceHandleA,
			HAL_HandleEnum::DIO);
	if (dioPortA == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	auto dioPortB = digitalChannelHandles->Get(digitalSourceHandleB,
			HAL_HandleEnum::DIO);
	if (dioPortB == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}

	auto handle = encoderHandles->Allocate();
	if (handle == HAL_kInvalidHandle) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}
	auto encoder = encoderHandles->Get(handle);
	if (encoder == nullptr) {  // would only occur on thread issue
		encoderHandles->Free(handle);
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}

	*encoder.get() = EncoderResource();

	int16_t index = getHandleIndex(handle);
	encoder->index = index;
	if (encodingType == HAL_EncoderEncodingType::HAL_Encoder_k1X) {
		encoder->vmx_config.SetEncoderEdge(EncoderConfig::EncoderEdge::x1);
	} else if (encodingType == HAL_EncoderEncodingType::HAL_Encoder_k2X) {
		encoder->vmx_config.SetEncoderEdge(EncoderConfig::EncoderEdge::x2);
	} else {
		encoder->vmx_config.SetEncoderEdge(EncoderConfig::EncoderEdge::x4);
	}

	if (!AllocateVMXPiEncoder(encoder, dioPortA, dioPortB, status)) {
		encoderHandles->Free(handle);
		return HAL_kInvalidHandle;
	}

	encoder->a_input_digital_handle = digitalSourceHandleA;
	encoder->b_input_digital_handle = digitalSourceHandleB;

	return handle;
}

void HAL_FreeEncoder(HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder != nullptr) {
		bool active;
		DeallocateVMXPiEncoder(encoder,active);
		*encoder.get() = EncoderResource();
	}
	encoderHandles->Free(encoderHandle);
}

void HAL_SetEncoderSimDevice(HAL_EncoderHandle handle,
                             HAL_SimDeviceHandle device) {}

/* Get the current count from the encoder.  This method compensates for the decoding type. */
int32_t HAL_GetEncoder(HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	/* In case of intermittent board communication error (e.g., due to  */
	/* CRC error during communication), return the last successfully    */
	/* returned value.                                                  */
	/* This behavior is implemented due to the robot base code which    */
	/* will terminate when non-zero status, generating an "Unclean      */
	/* Status Exception.                                                */
	int32_t counts;
	const int c_max_successive_comm_errors = 3;
	if(!mau::vmxIO->Encoder_GetCount(encoder->vmx_res_handle, counts, status)) {
		if ((VMXERR_IO_BOARD_COMM_ERROR == *status) && (encoder->readErrorCount < c_max_successive_comm_errors)) {
			*status = 0;
			counts = encoder->lastEncoderValue;
		} else {
			return 0;
		}
		encoder->readErrorCount++;
	} else {
		encoder->readErrorCount = 0;
		encoder->lastEncoderValue = counts;
	}

	if (encoder->reverse_direction) {
		counts = -counts;
	}

	return counts;
}

/* Get the current count from the encoder.  This raw value is the actual count, and */
/* is not scaled by the 1x, 2x or 4x scale factor.                                  */
int32_t HAL_GetEncoderRaw(HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	// The presence of this function suggests that the reference implementation always
	// operates in 4X mode (whereas VMX-pi configures the hardware to operate in 1, 2 or 4x mode).

	// To simulate the expected function behavior on VMX-pi, if 1x mode, multiply by 4; in 2x mode, multiply by 2.
	int32_t counts = HAL_GetEncoder(encoderHandle,status);

	switch (encoder->vmx_config.GetEncoderEdge()) {
	case EncoderConfig::EncoderEdge::x2:
		counts *= 2;
		break;
	case EncoderConfig::EncoderEdge::x4:
		counts *= 4;
		break;
	default:
		break;
	}

	return counts;
}

int32_t HAL_GetEncoderEncodingScale(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 1;
	}
	switch (encoder->vmx_config.GetEncoderEdge()) {
	case EncoderConfig::EncoderEdge::x1:
		return 1;
		break;
	case EncoderConfig::EncoderEdge::x2:
		return 2;
		break;
	case EncoderConfig::EncoderEdge::x4:
		return 4;
		break;
	}

	return 1;
}

void HAL_ResetEncoder(HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	uint8_t retry_count = 3;
	do { 
		mau::vmxIO->Encoder_Reset(encoder->vmx_res_handle, status);
	} while (mau::RetryWriteOnBoardCommError(status, retry_count)); 	
}

double HAL_GetEncoderPeriod(HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint16_t last_pulse_microseconds = 0;
	mau::vmxIO->Encoder_GetLastPulsePeriodMicroseconds(encoder->vmx_res_handle, last_pulse_microseconds, status);
	mau::ClearBoardCommErrorStatus(status);
	if (last_pulse_microseconds == 0) return 0.0;

	return static_cast<double>(last_pulse_microseconds) / 1000000.0;
}

void HAL_SetEncoderMaxPeriod(HAL_EncoderHandle encoderHandle, double maxPeriod,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	auto dioPortA = digitalChannelHandles->Get(encoder->a_input_digital_handle,
			HAL_HandleEnum::DIO);
	if (dioPortA == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}
	auto dioPortB = digitalChannelHandles->Get(encoder->b_input_digital_handle,
			HAL_HandleEnum::DIO);
	if (dioPortB == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	bool active;
	DeallocateVMXPiEncoder(encoder, active);

	uint8_t stall_timeout_20ms_periods = static_cast<uint8_t>(maxPeriod * (1000 / 20));
	encoder->vmx_config.SetStallTimeout20MsPeriods(stall_timeout_20ms_periods);

	AllocateVMXPiEncoder(encoder, dioPortA, dioPortB, status);
}

HAL_Bool HAL_GetEncoderStopped(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	bool forward = true;
	bool active = false;
	mau::vmxIO->InputCapture_InputStatus(encoder->vmx_res_handle, forward, active, status);
	mau::ClearBoardCommErrorStatus(status);
	return !active;
}

HAL_Bool HAL_GetEncoderDirection(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	bool forward = true;
	bool active = false;
	mau::vmxIO->InputCapture_InputStatus(encoder->vmx_res_handle, forward, active, status);
	mau::ClearBoardCommErrorStatus(status);
	return encoder->reverse_direction ? !forward : forward;
}

double HAL_GetEncoderDistance(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	int32_t encoder_counts = HAL_GetEncoder(encoderHandle, status);
	double distance = encoder->distancePerPulse * encoder_counts;
	return distance;
}

/*
* Get the current rate of the encoder.
*
* Units are distance per second as scaled by the value from
* SetDistancePerPulse().
*/
double HAL_GetEncoderRate(HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	double encoder_period = HAL_GetEncoderPeriod(encoderHandle, status);
	if (encoder_period == 0.0) return 0.0;

	double pulses_per_second = (1.0 / encoder_period / 2);

	double rate = pulses_per_second * encoder->distancePerPulse;

	return rate;
}

void HAL_SetEncoderMinRate(HAL_EncoderHandle encoderHandle, double minRate,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (minRate == 0.0) {
		*status = PARAMETER_OUT_OF_RANGE;
		return;
	}

	// The Inverse of the Min Rate is the Max Period
	double max_period = 1.0 / minRate;

	HAL_SetEncoderMaxPeriod(encoderHandle, max_period, status);
}

void HAL_SetEncoderDistancePerPulse(HAL_EncoderHandle encoderHandle,
		double distancePerPulse, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (distancePerPulse == 0.0) {
		*status = PARAMETER_OUT_OF_RANGE;
		return;
	}
	encoder->distancePerPulse = distancePerPulse;
}

void HAL_SetEncoderReverseDirection(HAL_EncoderHandle encoderHandle,
		HAL_Bool reverseDirection, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	encoder->reverse_direction = reverseDirection;
}

void HAL_SetEncoderSamplesToAverage(HAL_EncoderHandle encoderHandle,
		int32_t samplesToAverage, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	auto dioPortA = digitalChannelHandles->Get(encoder->a_input_digital_handle,
			HAL_HandleEnum::DIO);
	if (dioPortA == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}
	auto dioPortB = digitalChannelHandles->Get(encoder->b_input_digital_handle,
			HAL_HandleEnum::DIO);
	if (dioPortB == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	encoder->samplesToAverage = samplesToAverage;

	bool active;
	DeallocateVMXPiEncoder(encoder, active);

	// The reference implementations FPGA clock cycle time is understood to be 100ns.
	uint32_t samples_nanoseconds = samplesToAverage * 100;
	uint8_t closest_input_capture_filter = encoder->vmx_config.GetClosestCaptureChannelFilter(samples_nanoseconds);
	encoder->vmx_config.SetCaptureChannelFilter(InputCaptureConfigBase::CaptureChannel::CH1, closest_input_capture_filter);
	encoder->vmx_config.SetCaptureChannelFilter(InputCaptureConfigBase::CaptureChannel::CH2, closest_input_capture_filter);

	AllocateVMXPiEncoder(encoder, dioPortA, dioPortB, status);
}

int32_t HAL_GetEncoderSamplesToAverage(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	return encoder->samplesToAverage;
}

/* When this source is activated, the encoder count automatically resets. */
/* The digitalSourceHandle can either be a digital source handle          */
/* or an analog trigger handle                                            */
/* In either case, the resulting VMXChannelIndex can be identified.       */
/* It is possible that an interrupt is already assigned to the source.    */
/* If an interrupt is not already assigned to the source, allocate an     */
/* Interrupt Resource handle (store it away to ensure it is later         */
/* released).                                                             */
/* If an interrupt is already assigned to the source, retrieve the        */
/* Interrupt Resource handle (store it away, but ensure that it is not    */
/* later released).  However, if the interrupt resource is configured for */
/* a different interrupt edge type than the requested analogTriggerType,  */
/* return an error.                                                       */

void HAL_SetEncoderIndexSource(HAL_EncoderHandle encoderHandle,
		HAL_Handle digitalSourceHandle, HAL_AnalogTriggerType analogTriggerType,
		HAL_EncoderIndexingType type, int32_t* status) {

	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	VMXResourceIndex vmx_interrupt_res_index;
	VMXChannelInfo vmx_src_channel_info;
	if (!GetVMXInterruptResourceIndexForDigitalSourceHandle(digitalSourceHandle, vmx_interrupt_res_index, vmx_src_channel_info, status)) {
		return;
	}

	// Configure requested indexing type and level reset
	InterruptConfig interrupt_config;
	bool clear_on_level = false;   // If true, encoder count is reset to 0 when index is at clear_level
	bool clear_level_high = true;  // If true, clear level is high; else, clear level is low
	switch (type) {
	default:
	case HAL_EncoderIndexingType::HAL_kResetOnRisingEdge:
		interrupt_config.SetEdge(InterruptConfig::InterruptEdge::RISING);
		clear_on_level = false;
		break;
	case HAL_EncoderIndexingType::HAL_kResetOnFallingEdge:
		interrupt_config.SetEdge(InterruptConfig::InterruptEdge::FALLING);
		clear_on_level = false;
		break;
	case HAL_EncoderIndexingType::HAL_kResetWhileHigh:
		// Encoder counter is cleared both on falling edge interrupt, and also
		// whenever the counter value is read and the signal is high
		interrupt_config.SetEdge(InterruptConfig::InterruptEdge::FALLING);
		clear_on_level = true;
		clear_level_high = true;
		break;
	case HAL_EncoderIndexingType::HAL_kResetWhileLow:
		// Encoder counter is cleared both on rising edge interrupt, and also
		// whenever the counter value is read and the signal is low
		interrupt_config.SetEdge(InterruptConfig::InterruptEdge::RISING);
		clear_on_level = true;
		clear_level_high = false;
		break;
	}

	// Retrieve corresponding VMX-pi interrupt resource handle, determine if allocated
	// TODO:  Retrieve this from the vmxpi wpilib hal interrupt table, not directly from vmx-pi (for reusability)?
	VMXResourceHandle interrupt_resource_handle;
	if (!mau::vmxIO->GetResourceHandle(VMXResourceType::Interrupt, vmx_interrupt_res_index, interrupt_resource_handle, status)) {
		return;
	}
	bool allocated = false;
	bool shared = false;
	if (!mau::vmxIO->IsResourceAllocated(interrupt_resource_handle, allocated, shared, status)) {
		return;
	}

	if (!allocated) {
		// Requested Interrupt Resource not currently allocated: allocate/activate it
		VMXChannelInfo interrupt_chan_info(vmx_interrupt_res_index, VMXChannelCapability::InterruptInput);
		if (!mau::vmxIO->ActivateSinglechannelResource(interrupt_chan_info, &interrupt_config, interrupt_resource_handle, status)) {
			return;
		}
		// indicate the interrupt resource should be released automatically later
		encoder->index_interrupt_is_owned = true;

	} else {
		// Requested Interrupt is already allocated; verify it's currently-configured active edge matches
		// the requested TriggerType
		InterruptConfig curr_interrupt_config;
		VMXResourceConfig *p_cfg = &curr_interrupt_config;
		if (!mau::vmxIO->GetResourceConfig(interrupt_resource_handle, p_cfg, status)) {
			return;
		}

		// If the existing interrupt resource is configured with a different InterruptEdge than
		// what was requested via HAL_EncoderIndexingType, return an error.
		if (interrupt_config.GetEdge() != curr_interrupt_config.GetEdge()) {
			*status = ENCODER_INDEX_SRC_INT_CONFIG_CONFLICT;
			return;
		}

		// indicate the interrupt resource should NOT be released automatically later
		// (since the resource was previous allocated)
		encoder->index_interrupt_is_owned = false;
	}
	encoder->index_interrupt_vmx_res_handle = interrupt_resource_handle;

	// Configure VMX-pi to reset the encoder when interrupt occurs.
	if (!mau::vmxIO->Encoder_SetResetSource(encoder->vmx_res_handle, interrupt_resource_handle, clear_on_level, clear_level_high, status)) {
		if (encoder->index_interrupt_is_owned) {
			mau::vmxIO->DeactivateResource(interrupt_resource_handle, status);
			return;
		}
	}

	// At this point, the encoder counter will be auto-cleared (in the VMX-pi firmware & VMX-pi HAL Libraries)
	// whenever the requested trigger type occurs.
}

// It's not exactly clear why this API function is provided,
// as it appears to expose internal details that should not
// be necessary for external parties to access.
int32_t HAL_GetEncoderFPGAIndex(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	return encoder->index;
}

/**
 * The scale needed to convert a raw counter value into a number of encoder
 * pulses.  Since the raw counts represent every 4X pulse, this decoding
 * scale factor can be .25 for 4x mode, .5 for 2x mode, 1 for 1x mode.
 */
double HAL_GetEncoderDecodingScaleFactor(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0.0;
	}

	double encoding_scale = 1.0 / static_cast<double>(HAL_GetEncoderEncodingScale(encoderHandle, status));

	return double(encoding_scale);
}

/* Distance is unitless.  This allows users to define their own */
/* units, e.g., linear or angular.                              */
double HAL_GetEncoderDistancePerPulse(HAL_EncoderHandle encoderHandle,
		int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0.0;
	}

	return encoder->distancePerPulse;
}

HAL_EncoderEncodingType HAL_GetEncoderEncodingType(
		HAL_EncoderHandle encoderHandle, int32_t* status) {
	auto encoder = encoderHandles->Get(encoderHandle);
	if (encoder == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return HAL_Encoder_k4X;  // default to k4x
	}

	switch (encoder->vmx_config.GetEncoderEdge()) {
	case EncoderConfig::EncoderEdge::x1:
		return HAL_Encoder_k1X;
		break;
	case EncoderConfig::EncoderEdge::x2:
		return HAL_Encoder_k2X;
		break;
	default:
	case EncoderConfig::EncoderEdge::x4:
		return HAL_Encoder_k4X;
		break;
	}
}

}  // extern "C"
