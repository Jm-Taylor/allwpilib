#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/ErrorCode.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <algorithm>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

#include <cstring>
#include <chrono>
#include <thread>
#include <iostream> // std::cout
#include <string>

#include "hal/CAN.h"
#include "hal/DriverStation.h"
#include "CANInternal.h"

namespace ctre {
namespace phoenix {
namespace platform {
namespace can {

#define NUM_CAN_MESSAGES_IN_BATCH 10  // TODO: Tune this based on usage patterns.

//-------------- Low Level CANBus interface, this is required if using phoenix-canutil--------------------------//

void CANbus_GetStatus(float * percentBusUtilization, uint32_t * busOffCount,
		uint32_t * txFullCount, uint32_t * receiveErrorCount,
		uint32_t * transmitErrorCount, int32_t * status) {
	HAL_CAN_GetCANStatus(percentBusUtilization, busOffCount, txFullCount,
			receiveErrorCount, transmitErrorCount, status);
}

int32_t CANbus_SendFrame(uint32_t messageID, const uint8_t *data,
		uint8_t dataSize) {
	int32_t status = 0;
	int32_t periodMs = 0; // TODO:  Check this
	HAL_CAN_SendMessage(messageID | 0x80000000, data, dataSize, periodMs,
			&status);
	// TODO:  Reference implement returns 0 on success, -1 on failure
	return status;
}

int32_t CANbus_ReceiveFrame(canframe_t * toFillArray, uint32_t capacity,
		uint32_t * numberFilled) {
	HAL_CANStreamMessage msgs[NUM_CAN_MESSAGES_IN_BATCH];
	uint32_t remainingMessagesToRead = capacity;
	uint32_t default_stream_handle = MAU_HAL_BLACKBOARD_STREAM_HANDLE; // Constant defined by Mau HAL (see CANInternal.h)

	uint32_t currCANStreamMessageIndex = 0;
	while (remainingMessagesToRead > 0) {
		uint32_t currMessagesToRead =
				(remainingMessagesToRead > NUM_CAN_MESSAGES_IN_BATCH) ?
						NUM_CAN_MESSAGES_IN_BATCH : remainingMessagesToRead;
		remainingMessagesToRead -= currMessagesToRead;
		uint32_t currMessagesRead = 0;
		int32_t status = 0;

		HAL_CAN_ReadStreamSession(default_stream_handle, msgs,
				currMessagesToRead, &currMessagesRead, &status);
		if (status != 0) {
			if (currMessagesRead > 0) {
				*numberFilled = currMessagesRead;
				return 0;
			} else {
				*numberFilled = 0;
				return 1; // Return error indication, no messages retrieved.
			}
		}

		if (currMessagesRead > 0) {
			for (uint32_t i = 0; i < currMessagesRead; i++) {
				if (currCANStreamMessageIndex < capacity) {
					toFillArray[currCANStreamMessageIndex].arbID = msgs[i].messageID
							& 0x1FFFFFFF;
					if (msgs[i].dataSize > 8) {
						printf("Error:  CAN Frame Data Size limited to 8 in VMX-pi Platform_mau CANbus_ReceiveFrame().\n");
						msgs[i].dataSize = 8;
					}
					std::memcpy(toFillArray[currCANStreamMessageIndex].data,
							msgs[i].data, msgs[i].dataSize);
					toFillArray[currCANStreamMessageIndex].dlc = msgs[i].dataSize;
					toFillArray[currCANStreamMessageIndex].timeStampUs =
							msgs[i].timeStamp;
					//Don't set any flags on toFill for right now (TODO:  Check w/Omar on this part???)
					currCANStreamMessageIndex++;
				} else {
					printf("Over-capacity in VMX-pi Platform_mau CANbus_ReceiveFrame().\n");
				}
			}
		}
		if (currMessagesRead < currMessagesToRead) {
			break;
		}
	}
	*numberFilled = currCANStreamMessageIndex;
	return 0;
}

// Registers CAN Device Name with Platform
int32_t SetCANInterface(const char * interface) {
	// No-op on mau platform
	return 0;
}

//-------------- Mid Level CANBus interface, this is required if NOT using phoenix-canutil, --------------------------//
void CANComm_SendMessage(uint32_t messageID, const uint8_t *data,
		uint8_t dataSize, int32_t periodMs, int32_t *status) {
	HAL_CAN_SendMessage(messageID, data, dataSize, periodMs,
			status);
}

void CANComm_ReceiveMessage(uint32_t *messageID, uint32_t messageIDMask,
		uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp,
		int32_t *status) {
	HAL_CAN_ReceiveMessage(messageID, messageIDMask, data, dataSize, timeStamp,
			status);
}

void CANComm_OpenStreamSession(uint32_t *sessionHandle, uint32_t messageID,
		uint32_t messageIDMask, uint32_t maxMessages, int32_t *status) {
	HAL_CAN_OpenStreamSession(sessionHandle, messageID, messageIDMask,
			maxMessages, status);
}

void CANComm_CloseStreamSession(uint32_t sessionHandle) {
	HAL_CAN_CloseStreamSession(sessionHandle);
}
#ifdef __FRC_ROBORIO__
void CANComm_ReadStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status) {
	HAL_CAN_ReadStreamSession(sessionHandle, messages, messagesToRead, messagesRead, status);
}
#else
void CANComm_ReadStreamSession(uint32_t sessionHandle, canframe_t *messages,
		uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status) {
	struct HAL_CANStreamMessage msgs[NUM_CAN_MESSAGES_IN_BATCH];
	uint32_t remainingMessagesToRead = messagesToRead;

	uint32_t currCANStreamMessageIndex = 0;
	while (remainingMessagesToRead > 0) {
		uint32_t currMessagesToRead =
				(remainingMessagesToRead > NUM_CAN_MESSAGES_IN_BATCH) ?
						NUM_CAN_MESSAGES_IN_BATCH : remainingMessagesToRead;
		remainingMessagesToRead -= currMessagesToRead;
		uint32_t currMessagesRead = 0;

		HAL_CAN_ReadStreamSession(sessionHandle, msgs, currMessagesToRead,
				&currMessagesRead, status);
		if (*status != 0) {
			*messagesRead = currCANStreamMessageIndex;
			return;
		}

		if (currMessagesRead > 0) {
			for (uint32_t i = 0; i < currMessagesRead; i++) {
				// TODO:  can the ANDing of 0x1FFFFFFF be removed?
				if (currCANStreamMessageIndex < messagesToRead) {
					messages[currCANStreamMessageIndex].arbID = msgs[i].messageID
							& 0x1FFFFFFF;
					if (msgs[i].dataSize > 8) {
						printf("Error:  CAN Frame Data Size limited to 8 in VMX-pi Platform_mau CANbus_ReceiveFrame().\n");
						msgs[i].dataSize = 8;
					}
					std::memcpy(messages[currCANStreamMessageIndex].data,
							msgs[i].data, msgs[i].dataSize);
					messages[currCANStreamMessageIndex].dlc = msgs[i].dataSize;
					messages[currCANStreamMessageIndex].timeStampUs =
							msgs[i].timeStamp;
					//Don't set any flags on toFill for right now (TODO:  Check w/Omar on this part???)
					currCANStreamMessageIndex++;
				} else {
					printf("Over-capacity in VMX-pi Platform_mau CANComm_ReadStreamSession().\n");
				}
			}
		}
	}
	*messagesRead = currCANStreamMessageIndex;
	return;

}
#endif
int32_t CANComm_GetTxSchedulerStatus(void *unusedControlWorld) { // used to be GetControlWord
	return 0;
	// TODO:  Is this the correct behavior????
}

} //namespace can
} //namespace platform
} //namespace phoenix
} //namespace ctre

namespace ctre {
namespace phoenix {
namespace platform {

void SleepUs(int timeUs) {
	std::this_thread::sleep_for(std::chrono::milliseconds(500));	
}

int32_t SimCreate(DeviceType /*type*/, int /*id*/) {
	return 0;
}

int32_t SimDestroy(DeviceType /*type*/, int /*id*/) {
	return phoenix::ErrorCode::NotImplemented;
}
int32_t SimDestroyAll() {
	return phoenix::ErrorCode::NotImplemented;
}

int32_t DisposePlatform() {
	return phoenix::ErrorCode::OK;
}

int32_t StartPlatform() {
	return phoenix::ErrorCode::OK;
}

int32_t SimConfigGet(DeviceType /*type*/, uint32_t /*param*/,
		uint32_t /*valueToSend*/, uint32_t & /*outValueReceived*/,
		uint32_t & /*outSubvalue*/, uint32_t /*ordinal*/, uint32_t /*id*/) {
	return 0;
}

int32_t SimConfigSet(DeviceType /*type*/, uint32_t /*param*/,
		uint32_t /*value*/, uint32_t /*subValue*/, uint32_t /*ordinal*/,
		uint32_t /*id*/) {
	return 0;
}

/**
 * Get a stack trace, ignoring the first "offset" symbols.
 *
 * @param offset The number of symbols at the top of the stack to ignore
 */
std::string GetStackTrace(int /*offset*/) {

	return "GetStackTrace is not implemented.";
}

void ReportError(int isError, int32_t errorCode, int isLVCode,
		const char *details, const char *location, const char * callStack) {
	int printMsg = 1; // Output message to stdout as well as to remove log clients.
	HAL_SendError(isError, errorCode, isLVCode, details, location, callStack,
			printMsg);

}

} // namespace platform
} // namespace phoenix
} // namespace ctre
