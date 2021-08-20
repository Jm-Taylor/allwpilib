/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Power.h"

#include "MauInternal.h"

namespace hal {
    namespace init {
        void InitializePower() {}
    }
}

extern "C" {
    double HAL_GetVinVoltage(int32_t* status) {
        float default_voltage = 0.0f;
        float voltage = default_voltage;
    	mau::vmxPower->GetSystemVoltage(voltage, status);
        return (double)mau::DefaultOnBoardCommError(status, voltage, default_voltage);
    }

    double HAL_GetVinCurrent(int32_t* status) {
    	return 1.0;  // VMX-pi does not provide Vin current monitoring
    }

    double HAL_GetUserVoltage6V(int32_t* status) {
    	return 0.0; // VMX-pi has no 6V user rail
    }

    double HAL_GetUserCurrent6V(int32_t* status) {
    	return 0.0; // VMX-pi has no 6V user rail
    }

    HAL_Bool HAL_GetUserActive6V(int32_t* status) {
        return false;  // VMX-pi has no 6V user rail
    }

    int32_t HAL_GetUserCurrentFaults6V(int32_t* status) {
       return 0; // VMX-pi has no 6V user rail, so there can be no faults on it.
    }

    double HAL_GetUserVoltage5V(int32_t* status) {
    	double voltage = 5.0;
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			voltage = 0.0;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);

        return voltage;
    }

    double HAL_GetUserCurrent5V(int32_t* status) {
    	double current = 0.5; // Max user rail current on VMX-pi
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			current = 0.0;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);

        return current;
    }

    HAL_Bool HAL_GetUserActive5V(int32_t* status) {
    	bool active = true;
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			active = false;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);

        return active;
    }

    // TODO:  Implement Fault Counting on VMX-pi?
    int32_t HAL_GetUserCurrentFaults5V(int32_t* status) {
    	int32_t num5Vfaults = 0;
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			num5Vfaults = 1;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);
        
        return num5Vfaults;
    }

    double HAL_GetUserVoltage3V3(int32_t* status) {
    	double voltage = 3.3;
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			voltage = 0.0;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);

        return voltage;
    }

    double HAL_GetUserCurrent3V3(int32_t* status) {
    	double current = 0.5; // Max user rail current on VMX-pi
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			current = 0.0;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);
        
        return current;
    }

    HAL_Bool HAL_GetUserActive3V3(int32_t* status) {
    	bool active = true;
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			active = false;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);
        
        return active;
    }

    // TODO:  Implement Fault Counting on VMX-pi?
    int32_t HAL_GetUserCurrentFaults3V3(int32_t* status) {
    	int32_t num5Vfaults = 0;
    	bool overcurrent = false;
    	if (mau::vmxPower->GetOvercurrent(overcurrent, status)) {
    		if (overcurrent) {
    			num5Vfaults = 1;
    		}
    	}
        mau::ClearBoardCommErrorStatus(status);
        
        return num5Vfaults;
    }
}  // extern "C"
