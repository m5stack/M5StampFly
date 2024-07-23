
/* 
* This file is part of VL53LX Platform
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/




#include "stm32xxx_hal.h"

























#ifndef _VL53LX_PLATFORM_USER_DATA_H_
#define _VL53LX_PLATFORM_USER_DATA_H_

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#include "vl53lx_def.h"

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct {
	VL53LX_DevData_t   Data;
	/*!< Low Level Driver data structure */
    uint8_t   i2c_slave_address;
	uint8_t   comms_type;
	uint16_t  comms_speed_khz;
	I2C_HandleTypeDef *I2cHandle;
	uint8_t   I2cDevAddr;
	int     Present;
	int 	Enabled;
	int LoopState;
	int FirstStreamCountZero;
	int 	Idle;
	int		Ready;
	uint8_t RangeStatus;
	FixPoint1616_t SignalRateRtnMegaCps;
	VL53LX_DeviceState   device_state;  /*!< Device State */
} VL53LX_Dev_t;

typedef VL53LX_Dev_t* VL53LX_DEV;

/**
 * @def VL53LXDevDataGet
 * @brief Get ST private structure @a VL53LX_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like VL53L1DevDataGet(FilterData.field)[i] or
 * VL53L1DevDataGet(FilterData.MeasurementIndex)++
 */
#define VL53LXDevDataGet(Dev, field) (Dev->Data.field)


/**
 * @def VL53LXDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53LX_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define VL53LXDevDataSet(Dev, field, data) ((Dev->Data.field) = (data))











#define PALDevDataGet(Dev, field) (Dev->Data.field)












#define PALDevDataSet(Dev, field, VL53LX_PRM_00005) (Dev->Data.field)=(VL53LX_PRM_00005)









#define VL53LXDevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)








#define VL53LXDevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)



#ifdef __cplusplus
}
#endif

#endif

