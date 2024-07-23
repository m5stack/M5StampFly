
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

unsigned int i2creadCount = 0;
unsigned int i2cwriteCount = 0;
unsigned char SPI2C_Buffer[256];

#include <vl53lx_platform.h>
#ifndef SMALL_FOOTPRINT
#include <vl53lx_platform_ipp.h>
#endif
#include <vl53lx_platform_log.h>
#include <vl53lx_api.h>

//#include "stm32xxx_hal.h"
#include <time.h>
#include <math.h>

//#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

#ifdef VL53LX_LOG_ENABLE
#define trace_print(level, ...) VL53LX_trace_print_module_function(VL53LX_TRACE_MODULE_PLATFORM, level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) VL53LX_trace_print_module_function(VL53LX_TRACE_MODULE_NONE, VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif


i2c_cmd_handle_t i2chandle;

//#ifndef HAL_I2C_MODULE_ENABLED
//#warning "HAL I2C module must be enable "
//#endif
//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)



uint8_t _I2CBuffer[256];


#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_NUM              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
//#define I2C_SENSOR_ADDR 0x29

int i2c_master_port = I2C_MASTER_NUM;
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,         // select SDA GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,         // select SCL GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
    .clk_flags = 0,                          // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
};

/* when not customized by application define dummy one */
//#ifndef VL53LX_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
//#   define VL53LX_GetI2cBus(...) (void)0
//#endif
void VL53LX_GetI2cBus(void)
{
    i2chandle = i2c_cmd_link_create();
}

//#ifndef VL53LX_PutI2cBus
///** This macro can be overloaded by user to enforce i2c sharing in RTOS context
// */
//#   define VL53LX_PutI2cBus(...) (void)0
//#endif

void VL53LX_PutI2cBus(void)
{
    i2c_master_stop(i2chandle);
    i2c_master_cmd_begin(i2c_master_port,i2chandle, 1 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2chandle);
}

int vl53lx_i2c_init(void)
{
    int status=0;
    //i2c_driver_delete(i2c_master_port);
    ets_delay_us(100000);
    //delay(100);
    //status = i2c_param_config(i2c_master_port, &conf);
    //status = status|i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    return status;
}

int _I2CWrite(VL53LX_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    
    i2c_master_start(i2chandle);
    status = i2c_master_write_byte(i2chandle, (Dev->i2c_slave_address<<1)|I2C_MASTER_WRITE, I2C_MASTER_ACK);
    status = i2c_master_write(i2chandle, pdata, count, I2C_MASTER_ACK);
    return status;
}

int _I2CRead(VL53LX_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;

    i2c_master_start(i2chandle);
    status = i2c_master_write_byte(i2chandle, (Dev->i2c_slave_address<<1)|I2C_MASTER_READ, I2C_MASTER_ACK);
    if (count>1)
    {
        status = i2c_master_read(i2chandle, pdata, count-1, I2C_MASTER_ACK);
    }
    status = i2c_master_read_byte(i2chandle, pdata+count-1, I2C_MASTER_NACK);
    return status;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, count + 2);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    VL53LX_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, pdata, count);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53LX_PutI2cBus();
    return Status;
}

VL53LX_Error VL53LX_WrByte(VL53LX_DEV Dev, uint16_t index, uint8_t data) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;

    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    VL53LX_PutI2cBus();
    return Status;
}

VL53LX_Error VL53LX_WrWord(VL53LX_DEV Dev, uint16_t index, uint16_t data) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0x00FF;

    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    VL53LX_PutI2cBus();
    return Status;
}

VL53LX_Error VL53LX_WrDWord(VL53LX_DEV Dev, uint16_t index, uint32_t data) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
    _I2CBuffer[3] = (data >> 16) & 0xFF;
    _I2CBuffer[4] = (data >> 8)  & 0xFF;
    _I2CBuffer[5] = (data >> 0 ) & 0xFF;
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 6);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }
    VL53LX_PutI2cBus();
    return Status;
}

VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    uint8_t data;

    Status = VL53LX_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53LX_WrByte(Dev, index, data);
done:
    return Status;
}

VL53LX_Error VL53LX_RdByte(VL53LX_DEV Dev, uint16_t index, uint8_t *data) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if( status_int ){
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, data, 1);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

done:
    VL53LX_PutI2cBus();
    return Status;
}

VL53LX_Error VL53LX_RdWord(VL53LX_DEV Dev, uint16_t index, uint16_t *data) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);

    if( status_int ){
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }

done:
    VL53LX_PutI2cBus();
    *data = ((uint16_t)_I2CBuffer[0]<<8)+ (uint16_t)_I2CBuffer[1];
    return Status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_DEV Dev, uint16_t index, uint32_t *data) {
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53LX_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
        goto done;
    }

done:
    VL53LX_PutI2cBus();
    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];
    return Status;
}

VL53LX_Error VL53LX_GetTickCount(
	VL53LX_DEV Dev,
	uint32_t *ptick_count_ms)
{

    /* Returns current tick count in [ms] */

	VL53LX_Error status  = VL53LX_ERROR_NONE;

	//*ptick_count_ms = timeGetTime();
	*ptick_count_ms = 0;

#ifdef VL53LX_LOG_ENABLE
	trace_print(
		VL53LX_TRACE_LEVEL_DEBUG,
		"VL53LX_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);
#endif

	return status;
}











#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
	VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)














VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;
	
	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GetTimerFrequency: Freq : %dHz\n", *ptimer_freq_hz);
	return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms){
	(void)pdev;
    ets_delay_us(wait_ms*1000);
	//delay(wait_ms);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us){
	(void)pdev;
    ets_delay_us(wait_us);   
	//delay(wait_us/1000);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{

	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms = 0;
	uint32_t     current_time_ms = 0;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint8_t      trace_functions = VL53LX_TRACE_FUNCTION_NONE;
#endif

	char   register_name[VL53LX_MAX_STRING_LENGTH];

    /* look up register name */
#ifdef PAL_EXTENDED
	VL53LX_get_register_name(
			index,
			register_name);
#else
	VL53LX_COPYSTRING(register_name, "");
#endif

	/* Output to I2C logger for FMT/DFT  */

    /*trace_i2c("WaitValueMaskEx(%5d, 0x%04X, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, index, value, mask, poll_delay_ms); */
    trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
    		     timeout_ms, register_name, value, mask, poll_delay_ms);

	/* calculate time limit in absolute time */

	 VL53LX_GetTickCount(pdev, &start_time_ms);

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53LX_LOG_ENABLE
	trace_functions = VL53LX_get_trace_functions();
	VL53LX_set_trace_functions(VL53LX_TRACE_FUNCTION_NONE);
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53LX_ERROR_NONE) &&
		   (polling_time_ms < timeout_ms) &&
		   (found == 0)) {

		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_RdByte(
							pdev,
							index,
							&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53LX_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53LX_WaitMs(
					pdev,
					poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53LX_GetTickCount(pdev, &current_time_ms);
		polling_time_ms = current_time_ms - start_time_ms;

	}

#ifdef VL53LX_LOG_ENABLE
	/* Restore function logging */
	VL53LX_set_trace_functions(trace_functions);
#endif

	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;
}




