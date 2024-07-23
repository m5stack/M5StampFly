/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/**
 * @file   VL53LX_platform_log.c
 *
 * @brief  Code function definitions for EwokPlus25 Platform Logging Layer
 */

#include <stdio.h>    // sprintf(), vsnprintf(), printf()
#include <string.h>
#include <stdarg.h>
#include <malloc.h>
#include <vl53lx_platform_log.h>
#include <vl53lx_platform_user_config.h>


#ifdef VL53LX_LOG_ENABLE

	char * _trace_filename = NULL;
	FILE *_tracefile = NULL;

	uint32_t _trace_level     = VL53LX_TRACE_LEVEL_ALL;
	uint32_t _trace_modules   = VL53LX_TRACE_MODULE_ALL;
	uint32_t _trace_functions = VL53LX_TRACE_FUNCTION_ALL;

	int8_t VL53LX_trace_config(
		char *filename,
		uint32_t modules,
		uint32_t level,
		uint32_t functions)
	{
		int8_t status = 0;

	//    // Something in the commented out code below causes ncsim to crash!
	//
	//    if (((_trace_filename != NULL) && (_trace_filename != filename)) ||strcmp(filename,"")==0)
	//    {
	//        if ( _tracefile != NULL )
	//        {
	//            fclose(_tracefile);
	//            _tracefile = NULL;
	//        }
	//        free(_trace_filename);
	//        _trace_filename = NULL;
	//    }
	//

		if (((filename != NULL) && (_tracefile == NULL)) && strcmp(filename,""))
		{
			_tracefile = fopen(filename, "w+");

			//TODO: Add time and header banner to the log file to indicate we've just opened a new log session

			if ( _tracefile != NULL )
			{
				_trace_filename = (char*)malloc((strlen(filename) + 1) * sizeof(char));
				strcpy(_trace_filename, filename);
			} 
			else
			{
				printf("VL53LX_trace_config(): failed to open log file (%s)\n", filename);
				status = 1;
			}
		}

		_trace_modules   = modules;
		_trace_level     = level;
		_trace_functions = functions;

		return status;
	}

	void VL53LX_trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
	{
		if ( ((level <=_trace_level) && ((module & _trace_modules) > 0))
			|| ((function & _trace_functions) > 0) )
		{
			va_list arg_list;
			char message[VL53LX_MAX_STRING_LENGTH];

			va_start(arg_list, format);
			vsnprintf(message, VL53LX_MAX_STRING_LENGTH-1, format, arg_list); /*lint !e534  ignore return*/
			va_end(arg_list);

			if (_tracefile != NULL)
			{
				fprintf(_tracefile, message); /*lint !e592  ignore format specifier*/ 
			}
			else
			{
				CH_printf(CH_ID_TRACE, message); /*lint !e592  ignore format specifier*/
			}

	//        if (_tracefile != NULL)
	//            fprintf(_tracefile, message);
	//        else
	//            printf(message);
		}  /*lint !e438  ignore issues with arg_list*/ 
	}


	uint32_t VL53LX_get_trace_functions(void)
	{
		return _trace_functions;
	}


	void VL53LX_set_trace_functions(uint32_t function)
	{
		_trace_functions = function;
	}


	uint32_t VL53LX_clock(void)
	{
		/* Returns current tick count in [ms] */
		uint32_t tick_count_ms = (uint32_t)clock();
		return tick_count_ms;
	}
#endif // VL53LX_LOG_ENABLE
