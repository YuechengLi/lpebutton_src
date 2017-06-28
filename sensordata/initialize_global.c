/*
 * initialize_global.c
 *
 *  Created on: Aug 5, 2010
 *      Author: root
 */
#include "global_def.h"
#include <string.h>

void initialize_global(option_struct  *global_options) {
	global_options -> ID = 0;
	global_options -> AUTO_OFF_EN = 0;
	global_options -> CAMERA_ON = 1;
	global_options -> LOW_POWER_MODE = 1;
	global_options -> DMIC_ON = 0;
	global_options -> IMU_ON = 1;	
	//global_options -> IMU_SPEED = 5000000; //5MHz
	global_options -> GYRO_FS = 1;//500DPS
	global_options -> ACC_FS = 1;//4G
	global_options -> GPS_ON = 0;

	global_options -> MCU_MODE = 0;
	global_options -> MCU_BARO_ON = 0;
	global_options -> MCU_MOTION_ON = 0;
	global_options -> MCU_LIS_ON = 0;
	global_options -> MCU_OPTIC_ON = 0;
	global_options -> MCU_SAMPLING_RATE = 30;
	global_options -> IMAGE_ENCRYPTION = 0;

	global_options -> RTC_UPDATE = 0;
	strcpy(global_options -> RTC_TIME, "000000000000.00");
	global_options -> RTC_AUTO_CR = 1;
}
