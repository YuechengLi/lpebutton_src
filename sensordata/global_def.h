/*
 * global_def.h
 *
 *  Created on: April 4, 2011
 *      Author: Yaofeng Yue
 */

#ifndef GLOBAL_DEF_H_
#define GLOBAL_DEF_H_

#define MAXSTRING 200

#define PATH  "/media/lpebutton/eButton_Data/"
//"/boot/uboot/Data/"
#define CONFIG_FILE_NAME "/media/lpebutton/app/config"//"/boot/uboot/system/config"

//#define FOLDER_NAME "lcn_obesity"

#define MAX_PIC 400000
#define FOLDER_PIC 4000

typedef struct {
	int AUTO_OFF_EN;
	int ID;
        int CAMERA_ON;
	int IMAGE_ENCRYPTION;
	int LOW_POWER_MODE;
	int DMIC_ON;
	int IMU_ON;
	//int IMU_SPEED;
	int GYRO_FS;
	int ACC_FS;
	int GPS_ON;

	int MCU_MODE;
	int MCU_BARO_ON;
	int MCU_MOTION_ON;
	int MCU_LIS_ON;
	int MCU_OPTIC_ON;
	int MCU_SAMPLING_RATE;

	int RTC_UPDATE;
	char RTC_TIME[MAXSTRING];
	int RTC_AUTO_CR;
 
} option_struct;

void cmd_proc(char *file_name, int argc, char *argv[]);
void initialize_global(option_struct *);
void get_param(option_struct *, char *);

#endif /* GLOBAL_DEF_H_ */
