#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "global_def.h"


void get_param(option_struct *options, char *filename)
{
	FILE *RDconfig;
	char cmdstr[MAXSTRING];
	char optstr[MAXSTRING];

	int AUTO_OFF_EN_val;
	int ID_val;
        int CAMERA_ON_val;
	int IMAGE_ENCRYPTION_val;
	int LOW_POWER_MODE_val;
	int DMIC_ON_val;
	int IMU_ON_val;
	//int IMU_SPEED;
	int GYRO_FS_val;
	int ACC_FS_val;
	int GPS_ON_val;

	int MCU_MODE_val;
	int MCU_BARO_ON_val;
	int MCU_MOTION_ON_val;
	int MCU_LIS_ON_val;
	int MCU_OPTIC_ON_val;
	int MCU_SAMPLING_RATE_val;

	int RTC_UPDATE_val;
	char RTC_TIME_val[MAXSTRING];
	int RTC_AUTO_CR_val;


	/* Initialize global options */
	initialize_global(options);

	//if (strcmp(filename,CONFIG_FILE_NAME)==0)
	{
		RDconfig=fopen(filename,"r");
		/*if (RDconfig!=NULL) //copy backup config file
		{
			//system("mkdir -p /media/lpebutton/app");
			system("cp /boot/uboot/system/config /media/lpebutton");

			RDconfig=fopen(filename,"r");
		}	*/	

		printf("Use global control in the configuration file!\n");

  		while(!feof(RDconfig)) 
		{
			fgets(cmdstr,MAXSTRING,RDconfig);//MAXSTRING
	    		if(cmdstr[0]!='#' && cmdstr[0]!='\n' && cmdstr[0]!='\0') 
			{
		
	      		      sscanf(cmdstr,"%s",optstr);

			    	/*******************************
				 Get Model Global Parameters
				*****************************/
				if(strcasecmp("ID",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&ID_val);//options->ID);
					printf("ID = %d\n", ID_val);//options->ID);
			      	}
				if(strcasecmp("AUTO_OFF_EN",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&AUTO_OFF_EN_val);
			      	}
				if(strcasecmp("CAMERA_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&CAMERA_ON_val);
			      	}
 				if(strcasecmp("IMU_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&IMU_ON_val);
			      	}
				/*if(strcasecmp("IMU_SPEED",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&options->IMU_SPEED);
			      	}*/
				if(strcasecmp("GYRO_FS",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&GYRO_FS_val);
			      	}
				if(strcasecmp("ACC_FS",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&ACC_FS_val);
			      	}
 				if(strcasecmp("DMIC_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&DMIC_ON_val);
			      	}
				if(strcasecmp("GPS_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&GPS_ON_val);
				}
 				if(strcasecmp("IMAGE_ENCRYPTION",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&IMAGE_ENCRYPTION_val);
			      	}
			      	if(strcasecmp("LOW_POWER_MODE",optstr)==0) {
					printf("found LOW_POWER_MODE a is:");
					sscanf(cmdstr,"%*s %d",&LOW_POWER_MODE_val);
					printf(" %d\n",LOW_POWER_MODE_val);
			      	}
 				if(strcasecmp("MCU_MODE",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&MCU_MODE_val);
			      	}
 				if(strcasecmp("MCU_BARO_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&MCU_BARO_ON_val);
			      	}
 				if(strcasecmp("MCU_MOTION_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&MCU_MOTION_ON_val);
			      	}
 				if(strcasecmp("MCU_LIS_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&MCU_LIS_ON_val);
			      	}
 				if(strcasecmp("MCU_OPTIC_ON",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&MCU_OPTIC_ON_val);
			      	}
 				if(strcasecmp("MCU_SAMPLING_RATE",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&MCU_SAMPLING_RATE_val);
			      	}			      	
				if(strcasecmp("RTC_UPDATE",optstr)==0) {
					sscanf(cmdstr,"%*s %d",&RTC_UPDATE_val);
			      	}
	
			      	if(strcasecmp("RTC_TIME",optstr)==0) {
					sscanf(cmdstr,"%*s %s", &RTC_TIME_val);			
			      	}
			      	if(strcasecmp("RTC_AUTO_CR",optstr)==0) {
					sscanf(cmdstr,"%*s %d", &RTC_AUTO_CR_val);			
			      	}
			}
			
			//printf("found IMAGE_ENCRYPTION b is %d\n",options->IMAGE_ENCRYPTION);
			//printf("found LOW_POWER_MODE b is %d\n",options->LOW_POWER_MODE);
		}

		fclose(RDconfig);
		
		options->ID = ID_val;
		options->AUTO_OFF_EN = AUTO_OFF_EN_val;
		options->CAMERA_ON = CAMERA_ON_val;
		options->LOW_POWER_MODE = LOW_POWER_MODE_val;
		options->DMIC_ON = DMIC_ON_val;
		options->IMU_ON = IMU_ON_val;	
		//global_options->IMU_SPEED = 5000000; //5MHz
		options->GYRO_FS = GYRO_FS_val;//500DPS
		options->ACC_FS = ACC_FS_val;//4G
		options->GPS_ON = GPS_ON_val;

		options->MCU_MODE = MCU_MODE_val;
		options->MCU_BARO_ON = MCU_BARO_ON_val;
		options->MCU_MOTION_ON = MCU_MOTION_ON_val;
		options->MCU_LIS_ON = MCU_LIS_ON_val;
		options->MCU_OPTIC_ON = MCU_OPTIC_ON_val;
		options->MCU_SAMPLING_RATE = MCU_SAMPLING_RATE_val;
		options->IMAGE_ENCRYPTION = IMAGE_ENCRYPTION_val;

		options -> RTC_UPDATE = RTC_UPDATE_val;
		strcpy(options -> RTC_TIME,  RTC_TIME_val);
		options -> RTC_AUTO_CR = RTC_AUTO_CR_val;

		printf("found IMAGE_ENCRYPTION c is %d\n",options->IMAGE_ENCRYPTION);
		printf("found LOW_POWER_MODE c is %d\n",options->LOW_POWER_MODE);
	}
#if 1
	printf("AUTO_OFF_EN is %d\n", options->AUTO_OFF_EN);
	printf("eButton ID is %d\n",options->ID);
	printf("LOW_POWER_MODE is %d\n",options->LOW_POWER_MODE);
	printf("CAMERA_ON is %d\n",options->CAMERA_ON);
	printf("DMIC_ON is %d\n",options->DMIC_ON);
	printf("GPS_ON is %d\n",options->GPS_ON);
        printf("IMU_ON is %d\n",options->IMU_ON);
	//printf("IMU_SPEED is %d (Hz)\n",options->IMU_SPEED);
	printf("GYRO_FS is %d\n",options->GYRO_FS);
	printf("ACC_FS is %d\n",options->ACC_FS);
	printf("MCU_MODE is %d\n",options->MCU_MODE);
	printf("MCU_BARO_ON is %d\n",options->MCU_BARO_ON);
	printf("MCU_MOTION_ON is %d\n",options->MCU_MOTION_ON);
	printf("MCU_LIS_ON is %d\n",options->MCU_LIS_ON);
	printf("MCU_OPTIC_ON is %d\n",options->MCU_OPTIC_ON);
	printf("MCU_SAMPLING_RATE is %d\n",options->MCU_SAMPLING_RATE);
        printf("IMAGE_ENCRYPTION is %d\n",options->IMAGE_ENCRYPTION);
	printf("RTC_UPDATE is %d\n",options->RTC_UPDATE);
	printf("RTC_TIME is %s\n",options->RTC_TIME);
	printf("RTC_AUTO_CR is %d\n",options->RTC_AUTO_CR);
#endif
}
