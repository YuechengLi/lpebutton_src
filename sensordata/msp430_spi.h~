#ifndef MSP430_SPI_H_
#define MSP430_SPI_H_


typedef struct{

	unsigned char rtc_sec;
	unsigned char rtc_min;
	unsigned char rtc_hr;
	unsigned char rtc_day;
	unsigned char rtc_mon;
	unsigned char rtc_yr;
	unsigned char msp430_mode;
		
	unsigned char msp430_BARO_on;
	unsigned char msp430_MOTION_on;
	unsigned char msp430_LIS_MAG_on;
	unsigned char msp430_OPTIC_on;
	unsigned char msp430_SAMPLING_RATE;

}  MSP430_CFG;


typedef struct{

	short MPU_ACC_mG[3];
	short MPU_GYRO_10[3];
	short MPU_MAG_10[3];

	short LIS_Mag[3];

	unsigned short BEM280_humidity_100percent;
	short BEM280_temp_Cdegree;
	unsigned short BEM280_pressure_10Pa;
	
	  
	unsigned short visiblelight;
	unsigned short IRlight;
	unsigned short uv_index;
	unsigned short proximity;

	unsigned char hour;
	unsigned char min;
	unsigned char sec;

} MSP430_DATA;


void MSP430_init_config(unsigned int spi_speed_set, MSP430_CFG msp430_cfg_data);
unsigned char  MSP430_sensor_rd(MSP430_DATA *msp430_data);
unsigned char MSP430_read_bt_status();//unsigned int numFrames);
void MSP430_write_bt(unsigned char *pImg, unsigned char num);


#endif
