#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "mpu9250_spi.h"

#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0])

const char *mpu9250_spi_DEV= "/dev/spidev32765.0";


# define RX_LEN 48

struct spi_ioc_transfer xfer[2];
unsigned char buf[8];
unsigned char rxbuf [ RX_LEN ];

int mpuspifd;
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;
unsigned char lsb;

#define MAX_SPI_SPEED 5000000  //5MHz

/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
static const int gyro_scale_9250[] = {133090, 266181, 532362, 1064724};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
static const int accel_scale[] = {598, 1196, 2392, 4785};

int  asax, asay, asaz;
float Magnetometer_ASA[3];

float gyro_scale;
float acc_scale;

//MAG_DATA mage_data;
//MPU9250_sensor mpu9250_data;

static const struct inv_mpu9250_reg_map reg_set_9250 = {
	.sample_rate_div	= INV_MPU9250_REG_SAMPLE_RATE_DIV,
	.lpf                    = INV_MPU9250_REG_CONFIG,
	.user_ctrl              = INV_MPU9250_REG_USER_CTRL,
	.fifo_en                = INV_MPU9250_REG_FIFO_EN,
	.i2c_mst_ctrl 		= INV_MPU9250_REG_I2C_MST_CTRL ,
	.i2c_slv0_addr		= INV_MPU9250_REG_I2C_SLV0_ADDR,
	.i2c_slv0_reg 		= INV_MPU9250_REG_I2C_SLV0_REG ,
	.i2c_slv0_ctrl		= INV_MPU9250_REG_I2C_SLV0_CTRL,
	.i2c_slv0_DO		= INV_MPU9250_REG_I2C_SLV0_DO,
	.i2c_ext_sens_data0	= INV_MPU9250_EXT_SENS_DATA_00,
	.i2c_ext_sens_data1	= INV_MPU9250_EXT_SENS_DATA_01,
	.i2c_ext_sens_data2	= INV_MPU9250_EXT_SENS_DATA_02,
	.i2c_ext_sens_data3	= INV_MPU9250_EXT_SENS_DATA_03,
	.i2c_ext_sens_data4	= INV_MPU9250_EXT_SENS_DATA_04,
	.i2c_ext_sens_data5	= INV_MPU9250_EXT_SENS_DATA_05,
	.i2c_ext_sens_data6	= INV_MPU9250_EXT_SENS_DATA_06,
	.i2c_ext_sens_data7	= INV_MPU9250_EXT_SENS_DATA_07,
	.gyro_config            = INV_MPU9250_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU9250_REG_ACCEL_CONFIG,
	.accl_config2           = INV_MPU9250_REG_ACCEL_CONFIG2,
	.fifo_count_h           = INV_MPU9250_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU9250_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU9250_REG_RAW_GYRO,
	.raw_accl               = INV_MPU9250_REG_RAW_ACCEL,
	.temperature            = INV_MPU9250_REG_TEMPERATURE,
	.raw_asa		= AK8963_ASAX,
	.raw_mag		= AK8963_HXL,
	.int_pin_cfg		= INV_MPU9250_REG_INT_PIN_CFG,
	.int_enable             = INV_MPU9250_REG_INT_ENABLE,
	.int_status		= INV_MPU9250_REG_INT_STATUS,
	.pwr_mgmt_1             = INV_MPU9250_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU9250_REG_PWR_MGMT_2,
	.who_am_I		= INV_MPU9250_REG_WHO_AM_I,
	.accl_offset		= INV_MPU9250_REG_XA_OFFSET_H,
	.gyro_offset		= INV_MPU9250_REG_XG_OFFSET_H,
};

struct inv_mpu9250_chip_config chip_config_9250 = {
	.fsr = INV_MPU9250_FSR_500DPS,//INV_MPU9250_FSR_2000DPS,
	.lpf = INV_MPU9250_FILTER_184HZ,//INV_MPU9250_FILTER_20HZ,
	.fifo_rate = INV_MPU9250_INIT_FIFO_RATE,
	.gyro_fifo_enable = 0,
	.accl_fifo_enable = 0,
	.accl_fs = INV_MPU9250_FS_04G,//INV_MPU9250_FS_02G,
	//.gyro_fs = INV_MPU9250_FSR_500DPS,
};


void mag_mode_switch(unsigned char target_mode)
{
	unsigned int  status, i;

	//16bits, PWD mode
	do{
		
		WriteReg(reg_set_9250.i2c_slv0_addr, AK8963_I2C_ADDR|WRITE_FLAG);
		WriteReg(reg_set_9250.i2c_slv0_reg, AK8963_CNTL1);
		WriteReg(reg_set_9250.i2c_slv0_DO, MAGNETIC_16bits|target_mode);//// Register value to POWER DOWN mode and in 16bit
		WriteReg(reg_set_9250.i2c_slv0_ctrl, 0x80);//Enable I2C and set 1 byte
		for (i = 0; i < 10000; i++) {   
			asm("    nop");
		}

		//read
		WriteReg(reg_set_9250.i2c_slv0_addr, AK8963_I2C_ADDR|READ_FLAG);
		WriteReg(reg_set_9250.i2c_slv0_reg, AK8963_CNTL1);
		WriteReg(reg_set_9250.i2c_slv0_ctrl, 0x81); //Read 1 byte from the magnetometer
		for (i = 0; i < 3000; i++) {   
			asm("    nop");
		}
		ReadReg(reg_set_9250.i2c_ext_sens_data0, 1);//regmap_read(st->map, st->reg->i2c_ext_sens_data0, &status);
		status = rxbuf[0];

	}while((status&0x1f)!=(MAGNETIC_16bits|target_mode));
	
	//printf("\n\n\\******** mag status changed: 0x%x ********\\\n\n", status);

}



void WriteReg(unsigned char WriteAddr, unsigned char WriteData )
{
    	unsigned short status;
	int i;
    
	buf[0] = (WriteAddr&0x7F);
	buf[1] = WriteData;
	xfer[0].tx_buf = (unsigned long)buf;
	xfer[0].len = 2;
	xfer[0].speed_hz = spi_speed;
	status = ioctl(mpuspifd, SPI_IOC_MESSAGE(1), xfer);
    	if (status < 0) {
                perror("WriteReg");
        }

	//Has to wait enough time for successful configuration of MPU9250
	for (i = 0; i < 100; i++) {   
		asm("    nop");
	}
}

void ReadReg(unsigned char ReadAddr, int nbytes)
{

    	unsigned short i, status;

    	//    * tx_buf - a pointer to the data to be transferred
  	//    * rx_buf - a pointer to storage for received data
  	//    * len - length in bytes of tx and rx buffers
  	//    * speed_hz - the clock speed, in Hz
  	//    * delay_usecs - delay between last bit and deassertion of CS
  	//    * bits_per_word - override global word length for this transfer
  	//    * cs_change - strobe chip select between transfers?

	memset(rxbuf, 0, nbytes);

	buf[0] = (ReadAddr|0x80);
	xfer[0].tx_buf = (unsigned long) buf;
	xfer[0].len = 1;
	xfer[0].speed_hz = spi_speed;
	xfer[1].rx_buf = (unsigned long) rxbuf;
	xfer[1].len = nbytes;
	xfer[1].speed_hz = spi_speed;
	status = ioctl(mpuspifd, SPI_IOC_MESSAGE(2), xfer);

        if (status < 0) {
                perror("READREG");
                return;
        }


	//printf("rxbuf: %d,%d,%d\n", rxbuf[0],rxbuf[1],rxbuf[2]);
}

static int open_mpuspi(unsigned int spi_speed_set)
{
	int status_value = 0;

	//open spi device
	mpuspifd = open(mpu9250_spi_DEV, O_RDWR);
	if (mpuspifd<=0) { 
		perror(mpu9250_spi_DEV);
		exit(1);
	}


    	//----- SET SPI MODE -----
    	//SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    	//SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    	//SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    	//SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    	spi_mode = SPI_MODE_0;
    
    	//----- SET BITS PER WORD -----
    	spi_bitsPerWord = 8;
    
    	//----- SET SPI BUS SPEED -----
    	spi_speed = spi_speed_set;		

	lsb = 0;

    	status_value = ioctl(mpuspifd, SPI_IOC_WR_MODE, &spi_mode);
    	if(status_value < 0)
    	{
        	perror("Could not set SPIMode (WR)...ioctl fail");
        	exit(1);
   	}
	//printf("WR_MODE: %d\n", spi_mode);

    	status_value = ioctl(mpuspifd, SPI_IOC_RD_MODE, &spi_mode);
    	if(status_value < 0)
    	{
      		perror("Could not set SPIMode (RD)...ioctl fail");
      		exit(1);
    	}
	//printf("RD_MODE: %d\n", spi_mode);

	status_value = ioctl(mpuspifd, SPI_IOC_RD_LSB_FIRST, &lsb);
	if ( status_value < 0) 
	{
                perror("SPI rd_lsb_fist");
                exit(1);
        }

    	status_value = ioctl(mpuspifd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
    	if(status_value < 0)
    	{
      		perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      		exit(1);
    	}
	//printf("WR_BITS_PER_WORD: %d\n", spi_bitsPerWord);

    	status_value = ioctl(mpuspifd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
    	if(status_value < 0)
    	{
     		perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      		exit(1);
    	}
	//printf("WR_BITS_PER_WORD: %d\n", spi_bitsPerWord);

    	status_value = ioctl(mpuspifd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    	if(status_value < 0)
    	{
      		perror("Could not set SPI speed (WR)...ioctl fail");
      		exit(1);
    	}
	//printf("Max WR speed: %dHz\n", spi_speed);


    	status_value = ioctl(mpuspifd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    	if(status_value < 0)
    	{
      		perror("Could not set SPI speed (RD)...ioctl fail");
      		exit(1);
    	}
	//printf("Max RD speed: %dHz\n", spi_speed);

	return status_value;

}

/**
 *  inv_mpu9250_init_config() - Initialize hardware, disable FIFO.
 *
 *  Initial configuration:
 *  FSR: ± 500DPS
 *  DLPF: 184Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
void inv_mpu9250_init_config(unsigned int spi_speed_set, unsigned char gyro_fs, unsigned char acc_fs)
{
	int i,mpufd;
	unsigned char d;
	char wr_buf[]={0xff,0x00,0x1f,0x0f};
	char rd_buf[10];
	unsigned char mpu_ID;
	unsigned char MAG_ID;	
	unsigned int  status;

	//open spi device
	if(spi_speed_set>MAX_SPI_SPEED) //
		spi_speed_set = MAX_SPI_SPEED;
	if(open_mpuspi(spi_speed_set)<0)
	{
		perror("Error - Could not open SPI");
        	exit(1);
	}
		

	//check ID
	ReadReg(reg_set_9250.who_am_I, 1);
	mpu_ID = rxbuf[0];

	printf("\n\\******** MPU-9250 ID: 0x%x ********\\\n", mpu_ID);


	//initial 500dps
	d = ( (gyro_fs) << INV_MPU9250_GYRO_CONFIG_FSR_SHIFT);// 
	WriteReg(reg_set_9250.gyro_config, d);
	chip_config_9250.fsr = gyro_fs; 

	//initial 4G
	d = ( (acc_fs) << INV_MPU9250_ACCL_CONFIG_FSR_SHIFT);//
	WriteReg(reg_set_9250.accl_config, d);
	chip_config_9250.accl_fs = acc_fs; 

	//184Hz
	d = chip_config_9250.lpf;//INV_MPU9250_FILTER_20HZ;
	WriteReg(reg_set_9250.lpf, d);//regmap_write(st->map, st->reg->lpf, d);

	//initial 50HZ sampling rate
	//d = INV_MPU9250_ONE_K_HZ / INV_MPU9250_INIT_FIFO_RATE - 1;
	//WriteReg(reg_set_9250.sample_rate_div, d);

	///////////////////////////////////////////////////////////////////////////////////
	// I2C Master mode
	WriteReg(reg_set_9250.user_ctrl, 0x20);

	//  I2C configuration multi-master  IIC 400KHz
	WriteReg(reg_set_9250.i2c_mst_ctrl, 0x0D);

	//reset mag
	WriteReg(reg_set_9250.i2c_slv0_addr, AK8963_I2C_ADDR|WRITE_FLAG);
	WriteReg(reg_set_9250.i2c_slv0_reg, AK8963_CNTL2);
	WriteReg(reg_set_9250.i2c_slv0_DO, 0x01);// Reset AK8963
	WriteReg(reg_set_9250.i2c_slv0_ctrl, 0x80);//Enable I2C and set 1 byte
	//msleep(10);
	for (i = 0; i < 1000000; i++) {   
		asm("    nop");
	}

	//READ ID
	WriteReg(reg_set_9250.i2c_slv0_addr, AK8963_I2C_ADDR|READ_FLAG);
	WriteReg(reg_set_9250.i2c_slv0_reg, AK8963_WIA);
	WriteReg(reg_set_9250.i2c_slv0_ctrl, 0x81); //Read 1 byte from the magnetometer
	for (i = 0; i < 1000000; i++) {   
		asm("    nop");
	}	
	ReadReg(reg_set_9250.i2c_ext_sens_data0, 1);
	MAG_ID = rxbuf[0];
	printf("\\******** MAG ID: 0x%x ********\\\n", MAG_ID);

	//16bits, PWD mode
	mag_mode_switch(MAGNETIC_PWD);
	mag_mode_switch(MAGNETIC_FUSE_ROM);

	////Fuse Rom ACCESS MODE
	//read
	WriteReg(reg_set_9250.i2c_slv0_addr, AK8963_I2C_ADDR|READ_FLAG);
	WriteReg(reg_set_9250.i2c_slv0_reg, AK8963_ASAX);
	WriteReg(reg_set_9250.i2c_slv0_ctrl, 0x83); //Read 3 byte from the magnetometer
	for (i = 0; i < 3000000; i++) {   
		asm("    nop");
	}
	ReadReg(reg_set_9250.i2c_ext_sens_data0, 3);
	asax = rxbuf[0];
	asay = rxbuf[1];
	asaz = rxbuf[2];

	printf("\\******** asa-(x,y,z): (%d,%d,%d) ********\\\n", asax,asay,asaz);

	//mage_data.Magnetometer_ASA[0] = ((asax-128)/256+1)*0.15;//Magnetometer_Sensitivity_Scale_Factor_16bits;
	//mage_data.Magnetometer_ASA[1] = ((asay-128)/256+1)*0.15;//Magnetometer_Sensitivity_Scale_Factor_16bits;
	//mage_data.Magnetometer_ASA[2] = ((asaz-128)/256+1)*0.15;//Magnetometer_Sensitivity_Scale_Factor_16bits;
	Magnetometer_ASA[0] = (((float)asax-128.0)/256.0+1.0)*0.15;//Magnetometer_Sensitivity_Scale_Factor_16bits;
	Magnetometer_ASA[1] = (((float)asay-128.0)/256.0+1.0)*0.15;//Magnetometer_Sensitivity_Scale_Factor_16bits;
	Magnetometer_ASA[2] = (((float)asaz-128.0)/256.0+1.0)*0.15;//Magnetometer_Sensitivity_Scale_Factor_16bits;

	printf("\\******** asa-(fx,fy,fz): (%f,%f,%f) ********\\\n", Magnetometer_ASA[0],Magnetometer_ASA[1],Magnetometer_ASA[2]);

	////return to power down mode first, then CONT2 mode
	mag_mode_switch(MAGNETIC_PWD);

	//put magnetometer into CONT2 mode
	mag_mode_switch(MAGNETIC_CONT2_MEASUREMENT);

	////////////////////////////////////////////////////////////////////////////////

	
	if(chip_config_9250.fsr==0)
		gyro_scale = MPU9250G_250dps;//dps/LSB
	else if(chip_config_9250.fsr==1)
		gyro_scale = MPU9250G_500dps;
	else if(chip_config_9250.fsr==2)
		gyro_scale = MPU9250G_1000dps;
	else
		gyro_scale = MPU9250G_2000dps;

	if(chip_config_9250.accl_fs==0)
		acc_scale = MPU9250A_2g;//g/LSB
	else if(chip_config_9250.accl_fs==1)
		acc_scale = MPU9250A_4g;//g/LSB
	else if(chip_config_9250.accl_fs==2)
		acc_scale = MPU9250A_8g;//g/LSB
	else
		acc_scale = MPU9250A_16g;//g/LSB

	//printf("g_fs: %f, acc_fs: %f\n", gyro_scale, acc_scale);

}


void Motion_MPU9250_REGs_rd(MPU9250_sensor *MPU9250_DATA)
{
	int i;
	float data;



	//read mpu9250
        ReadReg(reg_set_9250.raw_accl, 14);
        
	//read mag
	WriteReg(reg_set_9250.i2c_slv0_addr, AK8963_I2C_ADDR|READ_FLAG);
	WriteReg(reg_set_9250.i2c_slv0_reg, AK8963_HXL);
	WriteReg(reg_set_9250.i2c_slv0_ctrl, 0x87); //read 6bytes+ST2

	//process mpu9250 data
	MPU9250_DATA->ACC_OUT[0] = (((short)rxbuf[0])<<8)+((short)rxbuf[1]);      
	MPU9250_DATA->ACC_OUT[1] = (((short)rxbuf[2])<<8)+((short)rxbuf[3]);	
        MPU9250_DATA->ACC_OUT[2] = (((short)rxbuf[4])<<8)+((short)rxbuf[5]);
	MPU9250_DATA->TEMP_OUT = (((short)rxbuf[6])<<8)+((short)rxbuf[7]);
	MPU9250_DATA->GYRO_OUT[0] = (((short)rxbuf[8])<<8)+((short)rxbuf[9]);
	MPU9250_DATA->GYRO_OUT[1] = (((short)rxbuf[10])<<8)+((short)rxbuf[11]);
	MPU9250_DATA->GYRO_OUT[2] = (((short)rxbuf[12])<<8)+((short)rxbuf[13]);
      
        MPU9250_DATA->ACC_fl[0] = acc_scale*(float)(MPU9250_DATA->ACC_OUT[0]);
        MPU9250_DATA->ACC_fl[1] = acc_scale*(float)(MPU9250_DATA->ACC_OUT[1]);
        MPU9250_DATA->ACC_fl[2] = acc_scale*(float)(MPU9250_DATA->ACC_OUT[2]);
        MPU9250_DATA->TEMP_fl = ((float)(MPU9250_DATA->TEMP_OUT)-21.0)/333.87+21.0;//((float)(MPU9250_DATA->TEMP_OUT))/340.0+36.53;//
        MPU9250_DATA->GYRO_fl[0] = gyro_scale*(float)(MPU9250_DATA->GYRO_OUT[0]);
        MPU9250_DATA->GYRO_fl[1] = gyro_scale*(float)(MPU9250_DATA->GYRO_OUT[1]);
        MPU9250_DATA->GYRO_fl[2] = gyro_scale*(float)(MPU9250_DATA->GYRO_OUT[2]);
	
	//process mag data    
	for (i = 0; i < 100; i++) {
		  asm("    nop");
	} 
	ReadReg(reg_set_9250.i2c_ext_sens_data0, 6);

	for(i=0; i<3; i++) {
		MPU9250_DATA->Mag_out[i]=((short)rxbuf[i*2+1]<<8)+((short)rxbuf[i*2]);
		data=(float)(MPU9250_DATA->Mag_out[i]);
		MPU9250_DATA->Magnetometer[i]=data*Magnetometer_ASA[i];
	}

	//printf("%d, %d, %d, %4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,  (%8.2f,%8.2f,%8.2f)\n\r",MPU9250_DATA->Mag_out[0],MPU9250_DATA->Mag_out[1],MPU9250_DATA->Mag_out[2],MPU9250_DATA->ACC_fl[0],MPU9250_DATA->ACC_fl[1],MPU9250_DATA->ACC_fl[2], MPU9250_DATA->TEMP_fl,MPU9250_DATA->GYRO_fl[0],MPU9250_DATA->GYRO_fl[1],MPU9250_DATA->GYRO_fl[2], MPU9250_DATA->Magnetometer[0],MPU9250_DATA->Magnetometer[1],MPU9250_DATA->Magnetometer[2]);
}
